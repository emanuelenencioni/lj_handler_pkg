#include "lj_handler_pkg/lj_handler.hpp"
#include <cmath>

float melex_max_deg = atan(2555.0/4500.0) * 180.0 / M_PI; // approx 29.74 degrees

LJHandlerNode::LJHandlerNode() : Node("lj_handler")
{
  // Declare parameters for pin assignments
  this->declare_parameter<std::string>("nominal_vs_steer_M_pin", "AIN10");
  this->declare_parameter<std::string>("nominal_vs_steer_S_pin", "AIN12");
  this->declare_parameter<std::string>("nominal_vs_accbrake_M_pin", "AIN6");
  this->declare_parameter<std::string>("nominal_vs_accbrake_S_pin", "AIN8");
  
  // Declare voltage range parameters
  this->declare_parameter<double>("input_voltage_min", 0.1);
  this->declare_parameter<double>("input_voltage_max", 5.26);
  
  // Declare percentage limits for steering
  this->declare_parameter<double>("steering_min_perc", 0.15);
  this->declare_parameter<double>("steering_max_perc", 0.85);
  
  // Declare percentage limits for throttle/brake
  this->declare_parameter<double>("throttle_min_perc", 0.23);
  this->declare_parameter<double>("throttle_max_perc", 0.77);
  
  // Declare steering parameters
  this->declare_parameter<double>("max_steering_angle", melex_max_deg); // degrees
  this->declare_parameter<double>("k_steering", 5.0); // Steering gain factor
  
  // Declare timeout parameters
  this->declare_parameter<double>("pose_timeout", 0.5); // seconds
  this->declare_parameter<double>("throttle_timeout", 0.5); // seconds
  this->declare_parameter<double>("safety_check_period", 0.1); // seconds
  
  // Declare topic parameters //TODO: set to actual topics
  this->declare_parameter<std::string>("steering_topic", "/follower/steering_cmd");
  this->declare_parameter<std::string>("throttle_topic", "/follower/acceleration_cmd");
  
  // Get parameters
  nominal_vs_steer_master_pin_ = this->get_parameter("nominal_vs_steer_M_pin").as_string();
  nominal_vs_steer_slave_pin_ = this->get_parameter("nominal_vs_steer_S_pin").as_string();
  nominal_vs_accbrake_master_pin_ = this->get_parameter("nominal_vs_accbrake_M_pin").as_string();
  nominal_vs_accbrake_slave_pin_ = this->get_parameter("nominal_vs_accbrake_S_pin").as_string();
  
  input_voltage_min_ = this->get_parameter("input_voltage_min").as_double();
  input_voltage_max_ = this->get_parameter("input_voltage_max").as_double();
  center_voltage_ = (input_voltage_max_ + input_voltage_min_) / 2.0;
  
  steering_min_perc_ = this->get_parameter("steering_min_perc").as_double();
  steering_max_perc_ = this->get_parameter("steering_max_perc").as_double();
  
  throttle_min_perc_ = this->get_parameter("throttle_min_perc").as_double();
  throttle_max_perc_ = this->get_parameter("throttle_max_perc").as_double();
  
  max_steering_angle_ = melex_max_deg;
  k_steering_ = this->get_parameter("k_steering").as_double();
  
  steering_timeout_sec_ = this->get_parameter("pose_timeout").as_double();
  throttle_timeout_sec_ = this->get_parameter("throttle_timeout").as_double();
  double safety_check_period = this->get_parameter("safety_check_period").as_double();

  std::string steering_topic = this->get_parameter("steering_topic").as_string();
  std::string throttle_topic = this->get_parameter("throttle_topic").as_string();
  
  // Initialize last message times to current time
  last_steering_time_ = this->get_clock()->now();
  last_throttle_time_ = this->get_clock()->now();
  
  // Open LabJack T7
  int err = LJM_Open(LJM_dtT7, LJM_ctUSB, "ANY", &handle_);
  if (err != LJME_NOERROR) {
    char errName[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, errName);
    RCLCPP_ERROR(this->get_logger(), "Failed to open LabJack T7: %s", errName);
    rclcpp::shutdown();
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "LabJack T7 opened successfully");
  
  // DAC output names
  // Steering: TDAC0=Master1, TDAC1=Master2, TDAC2=Slave1, TDAC3=Slave2
  // Throttle: TDAC4=Master1, TDAC5=Master2, TDAC6=Slave1, TDAC7=Slave2
  steering_dac_names_ = {"TDAC0", "TDAC1", "TDAC2", "TDAC3"};
  throttle_dac_names_ = {"TDAC4", "TDAC5", "TDAC6", "TDAC7"};

  // Create subscription to steering topic
  steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    steering_topic, 10,
    std::bind(&LJHandlerNode::steering_callback, this, std::placeholders::_1));
  
  // Create subscription to throttle topic
  throttle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    throttle_topic, 10,
    std::bind(&LJHandlerNode::throttle_callback, this, std::placeholders::_1));
  
  // Create safety timer to check for timeouts
  safety_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(safety_check_period),
    std::bind(&LJHandlerNode::check_safety_timeout, this));
  
  RCLCPP_INFO(this->get_logger(), "LJ Handler Node initialized");
  RCLCPP_INFO(this->get_logger(), "Subscribing to steering topic: '%s'", steering_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Subscribing to throttle topic: '%s'", throttle_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Max steering angle: %.2f°", max_steering_angle_);
  RCLCPP_INFO(this->get_logger(), "Throttle timeout: %.2fs", throttle_timeout_sec_);
  RCLCPP_INFO(this->get_logger(), "Safety check period: %.2fs", safety_check_period);
}

LJHandlerNode::~LJHandlerNode()
{
  // Set steering to center and throttle to zero before closing
  RCLCPP_INFO(this->get_logger(), "Setting steering to center and throttle to zero");
  set_steering_angle(0.0);
  set_throttle_brake(0.0);
  
  // Close LabJack handle
  if (handle_ > 0) {
    LJM_Close(handle_);
    RCLCPP_INFO(this->get_logger(), "LabJack connection closed");
  }
}

void LJHandlerNode::steering_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  // Update last steering time
  last_steering_time_ = this->get_clock()->now();

  // Get steering value in radians and convert to degrees
  double steering_deg = msg->data * 180.0 / M_PI;
  
  // Clamp to max steering angle
  steering_deg = std::max(-max_steering_angle_, std::min(max_steering_angle_, steering_deg));
  
  RCLCPP_DEBUG(this->get_logger(), "Steering command: %.2f rad (%.2f deg)", 
               msg->data, steering_deg);

  // Apply steering
  set_steering_angle(steering_deg);
}

void LJHandlerNode::throttle_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  // Update last throttle time
  last_throttle_time_ = this->get_clock()->now();
  
  // Get throttle value (expected range: -1.0 to 1.0)
  // -1.0 = full brake, 0.0 = neutral, 1.0 = full throttle
  double throttle_value = msg->data;
  
  // Clamp to valid range
  throttle_value = std::max(-1.0, std::min(1.0, throttle_value));
  
  RCLCPP_DEBUG(this->get_logger(), "Throttle command: %.2f", throttle_value);
  
  // Apply throttle/brake
  set_throttle_brake(throttle_value);
}

void LJHandlerNode::check_safety_timeout()
{
  rclcpp::Time current_time = this->get_clock()->now();
  
  // Check throttle timeout
  double throttle_time_diff = (current_time - last_throttle_time_).seconds();
  if (throttle_time_diff > throttle_timeout_sec_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Throttle timeout (%.2fs since last message) - setting to zero", 
                        throttle_time_diff);
    set_throttle_brake(0.0);
  }

  // Optional: Check steering timeout
  double steering_time_diff = (current_time - last_steering_time_).seconds();
  if (steering_time_diff > steering_timeout_sec_) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Steering timeout (%.2fs since last message)", steering_time_diff);
    // Optionally set steering to center on timeout
    // set_steering_angle(0.0);
  }
}

void LJHandlerNode::set_steering_angle(double steering_angle_deg)
{
  // Map steering angle to voltage
  // -max_angle -> input_voltage_min_, 0° -> center_voltage_, +max_angle -> input_voltage_max_
  double angle_ratio = (steering_angle_deg + max_steering_angle_) / (2.0 * max_steering_angle_);
  angle_ratio = std::max(0.0, std::min(1.0, angle_ratio)); // Clamp to [0, 1]
  
  double voltage_range = input_voltage_max_ - input_voltage_min_;
  double desired_voltage = input_voltage_min_ + (angle_ratio * voltage_range);
  
  // Read nominal voltages
  double nom_vs_steer_master, nom_vs_steer_slave, ph1, ph2;
  int err = read_nominal_voltages(nom_vs_steer_master, nom_vs_steer_slave, 
                                   ph1, ph2);
  if (err != LJME_NOERROR) {
    RCLCPP_WARN(this->get_logger(), "Failed to read nominal voltages for steering");
    return;
  }
    // Clamp the input voltage to the expected range
  desired_voltage = std::max(input_voltage_min_, std::min(input_voltage_max_, desired_voltage));
  
  // Normalize the desired voltage to a ratio from 0.0 to 1.0
  double master1_ratio = (desired_voltage - input_voltage_min_) / voltage_range;
  
  // Calculate voltage limits
  double master_max_out_v = nom_vs_steer_master * this->steering_max_perc_;
  double master_min_out_v = nom_vs_steer_master * this->steering_min_perc_;
  double slave_min_out_v = nom_vs_steer_slave * this->steering_min_perc_;
  double slave_max_out_v = nom_vs_steer_slave * this->steering_max_perc_;

  // First pair (M1, S1)
  double master1_voltage = nom_vs_steer_master * master1_ratio;
  master1_voltage = std::max(master_min_out_v, std::min(master_max_out_v, master1_voltage));
  
  double slave1_voltage = nom_vs_steer_slave * (1.0 - master1_ratio);
  slave1_voltage = std::max(slave_min_out_v, std::min(slave_max_out_v, slave1_voltage));
  
  // Second pair (M2, S2) works in opposition
  double master2_ratio = 1 - master1_ratio;
  double master2_voltage = nom_vs_steer_master * master2_ratio;
  master2_voltage = std::max(master_min_out_v, std::min(master_max_out_v, master2_voltage));

  double slave2_voltage = nom_vs_steer_slave * (1.0 - master2_ratio);
  slave2_voltage = std::max(slave_min_out_v, std::min(slave_max_out_v, slave2_voltage));
  
  // Write voltages to DACs
  double voltages[4] = {master1_voltage, master2_voltage, slave1_voltage, slave2_voltage};
  
  int errorAddress = INITIAL_ERR_ADDRESS;
  const char* dac_names_arr[4] = {steering_dac_names_[0].c_str(), steering_dac_names_[1].c_str(),
                                   steering_dac_names_[2].c_str(), steering_dac_names_[3].c_str()};
  err = LJM_eWriteNames(handle_, 4, dac_names_arr, voltages, &errorAddress);
  
  if (err != LJME_NOERROR) {
    char errName[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, errName);
    RCLCPP_WARN(this->get_logger(), "Error writing to %s DACs: %s (address: %d)", 
                "steering", errName, errorAddress);
    return;
  }
  
  double control_percentage = (master1_ratio * 200.0) - 100.0;
  RCLCPP_INFO(this->get_logger(),
              "%s: %.1f%% (%.2fV) -> M1: %.2fV, S1: %.2fV | M2: %.2fV, S2: %.2fV",
              "steering", control_percentage, desired_voltage,
              master1_voltage, slave1_voltage, master2_voltage, slave2_voltage);

}

void LJHandlerNode::set_throttle_brake(double throttle_value)
{
  // Map throttle value (-1.0 to 1.0) to voltage
  // -1.0 -> input_voltage_min_, 0.0 -> center_voltage_, 1.0 -> input_voltage_max_
  double throttle_ratio = (throttle_value + 1.0) / 2.0;
  throttle_ratio = std::max(0.0, std::min(1.0, throttle_ratio)); // Clamp to [0, 1]
  
  double voltage_range = input_voltage_max_ - input_voltage_min_;
  double desired_voltage = input_voltage_min_ + (throttle_ratio * voltage_range);
  
  // Read nominal voltages
  double ph1, ph2, nom_vs_accbrake_master, nom_vs_accbrake_slave;
  int err = read_nominal_voltages(ph1, ph2,
                                   nom_vs_accbrake_master, nom_vs_accbrake_slave);
  if (err != LJME_NOERROR) {
    RCLCPP_WARN(this->get_logger(), "Failed to read nominal voltages for throttle");
    return;
  }
  
    // Clamp the input voltage to the expected range
  desired_voltage = std::max(input_voltage_min_, std::min(input_voltage_max_, desired_voltage));
  
  // Normalize the desired voltage to a ratio from 0.0 to 1.0
  double master1_ratio = (desired_voltage - input_voltage_min_) / voltage_range;
  
  // Calculate voltage limits
  double master_max_out_v = nom_vs_accbrake_master * throttle_max_perc_;
  double master_min_out_v = nom_vs_accbrake_master * throttle_min_perc_;
  double slave_min_out_v = nom_vs_accbrake_slave * throttle_min_perc_;
  double slave_max_out_v = nom_vs_accbrake_slave * throttle_max_perc_;

  // First pair (M1, S1)
  double master1_voltage = nom_vs_accbrake_master * master1_ratio;
  master1_voltage = std::max(master_min_out_v, std::min(master_max_out_v, master1_voltage));
  
  double slave1_voltage = nom_vs_accbrake_slave * (1.0 - master1_ratio);
  slave1_voltage = std::max(slave_min_out_v, std::min(slave_max_out_v, slave1_voltage));
  
  // Second pair (M2, S2) works in opposition
  double master2_ratio = 1- master1_ratio;
  double master2_voltage = nom_vs_accbrake_master * master2_ratio;
  master2_voltage = std::max(master_min_out_v, std::min(master_max_out_v, master2_voltage));

  double slave2_voltage = nom_vs_accbrake_slave - slave1_voltage;
  slave2_voltage = std::max(slave_min_out_v, std::min(slave_max_out_v, slave2_voltage));
  
  // Write voltages to DACs
  double voltages[4] = {master1_voltage, master2_voltage, slave1_voltage, slave2_voltage};
  
  int errorAddress = INITIAL_ERR_ADDRESS;
  const char* dac_names_arr[4] = {throttle_dac_names_[0].c_str(), throttle_dac_names_[1].c_str(),
                                   throttle_dac_names_[2].c_str(), throttle_dac_names_[3].c_str()};
  err = LJM_eWriteNames(handle_, 4, dac_names_arr, voltages, &errorAddress);
  
  if (err != LJME_NOERROR) {
    char errName[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, errName);
    RCLCPP_WARN(this->get_logger(), "Error writing to %s DACs: %s (address: %d)", 
                "throttle", errName, errorAddress);
    return;
  }
  
  double control_percentage = (master1_ratio * 200.0) - 100.0;
  RCLCPP_INFO(this->get_logger(),
              "%s: %.1f%% (%.2fV) -> M1: %.2fV, S1: %.2fV | M2: %.2fV, S2: %.2fV",
              "throttle", control_percentage, desired_voltage,
              master1_voltage, slave1_voltage, master2_voltage, slave2_voltage);
}

void LJHandlerNode::set_control_axis(double desired_voltage,
                                     double nom_vs_accbrake_master,
                                     double nom_vs_slave,
                                     const std::vector<std::string>& dac_names,
                                     double min_perc,
                                     double max_perc,
                                     const std::string& axis_name, const bool opposition=true)
{
  // Clamp the input voltage to the expected range
  desired_voltage = std::max(input_voltage_min_, std::min(input_voltage_max_, desired_voltage));
  
  // Normalize the desired voltage to a ratio from 0.0 to 1.0
  double voltage_range = input_voltage_max_ - input_voltage_min_;
  double master1_ratio = (desired_voltage - input_voltage_min_) / voltage_range;
  
  // Calculate voltage limits
  double master_max_out_v = nom_vs_accbrake_master * max_perc;
  double master_min_out_v = nom_vs_accbrake_master * min_perc;
  double slave_min_out_v = nom_vs_slave * min_perc;
  double slave_max_out_v = nom_vs_slave * max_perc;
  
  // First pair (M1, S1)
  double master1_voltage = nom_vs_accbrake_master * master1_ratio;
  master1_voltage = std::max(master_min_out_v, std::min(master_max_out_v, master1_voltage));
  
  double slave1_voltage = nom_vs_slave * (1.0 - master1_ratio);
  slave1_voltage = std::max(slave_min_out_v, std::min(slave_max_out_v, slave1_voltage));
  
  // Second pair (M2, S2) works in opposition
  double master2_ratio = master1_ratio;
  if (opposition)
    master2_ratio = 1.0 - master1_ratio;

  double master2_voltage = nom_vs_accbrake_master * master2_ratio;
  master2_voltage = std::max(master_min_out_v, std::min(master_max_out_v, master2_voltage));
  
  double slave2_voltage = nom_vs_slave - slave1_voltage;
  slave2_voltage = std::max(slave_min_out_v, std::min(slave_max_out_v, slave2_voltage));
  
  // Write voltages to DACs
  double voltages[4] = {master1_voltage, master2_voltage, slave1_voltage, slave2_voltage};
  
  int errorAddress = INITIAL_ERR_ADDRESS;
  const char* dac_names_arr[4] = {dac_names[0].c_str(), dac_names[1].c_str(),
                                   dac_names[2].c_str(), dac_names[3].c_str()};
  int err = LJM_eWriteNames(handle_, 4, dac_names_arr, voltages, &errorAddress);
  
  if (err != LJME_NOERROR) {
    char errName[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, errName);
    RCLCPP_WARN(this->get_logger(), "Error writing to %s DACs: %s (address: %d)", 
                axis_name.c_str(), errName, errorAddress);
    return;
  }
  
  double control_percentage = (master1_ratio * 200.0) - 100.0;
  RCLCPP_INFO(this->get_logger(),
              "%s: %.1f%% (%.2fV) -> M1: %.2fV, S1: %.2fV | M2: %.2fV, S2: %.2fV",
              axis_name.c_str(), control_percentage, desired_voltage,
              master1_voltage, slave1_voltage, master2_voltage, slave2_voltage);
}

int LJHandlerNode::read_nominal_voltages(double& nom_vs_steer_master,
                                        double& nom_vs_steer_slave,
                                        double& nom_vs_accbrake_master,
                                        double& nom_vs_accbrake_slave)
{
  const char* names[4] = {
    nominal_vs_steer_master_pin_.c_str(),
    nominal_vs_steer_slave_pin_.c_str(),
    nominal_vs_accbrake_master_pin_.c_str(),
    nominal_vs_accbrake_slave_pin_.c_str()
  };
  double values[4];
  int errorAddress = INITIAL_ERR_ADDRESS;
  
  int err = LJM_eReadNames(handle_, 4, names, values, &errorAddress);
  
  if (err == LJME_NOERROR) {
    nom_vs_steer_master = values[0];
    nom_vs_steer_slave = values[1];
    nom_vs_accbrake_master = values[2];
    nom_vs_accbrake_slave = values[3];
  }
  
  return err;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LJHandlerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}