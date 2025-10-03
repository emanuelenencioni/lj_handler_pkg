#include "lj_handler_pkg/lj_handler.hpp"
#include <cmath>

LJHandlerNode::LJHandlerNode() : Node("lj_handler")
{
  // Declare parameters
  this->declare_parameter<std::string>("nominal_vs_master_pin", "AIN10");
  this->declare_parameter<std::string>("nominal_vs_slave_pin", "AIN12");
  this->declare_parameter<double>("min_perc", 0.15);
  this->declare_parameter<double>("max_perc", 0.85);
  this->declare_parameter<double>("max_steering_angle", 30.0); // degrees
  this->declare_parameter<double>("k_steering", 10.0); // Steering gain factor
  this->declare_parameter<double>("pose_timeout", 0.5); // seconds
  this->declare_parameter<std::string>("pose_topic", "/tag_pose"); // Topic name
  
  // Get parameters
  nominal_vs_master_pin_ = this->get_parameter("nominal_vs_master_pin").as_string();
  nominal_vs_slave_pin_ = this->get_parameter("nominal_vs_slave_pin").as_string();
  min_perc_ = this->get_parameter("min_perc").as_double();
  max_perc_ = this->get_parameter("max_perc").as_double();
  max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
  k_steering_ = this->get_parameter("k_steering").as_double();
  pose_timeout_sec_ = this->get_parameter("pose_timeout").as_double();
  std::string pose_topic = this->get_parameter("pose_topic").as_string();
  
  // Initialize last pose time to current time
  last_pose_time_ = this->get_clock()->now();
  
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
  
  // DAC output names: TDAC0=Master1, TDAC1=Master2, TDAC2=Slave1, TDAC3=Slave2
  dac_names_ = {"TDAC0", "TDAC1", "TDAC2", "TDAC3"};
  
  // Create subscription to pose topic
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    pose_topic, 10,
    std::bind(&LJHandlerNode::pose_callback, this, std::placeholders::_1));
  
  
  RCLCPP_INFO(this->get_logger(), "LJ Handler Node initialized");
  RCLCPP_INFO(this->get_logger(), "Subscribing to pose topic: '%s'", pose_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Steering gain factor: %.2f", k_steering_);
}

LJHandlerNode::~LJHandlerNode()
{
  // Set steering to center before closing
  RCLCPP_INFO(this->get_logger(), "Setting steering to center before shutdown");
  set_steering_angle(0.0);
  
  // Close LabJack handle
  if (handle_ > 0) {
    LJM_Close(handle_);
    RCLCPP_INFO(this->get_logger(), "LabJack connection closed");
  }
}

void LJHandlerNode::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // Update last pose time
  last_pose_time_ = this->get_clock()->now();
  
  // Extract y-position from pose (covariance is ignored for now)
  double y_position = msg->pose.pose.position.y;
  double x_position = msg->pose.pose.position.x;
  
  // Calculate steering angle using arctangent
  // This gives the angle needed to point towards the target
  double steering_angle = std::atan2(y_position, x_position) * 180.0 / M_PI;
  
  // Alternative: Simple proportional control based on y-position only
  // double steering_angle = std::atan(k_steering_ * y_position) * 180.0 / M_PI;
  
  // Clamp to max steering angle
  steering_angle = std::max(-max_steering_angle_, std::min(max_steering_angle_, steering_angle));
  
  RCLCPP_DEBUG(this->get_logger(), "Pose: (x=%.3f, y=%.3f) -> Steering angle: %.2f deg", 
               x_position, y_position, steering_angle);
  
  // Apply steering
  set_steering_angle(steering_angle);
}

void LJHandlerNode::set_steering_angle(double steering_angle_deg)
{
  // Read nominal voltages
  double nom_vs_master, nom_vs_slave;
  int err = read_nominal_voltages(nom_vs_master, nom_vs_slave);
  if (err != LJME_NOERROR) {
    RCLCPP_WARN(this->get_logger(), "Failed to read nominal voltages");
    return;
  }
  
  // Map steering angle to voltage ratio
  // -max_angle -> 0.0 (full left)
  //  0° -> 0.5 (straight/center)
  // +max_angle -> 1.0 (full right)
  double master1_ratio = 0.5 + (steering_angle_deg / (2.0 * max_steering_angle_));
  master1_ratio = std::max(0.0, std::min(1.0, master1_ratio)); // Clamp to [0, 1]
  
  // Calculate voltage limits
  double master_max_out_v = nom_vs_master * max_perc_;
  double master_min_out_v = nom_vs_master * min_perc_;
  double slave_min_out_v = nom_vs_slave * min_perc_;
  double slave_max_out_v = nom_vs_slave * max_perc_;
  
  // First pair (M1, S1)
  double master1_voltage = nom_vs_master * master1_ratio;
  master1_voltage = std::max(master_min_out_v, std::min(master_max_out_v, master1_voltage));
  
  double slave1_voltage = nom_vs_slave * (1.0 - master1_ratio);
  slave1_voltage = std::max(slave_min_out_v, std::min(slave_max_out_v, slave1_voltage));
  
  // Second pair (M2, S2) works in opposition
  double master2_ratio = 1.0 - master1_ratio;
  double master2_voltage = nom_vs_master * master2_ratio;
  master2_voltage = std::max(master_min_out_v, std::min(master_max_out_v, master2_voltage));
  
  double slave2_voltage = nom_vs_slave - slave1_voltage;
  slave2_voltage = std::max(slave_min_out_v, std::min(slave_max_out_v, slave2_voltage));
  
  // Write voltages to DACs
  double voltages[4] = {master1_voltage, master2_voltage, slave1_voltage, slave2_voltage};
  
  int errorAddress = INITIAL_ERR_ADDRESS;
  const char* dac_names_arr[4] = {dac_names_[0].c_str(), dac_names_[1].c_str(), 
                                   dac_names_[2].c_str(), dac_names_[3].c_str()};
  err = LJM_eWriteNames(handle_, 4, dac_names_arr, voltages, &errorAddress);
  
  if (err != LJME_NOERROR) {
    char errName[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, errName);
    RCLCPP_WARN(this->get_logger(), "Error writing to DACs: %s (address: %d)", errName, errorAddress);
    return;
  }
  
  double steering_percentage = (master1_ratio - 0.5) * 200.0;
  RCLCPP_INFO(this->get_logger(), 
              "Angle: %.1f° (%.1f%%) -> M1: %.2fV, S1: %.2fV | M2: %.2fV, S2: %.2fV",
              steering_angle_deg, steering_percentage,
              master1_voltage, slave1_voltage, master2_voltage, slave2_voltage);
}

int LJHandlerNode::read_nominal_voltages(double& nom_vs_master, double& nom_vs_slave)
{
  const char* names[2] = {nominal_vs_master_pin_.c_str(), nominal_vs_slave_pin_.c_str()};
  double values[2];
  int errorAddress = INITIAL_ERR_ADDRESS;
  
  int err = LJM_eReadNames(handle_, 2, names, values, &errorAddress);
  
  if (err == LJME_NOERROR) {
    nom_vs_master = values[0];
    nom_vs_slave = values[1];
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
