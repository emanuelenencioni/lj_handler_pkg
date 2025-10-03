#include "lj_handler_pkg/lj_handler.hpp"
#include <cmath>
#include <tf2/exceptions.h>

LJHandlerNode::LJHandlerNode() : Node("lj_handler")
{
  // Declare parameters
  this->declare_parameter<std::string>("nominal_vs_master_pin", "AIN10");
  this->declare_parameter<std::string>("nominal_vs_slave_pin", "AIN12");
  this->declare_parameter<double>("min_perc", 0.15);
  this->declare_parameter<double>("max_perc", 0.85);
  this->declare_parameter<double>("max_steering_angle", 30.0); // degrees
  this->declare_parameter<std::string>("target_frame", "base_link");
  this->declare_parameter<std::string>("source_frame", "front_vehicle");
  this->declare_parameter<double>("update_rate", 20.0); // Hz
  this->declare_parameter<double>("transform_timeout", 0.5); // seconds
  
  // Get parameters
  nominal_vs_master_pin_ = this->get_parameter("nominal_vs_master_pin").as_string();
  nominal_vs_slave_pin_ = this->get_parameter("nominal_vs_slave_pin").as_string();
  min_perc_ = this->get_parameter("min_perc").as_double();
  max_perc_ = this->get_parameter("max_perc").as_double();
  max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
  target_frame_ = this->get_parameter("target_frame").as_string();
  source_frame_ = this->get_parameter("source_frame").as_string();
  transform_timeout_sec_ = this->get_parameter("transform_timeout").as_double();
  
  double update_rate = this->get_parameter("update_rate").as_double();
  
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
  
  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Create timer for periodic transform lookup
  auto timer_period = std::chrono::duration<double>(1.0 / update_rate);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&LJHandlerNode::timer_callback, this));
  
  RCLCPP_INFO(this->get_logger(), "LJ Handler Node initialized");
  RCLCPP_INFO(this->get_logger(), "Looking for transform from '%s' to '%s'", 
              target_frame_.c_str(), source_frame_.c_str());
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

void LJHandlerNode::timer_callback()
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  
  try {
    // Look up the transform with a timeout
    transform_stamped = tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero,
                                                        tf2::durationFromSec(0.05)); // Short timeout for lookup
    
    // Check if transform is too old
    rclcpp::Time transform_time(transform_stamped.header.stamp);
    double age = (this->get_clock()->now() - transform_time).seconds();
    
    if (age > transform_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Transform is too old (%.2f s), setting steering to straight", age);
      set_steering_angle(0.0);
      return;
    }
    
    // Extract y-position from transform
    double y_position = transform_stamped.transform.translation.y;
    
    // Simple proportional control: map y position to steering angle
    double k_steering = 10.0; // Gain factor (adjust as needed)
    double steering_angle = std::atan(k_steering * y_position) * 180.0 / M_PI;
    
    // Clamp to max steering angle
    steering_angle = std::max(-max_steering_angle_, std::min(max_steering_angle_, steering_angle));
    
    RCLCPP_DEBUG(this->get_logger(), "Y position: %.3f m -> Steering angle: %.2f deg", 
                 y_position, steering_angle);
    
    // Apply steering
    set_steering_angle(steering_angle);
    
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Could not get transform: %s. Setting steering to straight.", ex.what());
    set_steering_angle(0.0);
  }
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
  
  // Map steering angle (-max_steering_angle to +max_steering_angle) to voltage ratio (0.0 to 1.0)
  // -max_angle -> 0.0, 0 -> 0.5, +max_angle -> 1.0
  double master1_ratio = (steering_angle_deg + max_steering_angle_) / (2.0 * max_steering_angle_);
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
  
  double steering_percentage = (master1_ratio * 200.0) - 100.0;
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
