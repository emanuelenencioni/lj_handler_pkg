#ifndef LJ_HANDLER_HPP
#define LJ_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <LabJackM.h>
#include <string>
#include <vector>

class LJHandlerNode : public rclcpp::Node
{
public:
  LJHandlerNode();
  ~LJHandlerNode();

private:
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void set_steering_angle(double steering_angle_deg);
  int read_nominal_voltages(double& nom_vs_master, double& nom_vs_slave);
  
  // Member variables
  int handle_;
  std::string nominal_vs_master_pin_;
  std::string nominal_vs_slave_pin_;
  double min_perc_;
  double max_perc_;
  double max_steering_angle_;
  double k_steering_; // Steering gain factor
  std::vector<std::string> dac_names_;
  
  // Timeout for pose messages
  double pose_timeout_sec_;
  rclcpp::Time last_pose_time_;
  
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  
  static constexpr int INITIAL_ERR_ADDRESS = -1;
};

#endif // LJ_HANDLER_HPP