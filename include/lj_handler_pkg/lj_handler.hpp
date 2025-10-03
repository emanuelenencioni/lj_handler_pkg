#ifndef LJ_HANDLER_HPP
#define LJ_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <LabJackM.h>
#include <string>
#include <vector>

class LJHandlerNode : public rclcpp::Node
{
public:
  LJHandlerNode();
  ~LJHandlerNode();

private:
  void timer_callback();
  void set_steering_angle(double steering_angle_deg);
  int read_nominal_voltages(double& nom_vs_master, double& nom_vs_slave);
  
  // Member variables
  int handle_;
  std::string nominal_vs_master_pin_;
  std::string nominal_vs_slave_pin_;
  double min_perc_;
  double max_perc_;
  double max_steering_angle_;
  std::vector<std::string> dac_names_;
  
  // TF2 variables
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string target_frame_;
  std::string source_frame_;
  double transform_timeout_sec_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  static constexpr int INITIAL_ERR_ADDRESS = -1;
};

#endif // LJ_HANDLER_HPP