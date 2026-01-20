#ifndef MY_ROBOT_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
// Include Wrapper
#include "my_robot_controllers/algo/robot_kinematics.hpp"

namespace my_robot_controllers
{

class CartesianVelocityController : public controller_interface::ControllerInterface
{
public:
  CartesianVelocityController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;
  std::string base_link_;
  std::string end_effector_link_;

  // Algorithm Module
  std::shared_ptr<algo::RobotKinematics> kinematics_solver_;

  // Pre-allocated buffers
  KDL::JntArray q_current_;
  algo::VectorXd q_dot_cmd_;
  algo::Vector6d v_target_;

  // Real-time Buffer (Lock-free data transfer)
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> cmd_vel_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};

}  // namespace my_robot_controllers

#endif