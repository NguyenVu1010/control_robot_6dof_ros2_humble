#ifndef MY_ROBOT_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"

// 1. Include thư viện Algo
#include "my_robot_controllers/algo/robot_kinematics.hpp"
// 2. Include thư viện FK (để tính vị trí gửi ra SHM)
#include "my_robot_controllers/algo/fk_solver.hpp"
// 3. Include thư viện SHM
#include "my_robot_controllers/shm/shm_manager.hpp"

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

  // Modules Toán học
  std::shared_ptr<algo::RobotKinematics> kinematics_solver_;
  std::shared_ptr<algo::FkSolver> fk_solver_; // Mới thêm

  // Module Giao tiếp Shared Memory
  std::shared_ptr<shm::ShmManager> shm_manager_;

  // Biến lưu trạng thái
  KDL::JntArray q_current_;
  algo::VectorXd q_dot_cmd_;
  algo::Vector6d v_target_;

  // Real-time Buffer (Topic backup)
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> cmd_vel_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};

} 

#endif