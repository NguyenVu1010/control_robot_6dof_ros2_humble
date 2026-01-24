#ifndef MY_ROBOT_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"

// 1. Module SHM (Nội bộ)
#include "my_robot_controllers/shm/shm_manager.hpp"

// 2. Thư viện Động học (Package: simple_kinematics_lib)
#include "simple_kinematics_lib/kinematics_core.hpp"

// 3. Thư viện Quỹ đạo (Package: simple_trajectory_lib)
#include "simple_trajectory_lib/trajectory_generator.hpp"

namespace my_robot_controllers
{

class CartesianVelocityController : public controller_interface::ControllerInterface
{
public:
  CartesianVelocityController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- THAM SỐ CẤU HÌNH ---
  std::vector<std::string> joint_names_;
  std::string base_link_;
  std::string end_effector_link_;
  
  // --- CÁC MODULE CHỨC NĂNG ---
  // 1. Quản lý Shared Memory
  std::shared_ptr<shm::ShmManager> shm_manager_;

  // 2. Lõi tính toán Động học (FK, IK)
  std::shared_ptr<srk::KinematicsCore> kinematics_core_;

  // 3. Bộ sinh quỹ đạo (Trajectory Generator)
  std::shared_ptr<stl::TrajectoryGenerator> traj_gen_;

  // --- BIẾN TRẠNG THÁI (Dùng kiểu của thư viện mới) ---
  srk::JntArray q_current_;   // Vị trí khớp hiện tại
  srk::JntArray q_dot_cmd_;   // Vận tốc khớp lệnh
  srk::Vector6d v_target_;    // Vận tốc Cartesian mong muốn (v, w)
  
  // --- ROS 2 TOPIC (Dự phòng/Fallback) ---
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> cmd_vel_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};

}  // namespace my_robot_controllers

#endif  // MY_ROBOT_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_