#ifndef MY_ROBOT_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "my_robot_controllers/shm/shm_manager.hpp"
#include "simple_kinematics_lib/kinematics_core.hpp"

namespace my_robot_controllers {
    class Visualizer;
    class PathTracker;
    class JointPathPlannerOMPL; // Planner OMPL mới
}

namespace my_robot_controllers {

class CartesianVelocityController : public controller_interface::ControllerInterface {
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
  // API cũ và Module Helpers
  std::vector<std::string> joint_names_;
  std::string base_link_;
  std::string end_effector_link_;

  std::shared_ptr<shm::ShmManager> shm_manager_;
  std::shared_ptr<srk::KinematicsCore> kinematics_core_;
  std::shared_ptr<Visualizer> visualizer_;
  std::shared_ptr<PathTracker> path_tracker_;
  std::shared_ptr<JointPathPlannerOMPL> planner_;

  srk::JntArray q_current_;
  srk::JntArray q_dot_cmd_;
  bool prev_cmd_active_ = false;
  std::vector<double> joint_limits_min_;
  std::vector<double> joint_limits_max_;
  Eigen::Vector3d obs_center_;
  double obs_radius_;
};

} // namespace my_robot_controllers

#endif