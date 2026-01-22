#include "my_robot_controllers/cartesian_velocity_controller.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <kdl/chain.hpp> 
#include <algorithm> // std::clamp

namespace my_robot_controllers
{

CartesianVelocityController::CartesianVelocityController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn CartesianVelocityController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::string>("base_link", "base_link");
    auto_declare<std::string>("end_effector_link", "tool0");
    auto_declare<std::string>("robot_description", "");
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  base_link_ = get_node()->get_parameter("base_link").as_string();
  end_effector_link_ = get_node()->get_parameter("end_effector_link").as_string();
  std::string robot_desc = get_node()->get_parameter("robot_description").as_string();

  if (joint_names_.empty() || robot_desc.empty()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromString(robot_desc, kdl_tree)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  
  KDL::Chain kdl_chain;
  if (!kdl_tree.getChain(base_link_, end_effector_link_, kdl_chain)) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Init Modules
  kinematics_solver_ = std::make_shared<algo::RobotKinematics>(
      kdl_tree, base_link_, end_effector_link_);
  fk_solver_ = std::make_shared<algo::FkSolver>(
      kdl_tree, base_link_, end_effector_link_);

  // Init SHM
  shm_manager_ = std::make_shared<shm::ShmManager>(shm::ROBOT_SHM_NAME, true);
  if (!shm_manager_->init()) return controller_interface::CallbackReturn::ERROR;

  // Resize Variables
  auto n_joints = kinematics_solver_->getNrOfJoints();
  q_current_.resize(n_joints);
  q_dot_cmd_.resize(n_joints);
  q_dot_cmd_.setZero();
  v_target_.setZero();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/velocity");
  }
  config.names.push_back("gripper_right_joint/position");

  return config;
}

controller_interface::InterfaceConfiguration CartesianVelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/position");
  }
  return config;
}

controller_interface::CallbackReturn CartesianVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  q_dot_cmd_.setZero();
  v_target_.setZero();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & interface : command_interfaces_) interface.set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianVelocityController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1. Đọc Feedback
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q_current_(i) = state_interfaces_[i].get_value();
  }

  auto* shm_data = shm_manager_->get();
  bool shm_active = false;
  int current_mode = shm::MODE_IDLE;
  double target_gripper_pos = 0.0;

  // 2. Xử lý SHM
  KDL::Frame current_pose;
  if (shm_data && fk_solver_->calculate(q_current_, current_pose)) {
      // Ghi Feedback ra SHM
      shm_data->ee_pos[0] = current_pose.p.x();
      shm_data->ee_pos[1] = current_pose.p.y();
      shm_data->ee_pos[2] = current_pose.p.z();
      double r, p, y; current_pose.M.GetRPY(r, p, y);
      shm_data->ee_rpy[0] = r; shm_data->ee_rpy[1] = p; shm_data->ee_rpy[2] = y;
      for(size_t i=0; i<6; ++i) shm_data->joint_pos[i] = q_current_(i);

      // Đọc Lệnh
      if (shm_data->cmd_active) {
          shm_active = true;
          current_mode = shm_data->control_mode;
          target_gripper_pos = shm_data->cmd_gripper;

          // =========================================================
          // LOGIC XỬ LÝ THEO TỪNG CHẾ ĐỘ (MODE)
          // =========================================================
          
          // --- MODE 1: POSE CONTROL (P-Controller) ---
          if (current_mode == shm::MODE_CARTESIAN_POSE) {
              KDL::Vector target_p(shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2]);
              KDL::Rotation target_M = KDL::Rotation::RPY(shm_data->target_rpy[0], shm_data->target_rpy[1], shm_data->target_rpy[2]);
              KDL::Twist error = KDL::diff(current_pose, KDL::Frame(target_M, target_p));

              double Kp_lin = 4.0, Kp_ang = 2.0;
              for(int i=0; i<3; ++i) {
                  v_target_(i)   = std::clamp(error.vel(i) * Kp_lin, -0.5, 0.5);
                  v_target_(i+3) = std::clamp(error.rot(i) * Kp_ang, -1.0, 1.0);
              }
              if (error.vel.Norm() < 0.001 && error.rot.Norm() < 0.01) v_target_.setZero();
          }
          
          // --- MODE 2: TRAJECTORY (Direct Cartesian Velocity) ---
          else if (current_mode == shm::MODE_TRAJECTORY) {
              v_target_(0) = shm_data->traj_vel_linear[0];
              v_target_(1) = shm_data->traj_vel_linear[1];
              v_target_(2) = shm_data->traj_vel_linear[2];
              v_target_(3) = shm_data->traj_vel_angular[0];
              v_target_(4) = shm_data->traj_vel_angular[1];
              v_target_(5) = shm_data->traj_vel_angular[2];
          }

          // --- MODE 3: JOINT MANUAL ---
          else if (current_mode == shm::MODE_JOINT_MANUAL) {
              // Đọc trực tiếp vận tốc khớp từ SHM vào q_dot_cmd_
              for(int i=0; i<6; ++i) {
                  q_dot_cmd_(i) = shm_data->manual_joint_vel[i];
              }
          }
      }
  }

  // 3. QUYẾT ĐỊNH GỬI LỆNH
  
  if (!shm_active) {
      // Nếu mất kết nối -> Dừng
      for (size_t i = 0; i < joint_names_.size(); ++i) command_interfaces_[i].set_value(0.0);
  }
  else {
      // NẾU LÀ MODE MANUAL: BỎ QUA IK SOLVER
      if (current_mode == shm::MODE_JOINT_MANUAL) {
          // Gửi thẳng q_dot_cmd_ (đã gán ở trên) xuống Hardware
          for (size_t i = 0; i < joint_names_.size(); ++i) {
              command_interfaces_[i].set_value(q_dot_cmd_(i));
          }
      }
      // NẾU LÀ MODE POSE HOẶC TRAJECTORY: PHẢI QUA IK SOLVER
      else {
          // Tính toán q_dot_cmd_ từ v_target_
          if (kinematics_solver_->convertCartToJnt(q_current_, v_target_, q_dot_cmd_)) {
              for (size_t i = 0; i < joint_names_.size(); ++i) {
                  command_interfaces_[i].set_value(q_dot_cmd_(i));
              }
          } else {
              // Lỗi IK -> Dừng
              for (size_t i = 0; i < joint_names_.size(); ++i) command_interfaces_[i].set_value(0.0);
          }
      }
  }

  // 4. Gửi lệnh Gripper
  if (shm_active && command_interfaces_.size() > 6) {
      command_interfaces_[6].set_value(target_gripper_pos);
  }

  return controller_interface::return_type::OK;
}

void CartesianVelocityController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Deprecated
}

} 

PLUGINLIB_EXPORT_CLASS(
  my_robot_controllers::CartesianVelocityController, controller_interface::ControllerInterface)