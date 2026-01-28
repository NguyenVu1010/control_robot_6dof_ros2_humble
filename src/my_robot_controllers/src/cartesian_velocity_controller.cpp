#include "my_robot_controllers/cartesian_velocity_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm> 
#include <cmath>     

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
  // 1. Load Parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  base_link_ = get_node()->get_parameter("base_link").as_string();
  end_effector_link_ = get_node()->get_parameter("end_effector_link").as_string();
  std::string robot_desc = get_node()->get_parameter("robot_description").as_string();

  if (joint_names_.empty() || robot_desc.empty()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // 2. Init Kinematics Core
  kinematics_core_ = std::make_shared<srk::KinematicsCore>();
  if (!kinematics_core_->init(robot_desc, base_link_, end_effector_link_)) {
      return controller_interface::CallbackReturn::ERROR;
  }

  // 3. Init Trajectory Generator
  traj_gen_ = std::make_shared<stl::TrajectoryGenerator>();

  // 4. Init Shared Memory
  shm_manager_ = std::make_shared<shm::ShmManager>(shm::ROBOT_SHM_NAME, true);
  if (!shm_manager_->init()) return controller_interface::CallbackReturn::ERROR;

  // 5. Resize Variables
  auto n_joints = kinematics_core_->getNrOfJoints();
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
  // Gripper là interface thứ 7 (index 6)
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

controller_interface::CallbackReturn CartesianVelocityController::on_activate(const rclcpp_lifecycle::State &)
{
  q_dot_cmd_.setZero();
  v_target_.setZero();
  if (traj_gen_) traj_gen_->stop();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & interface : command_interfaces_) interface.set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianVelocityController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // 1. Feedback
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q_current_(i) = state_interfaces_[i].get_value();
  }

  auto* shm_data = shm_manager_->get();
  if (!shm_data) return controller_interface::return_type::OK;

  bool shm_active = false;
  double dt = period.seconds();

  // 2. GHI FEEDBACK & LẤY VỊ TRÍ HIỆN TẠI
  srk::Frame current_pose;
  if (kinematics_core_->solveFK(q_current_, current_pose)) {
      Eigen::Vector3d pos = current_pose.translation();
      shm_data->ee_pos[0] = pos.x(); shm_data->ee_pos[1] = pos.y(); shm_data->ee_pos[2] = pos.z();
      
      // Sử dụng Quat để tránh lỗi suy biến Euler khi tính toán nhưng vẫn ghi RPY ra SHM cho UI
      Eigen::Vector3d rpy = current_pose.rotation().eulerAngles(0, 1, 2); 
      shm_data->ee_rpy[0] = rpy[0]; shm_data->ee_rpy[1] = rpy[1]; shm_data->ee_rpy[2] = rpy[2];
      for(size_t i=0; i<6; ++i) shm_data->joint_pos[i] = q_current_(i);
  }

  q_dot_cmd_.setZero();
  v_target_.setZero();

  if (shm_data->cmd_active) {
      shm_active = true;
      int current_mode = shm_data->control_mode;

      if (current_mode == shm::MODE_CARTESIAN_POSE) {
          if (traj_gen_) traj_gen_->stop();
          srk::Frame target_frame = srk::Frame::Identity();
          target_frame.translation() << shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2];
          target_frame.linear() = (Eigen::AngleAxisd(shm_data->target_rpy[0], Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(shm_data->target_rpy[1], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(shm_data->target_rpy[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
          kinematics_core_->solveIK_Position(q_current_, target_frame, q_dot_cmd_);
      }
      else if (current_mode == shm::MODE_TRAJECTORY) {
          // Lấy vận tốc Jogging
          srk::Vector6d v_jog;
          v_jog << shm_data->traj_vel_linear[0], shm_data->traj_vel_linear[1], shm_data->traj_vel_linear[2],
                   shm_data->traj_vel_angular[0], shm_data->traj_vel_angular[1], shm_data->traj_vel_angular[2];

          // 1. Xử lý Trigger Path (Chạy quỹ đạo)
          static int last_traj_trigger = 0;
          int trig = shm_data->traj_start_trigger; 
          if (trig != last_traj_trigger) {
              last_traj_trigger = trig;
              srk::Frame end_pose = srk::Frame::Identity();
              end_pose.translation() << shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2];
              end_pose.linear() = (Eigen::AngleAxisd(shm_data->target_rpy[0], Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(shm_data->target_rpy[1], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(shm_data->target_rpy[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
              traj_gen_->setPath(current_pose, end_pose, std::max(0.1, shm_data->traj_duration));
          }

          // 2. LOGIC ƯU TIÊN ĐIỀU KHIỂN
          if (traj_gen_->isRunning()) {
              // ĐANG CHẠY PATH: Tính toán bước tiếp theo
              srk::Frame target_step; srk::Vector6d v_ff;
              if (traj_gen_->computeStep(dt, target_step, v_ff)) {
                  double Kp = 10.0;
                  Eigen::Vector3d p_err = target_step.translation() - current_pose.translation();
                  Eigen::Quaterniond q_diff = Eigen::Quaterniond(target_step.linear()) * Eigen::Quaterniond(current_pose.linear()).inverse();
                  Eigen::AngleAxisd aa_err(q_diff);
                  v_target_.head(3) = v_ff.head(3) + (p_err * Kp);
                  v_target_.tail(3) = v_ff.tail(3) + (aa_err.axis() * aa_err.angle() * Kp);
                  kinematics_core_->solveIK_Velocity(q_current_, v_target_, q_dot_cmd_);
                  
                  // CẬP NHẬT target_pos liên tục để khi dừng lại robot đứng yên tại chỗ đó
                  shm_data->target_pos[0] = target_step.translation().x();
                  shm_data->target_pos[1] = target_step.translation().y();
                  shm_data->target_pos[2] = target_step.translation().z();
              }
          } 
          else if (v_jog.norm() > 1e-6) {
              // ĐANG JOGGING: Di chuyển bằng vận tốc
              kinematics_core_->solveIK_Velocity(q_current_, v_jog, q_dot_cmd_);
              
              // QUAN TRỌNG: Cập nhật target_pos trong SHM bằng vị trí HIỆN TẠI
              // Để khi nhả nút Jog, lệnh Hold Position sẽ giữ robot ở vị trí mới này
              shm_data->target_pos[0] = current_pose.translation().x();
              shm_data->target_pos[1] = current_pose.translation().y();
              shm_data->target_pos[2] = current_pose.translation().z();
              // Có thể cập nhật cả target_rpy nếu cần giữ hướng mới
          }
          else {
              // GIỮ VỊ TRÍ (HOLD): Chỉ thực hiện khi không có lệnh động nào khác
              srk::Frame target_hold = srk::Frame::Identity();
              target_hold.translation() << shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2];
              target_hold.linear() = (Eigen::AngleAxisd(shm_data->target_rpy[0], Eigen::Vector3d::UnitX()) *
                                     Eigen::AngleAxisd(shm_data->target_rpy[1], Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(shm_data->target_rpy[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
              kinematics_core_->solveIK_Position(q_current_, target_hold, q_dot_cmd_);
          }
      }
      else if (current_mode == shm::MODE_JOINT_MANUAL) {
          if (traj_gen_) traj_gen_->stop();
          for(int i=0; i<6; ++i) q_dot_cmd_(i) = shm_data->manual_joint_vel[i];
          
          // Cập nhật target Cartesian để khi thoát mode Joint Manual robot không giật về vị trí cũ
          shm_data->target_pos[0] = current_pose.translation().x();
          shm_data->target_pos[1] = current_pose.translation().y();
          shm_data->target_pos[2] = current_pose.translation().z();
      }
  }

  // 4. XUẤT LỆNH
  if (!shm_active) {
      for (size_t i = 0; i < joint_names_.size(); ++i) command_interfaces_[i].set_value(0.0);
  }
  else {
      for (size_t i = 0; i < joint_names_.size(); ++i) {
          command_interfaces_[i].set_value(q_dot_cmd_(i));
      }
      if (command_interfaces_.size() > joint_names_.size()) {
          command_interfaces_[joint_names_.size()].set_value(shm_data->cmd_gripper);
      }
  }

  return controller_interface::return_type::OK;
}

void CartesianVelocityController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr) {}

} 

PLUGINLIB_EXPORT_CLASS(
  my_robot_controllers::CartesianVelocityController, controller_interface::ControllerInterface)