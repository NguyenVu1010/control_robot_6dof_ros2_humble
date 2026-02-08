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
  // 1. Lấy Feedback từ Hardware
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q_current_(i) = state_interfaces_[i].get_value();
  }

  auto* shm_data = shm_manager_->get();
  if (!shm_data) return controller_interface::return_type::OK;

  double dt = period.seconds();
  
  // 2. Tính toán Pose hiện tại (Forward Kinematics)
  srk::Frame current_pose;
  if (kinematics_core_->solveFK(q_current_, current_pose)) {
      // Ghi feedback vào SHM cho GUI hiển thị
      Eigen::Vector3d pos = current_pose.translation();
      shm_data->ee_pos[0] = pos.x(); shm_data->ee_pos[1] = pos.y(); shm_data->ee_pos[2] = pos.z();
      
      // Chuyển đổi sang RPY để GUI dễ đọc
      Eigen::Vector3d rpy = current_pose.rotation().eulerAngles(0, 1, 2); 
      shm_data->ee_rpy[0] = rpy[0]; shm_data->ee_rpy[1] = rpy[1]; shm_data->ee_rpy[2] = rpy[2];
      for(size_t i=0; i<6; ++i) shm_data->joint_pos[i] = q_current_(i);
  }

  // Khởi tạo lệnh mặc định là dừng
  q_dot_cmd_.setZero();

  // 3. LOGIC CHUYỂN MODE (TRANSITION)
  // Nếu GUI vừa nhấn "Activate" hoặc vừa đổi "Mode"
  bool mode_changed = (shm_data->control_mode != last_mode_);
  bool activated = (shm_data->cmd_active && !last_cmd_active_);

  if (activated || (shm_data->cmd_active && mode_changed)) {
      // ĐỒNG BỘ HÓA: Khi mới vào mode, đặt Target = Vị trí hiện tại để tránh robot bị nhảy
      shm_data->target_pos[0] = current_pose.translation().x();
      shm_data->target_pos[1] = current_pose.translation().y();
      shm_data->target_pos[2] = current_pose.translation().z();
      
      Eigen::Vector3d rpy = current_pose.rotation().eulerAngles(0, 1, 2);
      shm_data->target_rpy[0] = rpy[0];
      shm_data->target_rpy[1] = rpy[1];
      shm_data->target_rpy[2] = rpy[2];

      // Dừng quỹ đạo cũ nếu có
      if (traj_gen_) traj_gen_->stop();
  }

  // Lưu trạng thái cho vòng lặp sau
  last_mode_ = shm_data->control_mode;
  last_cmd_active_ = shm_data->cmd_active.load();

  // 4. THỰC THI MODE HIỆN TẠI
  if (shm_data->cmd_active) {
      switch (shm_data->control_mode) {
          
          case shm::MODE_IDLE:
              q_dot_cmd_.setZero();
              break;

          case shm::MODE_CARTESIAN_POSE: {
              srk::Frame target_frame = srk::Frame::Identity();
              target_frame.translation() << shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2];
              target_frame.linear() = (Eigen::AngleAxisd(shm_data->target_rpy[0], Eigen::Vector3d::UnitX()) *
                                       Eigen::AngleAxisd(shm_data->target_rpy[1], Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(shm_data->target_rpy[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
              
              // IK Position thường dùng P-gain bên trong để tạo vận tốc hướng về Target
              kinematics_core_->solveIK_Position(q_current_, target_frame, q_dot_cmd_);
              break;
          }

          case shm::MODE_TRAJECTORY: {
              srk::Vector6d v_jog;
              v_jog << shm_data->traj_vel_linear[0], shm_data->traj_vel_linear[1], shm_data->traj_vel_linear[2],
                       shm_data->traj_vel_angular[0], shm_data->traj_vel_angular[1], shm_data->traj_vel_angular[2];

              // Kiểm tra trigger chạy Path từ GUI
              static int last_trig = 0;
              if (shm_data->traj_start_trigger != last_trig) {
                  last_trig = shm_data->traj_start_trigger;
                  srk::Frame end_pose = srk::Frame::Identity();
                  end_pose.translation() << shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2];
                  end_pose.linear() = (Eigen::AngleAxisd(shm_data->target_rpy[0], Eigen::Vector3d::UnitX()) *
                                       Eigen::AngleAxisd(shm_data->target_rpy[1], Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(shm_data->target_rpy[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
                  traj_gen_->setPath(current_pose, end_pose, std::max(0.1, shm_data->traj_duration));
              }

              if (traj_gen_->isRunning()) {
                  srk::Frame target_step; srk::Vector6d v_ff;
                  if (traj_gen_->computeStep(dt, target_step, v_ff)) {
                      double Kp = 10.0;
                      Eigen::Vector3d p_err = target_step.translation() - current_pose.translation();
                      Eigen::Quaterniond q_diff = Eigen::Quaterniond(target_step.linear()) * Eigen::Quaterniond(current_pose.linear()).inverse();
                      Eigen::AngleAxisd aa_err(q_diff);
                      
                      srk::Vector6d v_target;
                      v_target.head(3) = v_ff.head(3) + (p_err * Kp);
                      v_target.tail(3) = v_ff.tail(3) + (aa_err.axis() * aa_err.angle() * Kp);
                      kinematics_core_->solveIK_Velocity(q_current_, v_target, q_dot_cmd_);
                      
                      // Cập nhật target_pos liên tục để khi dừng lại robot đứng yên tại chỗ đó
                      shm_data->target_pos[0] = current_pose.translation().x();
                      shm_data->target_pos[1] = current_pose.translation().y();
                      shm_data->target_pos[2] = current_pose.translation().z();
                  }
              } 
              else if (v_jog.norm() > 1e-6) {
                  // Đang nhấn nút Jogging trên GUI
                  kinematics_core_->solveIK_Velocity(q_current_, v_jog, q_dot_cmd_);
                  // Sync target để khi nhả nút Jog robot sẽ "Hold" tại vị trí mới
                  shm_data->target_pos[0] = current_pose.translation().x();
                  shm_data->target_pos[1] = current_pose.translation().y();
                  shm_data->target_pos[2] = current_pose.translation().z();
              }
              else {
                  // Chế độ HOLD: Giữ vị trí hiện tại
                  srk::Frame target_hold = srk::Frame::Identity();
                  target_hold.translation() << shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2];
                  target_hold.linear() = (Eigen::AngleAxisd(shm_data->target_rpy[0], Eigen::Vector3d::UnitX()) *
                                         Eigen::AngleAxisd(shm_data->target_rpy[1], Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(shm_data->target_rpy[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
                  kinematics_core_->solveIK_Position(q_current_, target_hold, q_dot_cmd_);
              }
              break;
          }

          case shm::MODE_JOINT_MANUAL:
              for(int i=0; i<6; ++i) q_dot_cmd_(i) = shm_data->manual_joint_vel[i];
              // Sync Cartesian Target để thoát mode Joint không bị giật
              shm_data->target_pos[0] = current_pose.translation().x();
              shm_data->target_pos[1] = current_pose.translation().y();
              shm_data->target_pos[2] = current_pose.translation().z();
              break;
      }
  }

  // 5. Gửi lệnh xuống Hardware
  for (size_t i = 0; i < joint_names_.size(); ++i) {
      // Giới hạn an toàn đơn giản (nên có)
      double vel = std::clamp(q_dot_cmd_(i), -1.5, 1.5); 
      command_interfaces_[i].set_value(vel);
  }
  
  // Gửi lệnh Gripper (nếu có)
  if (command_interfaces_.size() > 6) {
      command_interfaces_[6].set_value(shm_data->cmd_gripper);
  }

  return controller_interface::return_type::OK;
}

void CartesianVelocityController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr) {}

} 

PLUGINLIB_EXPORT_CLASS(
  my_robot_controllers::CartesianVelocityController, controller_interface::ControllerInterface)