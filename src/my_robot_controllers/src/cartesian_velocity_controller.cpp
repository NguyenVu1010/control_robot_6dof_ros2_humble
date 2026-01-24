#include "my_robot_controllers/cartesian_velocity_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm> // std::clamp
#include <cmath>     // M_PI

// --- SỬA: Dùng thư viện mới thay vì KDL trực tiếp ---
#include "simple_kinematics_lib/kinematics_core.hpp"
#include "simple_trajectory_lib/trajectory_generator.hpp"

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
    RCLCPP_ERROR(get_node()->get_logger(), "Parameters empty (joints or robot_description)");
    return controller_interface::CallbackReturn::ERROR;
  }

  // --- SỬA: KHỞI TẠO THƯ VIỆN ĐỘNG HỌC MỚI (SIMPLE KINEMATICS LIB) ---
  // Không dùng kdl_parser nữa, dùng hàm init của thư viện
  kinematics_core_ = std::make_shared<srk::KinematicsCore>();
  
  if (!kinematics_core_->init(robot_desc, base_link_, end_effector_link_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to init Kinematics Lib (Check URDF/Link Names)");
      return controller_interface::CallbackReturn::ERROR;
  }

  // Kiểm tra số lượng khớp
  if (kinematics_core_->getNrOfJoints() != joint_names_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Mismatch joints! Lib found: %d, YAML config: %ld", 
                   kinematics_core_->getNrOfJoints(), joint_names_.size());
      return controller_interface::CallbackReturn::ERROR;
  }

  // --- SỬA: KHỞI TẠO THƯ VIỆN QUỸ ĐẠO MỚI (SIMPLE TRAJECTORY LIB) ---
  traj_gen_ = std::make_shared<stl::TrajectoryGenerator>();

  // 4. Init Shared Memory (Server Mode = true)
  shm_manager_ = std::make_shared<shm::ShmManager>(shm::ROBOT_SHM_NAME, true);
  if (!shm_manager_->init()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to init Shared Memory!");
      return controller_interface::CallbackReturn::ERROR;
  }

  // 5. Resize Variables (Dùng kiểu dữ liệu Eigen của thư viện mới)
  auto n_joints = kinematics_core_->getNrOfJoints();
  q_current_.resize(n_joints);
  q_dot_cmd_.resize(n_joints);
  
  // Reset về 0
  q_dot_cmd_.setZero();
  q_current_.setZero();
  v_target_.setZero();

  RCLCPP_INFO(get_node()->get_logger(), "Controller Configured Successfully!");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // 1. Arm Joints (Velocity)
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/velocity");
  }
  
  // 2. Gripper Joint (Position)
  config.names.push_back("gripper_right_joint/position");

  return config;
}

controller_interface::InterfaceConfiguration CartesianVelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Feedback Position cho Arm
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
  if (traj_gen_) traj_gen_->stop();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & interface : command_interfaces_) interface.set_value(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianVelocityController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // 1. Đọc Feedback từ Hardware
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q_current_(i) = state_interfaces_[i].get_value();
  }

  auto* shm_data = shm_manager_->get();
  bool shm_active = false;
  int current_mode = shm::MODE_IDLE;
  double target_gripper_pos = 0.0;
  static int last_traj_trigger = 0;

  // 2. Tính FK (Sử dụng thư viện mới)
  srk::Frame current_pose; 
  // Hàm solveFK trả về true/false, kết quả lưu vào current_pose (Eigen::Isometry3d)
  bool fk_ok = kinematics_core_->solveFK(q_current_, current_pose);

  if (shm_data && fk_ok) {
      // --- GHI FEEDBACK RA SHM ---
      Eigen::Vector3d pos = current_pose.translation();
      shm_data->ee_pos[0] = pos.x();
      shm_data->ee_pos[1] = pos.y();
      shm_data->ee_pos[2] = pos.z();
      
      // Chuyển Rotation Matrix sang RPY (XYZ convention)
      Eigen::Vector3d rpy = current_pose.rotation().eulerAngles(0, 1, 2); 
      shm_data->ee_rpy[0] = rpy[0]; 
      shm_data->ee_rpy[1] = rpy[1]; 
      shm_data->ee_rpy[2] = rpy[2];
      
      for(size_t i=0; i<6; ++i) shm_data->joint_pos[i] = q_current_(i);

      // --- ĐỌC LỆNH TỪ SHM ---
      if (shm_data->cmd_active) {
          shm_active = true;
          current_mode = shm_data->control_mode;
          target_gripper_pos = shm_data->cmd_gripper;

          // =========================================================
          // MODE 1: POSE CONTROL (Sử dụng hàm solveIK_Position của Lib)
          // =========================================================
          if (current_mode == shm::MODE_CARTESIAN_POSE) {
              if (traj_gen_) traj_gen_->stop();

              // Tạo Frame đích từ dữ liệu SHM
              srk::Frame target_frame = srk::Frame::Identity();
              target_frame.translation() << shm_data->target_pos[0], 
                                            shm_data->target_pos[1], 
                                            shm_data->target_pos[2];
              
              // Tạo Rotation từ RPY
              target_frame.linear() = (Eigen::AngleAxisd(shm_data->target_rpy[0], Eigen::Vector3d::UnitX()) *
                                       Eigen::AngleAxisd(shm_data->target_rpy[1], Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(shm_data->target_rpy[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();

              // Gọi hàm solveIK_Position của thư viện mới
              // Hàm này đã tích hợp P-Controller và xử lý Singularity
              kinematics_core_->solveIK_Position(q_current_, target_frame, q_dot_cmd_);
          }
          
          // =========================================================
          // MODE 2: TRAJECTORY (Sử dụng thư viện Simple Trajectory Lib)
          // =========================================================
          else if (current_mode == shm::MODE_TRAJECTORY) {
              // Check Trigger từ GUI
              int trig = shm_data->traj_start_trigger; 
              if (trig != last_traj_trigger) {
                  last_traj_trigger = trig;
                  
                  // Setup Path (Current -> Target)
                  srk::Frame end_pose = srk::Frame::Identity();
                  end_pose.translation() << shm_data->target_pos[0], 
                                            shm_data->target_pos[1], 
                                            shm_data->target_pos[2];
                  end_pose.linear() = (Eigen::AngleAxisd(shm_data->target_rpy[0], Eigen::Vector3d::UnitX()) *
                                       Eigen::AngleAxisd(shm_data->target_rpy[1], Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(shm_data->target_rpy[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
                  
                  double T = shm_data->traj_duration; 
                  if (T < 0.5) T = 0.5;

                  traj_gen_->setPath(current_pose, end_pose, T);
              }

              // Compute Velocity Step
              srk::Vector6d v_traj;
              if (traj_gen_->computeStep(period.seconds(), v_traj)) {
                  // Tính IK Velocity từ v_traj dùng thư viện động học
                  kinematics_core_->solveIK_Velocity(q_current_, v_traj, q_dot_cmd_);
              } else {
                  // Hết hành trình -> Dừng khớp
                  q_dot_cmd_.setZero(); 
              }
          }

          // =========================================================
          // MODE 3: JOINT MANUAL (Điều khiển trực tiếp)
          // =========================================================
          else if (current_mode == shm::MODE_JOINT_MANUAL) {
              if (traj_gen_) traj_gen_->stop();
              // Gán trực tiếp, không qua IK
              for(int i=0; i<6; ++i) {
                  q_dot_cmd_(i) = shm_data->manual_joint_vel[i];
              }
          }
      }
  }

  // 3. GỬI LỆNH XUỐNG HARDWARE (ARM)
  if (!shm_active) {
      // Dừng an toàn
      for (size_t i = 0; i < joint_names_.size(); ++i) command_interfaces_[i].set_value(0.0);
  }
  else {
      // Gửi q_dot_cmd_ (đã được tính toán ở trên tùy theo Mode)
      for (size_t i = 0; i < joint_names_.size(); ++i) {
          command_interfaces_[i].set_value(q_dot_cmd_(i));
      }
  }

  // 4. GỬI LỆNH GRIPPER
  // Interface kẹp là cái cuối cùng (index 6)
  if (shm_active && command_interfaces_.size() > 6) {
      command_interfaces_[6].set_value(target_gripper_pos);
  }

  return controller_interface::return_type::OK;
}

// Deprecated callback
void CartesianVelocityController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr /*msg*/) {}

} 

PLUGINLIB_EXPORT_CLASS(
  my_robot_controllers::CartesianVelocityController, controller_interface::ControllerInterface)