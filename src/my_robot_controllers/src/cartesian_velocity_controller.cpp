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
          target_gripper_pos = -shm_data->cmd_gripper;
          // =========================================================
          // MODE 1: POSE CONTROL (Sử dụng hàm solveIK_Position của Lib)
          // =========================================================
          if (current_mode == shm::MODE_CARTESIAN_POSE) {
            if (traj_gen_) traj_gen_->stop();

            // 1. Tính sai số (Error)
            srk::Frame target_frame = srk::Frame::Identity();
            target_frame.translation() << shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2];
            target_frame.linear() = (Eigen::AngleAxisd(shm_data->target_rpy[0], Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(shm_data->target_rpy[1], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(shm_data->target_rpy[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();

            Eigen::Vector3d p_err = target_frame.translation() - current_pose.translation();
            
            Eigen::Quaterniond q_cur(current_pose.linear());
            Eigen::Quaterniond q_tar(target_frame.linear());
            Eigen::Quaterniond q_diff = q_tar * q_cur.inverse();
            Eigen::AngleAxisd aa(q_diff);
            Eigen::Vector3d w_err = aa.axis() * aa.angle();

            // 2. Tính toán PI (Proportional + Integral)
            // dt = period.seconds()
            double dt = period.seconds();
            
            // Cấu hình Gain (Bạn có thể đưa ra file yaml)
            double Kp_lin = 5.0;
            double Ki_lin = 0.5; // Tích phân nhẹ
            double Kp_rot = 3.0;
            double Ki_rot = 0.1;

            // Giới hạn tích phân (Anti-Windup) - CỰC KỲ QUAN TRỌNG
            // Ngăn không cho I cộng dồn quá lớn gây nguy hiểm
            double max_i_lin = 0.2; // Tối đa đóng góp 0.2 m/s
            double max_i_rot = 0.5; // Tối đa đóng góp 0.5 rad/s

            // Tính toán Linear (XYZ)
            for(int i=0; i<3; ++i) {
                // Tích phân: Sum = Sum + Error * dt
                error_sum_(i) += p_err(i) * dt;
                
                // Kẹp giá trị tích phân (Anti-windup)
                error_sum_(i) = std::clamp(error_sum_(i), -max_i_lin/Ki_lin, max_i_lin/Ki_lin);
                
                // Công thức PI: V = Kp*E + Ki*Sum
                v_target_(i) = Kp_lin * p_err(i) + Ki_lin * error_sum_(i);
            }

            // Tính toán Angular (RPY)
            for(int i=0; i<3; ++i) {
                error_sum_(i+3) += w_err(i) * dt;
                error_sum_(i+3) = std::clamp(error_sum_(i+3), -max_i_rot/Ki_rot, max_i_rot/Ki_rot);
                
                v_target_(i+3) = Kp_rot * w_err(i) + Ki_rot * error_sum_(i+3);
            }

            // 3. Gọi IK Solver (Velocity Mode)
            // Lưu ý: Ta dùng solveIK_Velocity chứ không dùng solveIK_Position nữa
            // Vì ta đã tự tính vận tốc (v_target_) ở trên rồi.
            kinematics_core_->solveIK_Velocity(q_current_, v_target_, q_dot_cmd_);
          }
          
          // =========================================================
          // MODE 2: TRAJECTORY (Sử dụng thư viện Simple Trajectory Lib)
          // =========================================================
          else if (current_mode == shm::MODE_TRAJECTORY) {
            double dt = period.seconds();
            int trig = shm_data->traj_start_trigger;
            
            if (trig != last_traj_trigger) {
                last_traj_trigger = trig;
                
                // 1. Khi bắt đầu, đặt Pose lý tưởng = Pose thực tế hiện tại
                desired_pose_ = current_pose;
                error_sum_.setZero();
                
                // Khởi tạo đích đến cho thư viện
                srk::Frame end_pose = srk::Frame::Identity();
                end_pose.translation() << shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2];
                end_pose.linear() = (Eigen::AngleAxisd(shm_data->target_rpy[0], Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(shm_data->target_rpy[1], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(shm_data->target_rpy[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
                
                double T = shm_data->traj_duration;
                if (T < 0.1) T = 0.5;
                traj_gen_->setPath(current_pose, end_pose, T);
            }

            // 2. Lấy vận tốc lý tưởng (Feedforward) từ thư viện (Dùng hàm 2 tham số hợp lệ)
            srk::Vector6d v_feedforward;
            if (traj_gen_->computeStep(dt, v_feedforward)) {
                
                // --- TỰ CẬP NHẬT POSE LÝ TƯỞNG (INTEGRATION) ---
                // Tịnh tiến: P_new = P_old + V * dt
                desired_pose_.translation() += v_feedforward.head<3>() * dt;
                
                // Quay: Sử dụng Quaternion tích phân để tránh lỗi Euler
                Eigen::Vector3d w = v_feedforward.tail<3>();
                if (w.norm() > 1e-6) {
                    Eigen::Quaterniond delta_q(Eigen::AngleAxisd(w.norm() * dt, w.normalized()));
                    desired_pose_.linear() = (delta_q * Eigen::Quaterniond(desired_pose_.linear())).toRotationMatrix();
                }

                // --- TÍNH SAI SỐ GIỮA POSE LÝ TƯỞNG VÀ THỰC TẾ ---
                Eigen::Vector3d p_err = desired_pose_.translation() - current_pose.translation();
                
                Eigen::Quaterniond q_actual(current_pose.linear());
                Eigen::Quaterniond q_desired(desired_pose_.linear());
                Eigen::Quaterniond q_diff = q_desired * q_actual.inverse();
                Eigen::AngleAxisd aa_err(q_diff);
                Eigen::Vector3d w_err = aa_err.axis() * aa_err.angle();

                // --- BỘ ĐIỀU KHIỂN BÙ SAI SỐ (FEEDBACK) ---
                double Kp_track_lin = 20.0; // Tăng Gain để bám cực sát
                double Kp_track_rot = 15.0;
                double Ki_track = 0.1;

                srk::Vector6d v_feedback;
                for(int i=0; i<3; ++i) {
                    error_sum_(i) += p_err(i) * dt;
                    v_feedback(i) = Kp_track_lin * p_err(i) + Ki_track * error_sum_(i);
                    
                    error_sum_(i+3) += w_err(i) * dt;
                    v_feedback(i+3) = Kp_track_rot * w_err(i) + Ki_track * error_sum_(i+3);
                }

                // TỔNG VẬN TỐC = LÝ THUYẾT + BÙ SAI SỐ
                v_target_ = v_feedforward + v_feedback;

                // Giải IK
                kinematics_core_->solveIK_Velocity(q_current_, v_target_, q_dot_cmd_);
            } else {
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