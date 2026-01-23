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
  // 1. Load Parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  base_link_ = get_node()->get_parameter("base_link").as_string();
  end_effector_link_ = get_node()->get_parameter("end_effector_link").as_string();
  std::string robot_desc = get_node()->get_parameter("robot_description").as_string();

  if (joint_names_.empty() || robot_desc.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameters empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // 2. Parse KDL Tree from URDF
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromString(robot_desc, kdl_tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // 3. Validate Chain (Local check)
  KDL::Chain kdl_chain;
  if (!kdl_tree.getChain(base_link_, end_effector_link_, kdl_chain)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get chain from %s to %s", 
                 base_link_.c_str(), end_effector_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Check Joints count (Arm only)
  if (kdl_chain.getNrOfJoints() != joint_names_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Mismatch joints count! URDF: %d, YAML: %ld", 
                   kdl_chain.getNrOfJoints(), joint_names_.size());
      return controller_interface::CallbackReturn::ERROR;
  }

  // 4. Init Algorithm Modules
  // Module 1: IK Solver
  kinematics_solver_ = std::make_shared<algo::RobotKinematics>(
      kdl_tree, base_link_, end_effector_link_);
      
  // Module 2: FK Solver (Position Feedback)
  fk_solver_ = std::make_shared<algo::FkSolver>(
      kdl_tree, base_link_, end_effector_link_);

  // Module 3: Trajectory Solver (Path Planning)
  // Lưu ý: Bạn cần thêm std::shared_ptr<algo::TrajectorySolver> traj_solver_; vào file .hpp
  // Nếu chưa có, hãy thêm vào header. Hoặc khởi tạo local nếu chỉ dùng trong update (nhưng cần persistence).
  // Giả sử đã có trong header:
  traj_solver_ = std::make_shared<algo::TrajectorySolver>();

  // 5. Init Shared Memory
  shm_manager_ = std::make_shared<shm::ShmManager>(shm::ROBOT_SHM_NAME, true);
  if (!shm_manager_->init()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to init Shared Memory!");
      return controller_interface::CallbackReturn::ERROR;
  }

  // 6. Resize Variables
  auto n_joints = kinematics_solver_->getNrOfJoints();
  q_current_.resize(n_joints);
  q_dot_cmd_.resize(n_joints);
  q_dot_cmd_.setZero();
  v_target_.setZero();

  RCLCPP_INFO(get_node()->get_logger(), "Controller Configured Successfully. Ready for SHM.");
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
  
  // 2. Gripper Joint (Position) - Index 6
  config.names.push_back("finger_right_joint/position");

  return config;
}

controller_interface::InterfaceConfiguration CartesianVelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Read Position of Arm Joints
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
  // Reset Trajectory
  if(traj_solver_) traj_solver_->stop();
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

  // --- VARIABLES ---
  auto* shm_data = shm_manager_->get();
  bool shm_active = false;
  int current_mode = shm::MODE_IDLE;
  double target_gripper_pos = 0.0;
  static int last_traj_trigger = 0; // Biến tĩnh để theo dõi trigger thay đổi

  // Default stop
  v_target_.setZero();
  q_dot_cmd_.setZero();

  // 2. Xử lý Logic (FK + Control Modes)
  KDL::Frame current_pose;
  
  // Tính FK
  if (shm_data && fk_solver_->calculate(q_current_, current_pose)) {
      
      // A. Ghi Feedback ra SHM
      shm_data->ee_pos[0] = current_pose.p.x();
      shm_data->ee_pos[1] = current_pose.p.y();
      shm_data->ee_pos[2] = current_pose.p.z();
      
      double r, p, y; current_pose.M.GetRPY(r, p, y);
      shm_data->ee_rpy[0] = r; shm_data->ee_rpy[1] = p; shm_data->ee_rpy[2] = y;
      
      for(size_t i=0; i<6; ++i) shm_data->joint_pos[i] = q_current_(i);

      // B. Đọc Lệnh Điều Khiển
      if (shm_data->cmd_active) {
          shm_active = true;
          current_mode = shm_data->control_mode;
          target_gripper_pos = shm_data->cmd_gripper;

          // =========================================================
          // MODE 1: POSE CONTROL (P-Controller: Position -> Velocity)
          // =========================================================
          if (current_mode == shm::MODE_CARTESIAN_POSE) {
              traj_solver_->stop(); // Dừng bộ sinh quỹ đạo nếu có

              KDL::Vector target_p(shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2]);
              KDL::Rotation target_M = KDL::Rotation::RPY(shm_data->target_rpy[0], shm_data->target_rpy[1], shm_data->target_rpy[2]);
              
              // Tính sai số
              KDL::Twist error = KDL::diff(current_pose, KDL::Frame(target_M, target_p));

              // Hệ số P
              double Kp_lin = 4.0;
              double Kp_ang = 2.0;
              double max_lin = 0.5; 
              double max_ang = 1.0;

              for(int i=0; i<3; ++i) {
                  v_target_(i)   = std::clamp(error.vel(i) * Kp_lin, -max_lin, max_lin);
                  v_target_(i+3) = std::clamp(error.rot(i) * Kp_ang, -max_ang, max_ang);
              }
              // Deadzone
              if (error.vel.Norm() < 0.002 && error.rot.Norm() < 0.02) v_target_.setZero();
          }
          
          // =========================================================
          // MODE 2: TRAJECTORY (Point-to-Point Interpolation)
          // =========================================================
          else if (current_mode == shm::MODE_TRAJECTORY) {
              int trigger = shm_data->traj_start_trigger; // Offset 208 in SHM struct
              
              // Nếu GUI bấm nút Trigger (giá trị thay đổi) -> Setup hành trình mới
              if (trigger != last_traj_trigger) {
                  last_traj_trigger = trigger;
                  
                  KDL::Vector p_end(shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2]);
                  KDL::Rotation m_end = KDL::Rotation::RPY(shm_data->target_rpy[0], shm_data->target_rpy[1], shm_data->target_rpy[2]);
                  
                  double T = shm_data->traj_duration; // Offset 200
                  if(T < 0.5) T = 0.5;

                  // Gọi module solver để set quỹ đạo từ Current -> End trong T giây
                  traj_solver_->setTrajectory(current_pose, KDL::Frame(m_end, p_end), T);
              }

              // Tính vận tốc tại bước thời gian hiện tại
              // Hàm computeVelocity sẽ trả về v_target_ mượt mà
              traj_solver_->computeVelocity(period.seconds(), v_target_);
          }

          // =========================================================
          // MODE 3: JOINT MANUAL (Direct Joint Velocity)
          // =========================================================
          else if (current_mode == shm::MODE_JOINT_MANUAL) {
              traj_solver_->stop();
              // Đọc thẳng vận tốc khớp, không qua IK
              for(int i=0; i<6; ++i) {
                  q_dot_cmd_(i) = shm_data->manual_joint_vel[i];
              }
          }
      }
  }

  // 3. GỬI LỆNH XUỐNG HARDWARE
  
  if (!shm_active) {
      // An toàn: Dừng robot
      for (size_t i = 0; i < joint_names_.size(); ++i) command_interfaces_[i].set_value(0.0);
  }
  else {
      // --- NHÁNH 1: MANUAL ---
      if (current_mode == shm::MODE_JOINT_MANUAL) {
          for (size_t i = 0; i < joint_names_.size(); ++i) {
              command_interfaces_[i].set_value(q_dot_cmd_(i));
          }
      }
      // --- NHÁNH 2: CARTESIAN (POSE & TRAJ) ---
      else {
          // Tính IK từ v_target_ (đã được tính bởi P-Controller hoặc TrajSolver)
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

  // 4. ĐIỀU KHIỂN KẸP (Luôn hoạt động nếu SHM active)
  if (shm_active && command_interfaces_.size() > 6) {
      command_interfaces_[6].set_value(target_gripper_pos);
  }

  return controller_interface::return_type::OK;
}

// Hàm này không dùng nữa
void CartesianVelocityController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr /*msg*/) {}

} 

PLUGINLIB_EXPORT_CLASS(
  my_robot_controllers::CartesianVelocityController, controller_interface::ControllerInterface)