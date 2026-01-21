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

  // 2. Parse KDL
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromString(robot_desc, kdl_tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // Validation Chain
  KDL::Chain kdl_chain;
  if (!kdl_tree.getChain(base_link_, end_effector_link_, kdl_chain)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get chain");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Check Joints count (Chỉ check 6 khớp cánh tay)
  if (kdl_chain.getNrOfJoints() != joint_names_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Mismatch joints count!");
      return controller_interface::CallbackReturn::ERROR;
  }

  // 3. Init Algo Modules
  kinematics_solver_ = std::make_shared<algo::RobotKinematics>(
      kdl_tree, base_link_, end_effector_link_);
      
  fk_solver_ = std::make_shared<algo::FkSolver>(
      kdl_tree, base_link_, end_effector_link_);

  // 4. Init Shared Memory
  shm_manager_ = std::make_shared<shm::ShmManager>(shm::ROBOT_SHM_NAME, true);
  if (!shm_manager_->init()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to init Shared Memory!");
      return controller_interface::CallbackReturn::ERROR;
  }

  // 5. Resize Variables
  auto n_joints = kinematics_solver_->getNrOfJoints();
  q_current_.resize(n_joints);
  q_dot_cmd_.resize(n_joints);
  q_dot_cmd_.setZero();
  v_target_.setZero();

  // 6. Subscriber (Backup)
  sub_cmd_vel_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianVelocityController::cmdVelCallback, this, std::placeholders::_1));

  cmd_vel_buffer_.writeFromNonRT(std::make_shared<geometry_msgs::msg::Twist>());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // 1. Cánh tay (Velocity)
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/velocity");
  }
  
  // 2. Kẹp (Position) - Quan trọng: Phải đúng tên khớp URDF
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
  for (auto & interface : command_interfaces_) {
    interface.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianVelocityController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1. Đọc Feedback từ Hardware (Chỉ 6 khớp cánh tay)
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q_current_(i) = state_interfaces_[i].get_value();
  }

  // 2. GIAO TIẾP SHARED MEMORY
  bool shm_cmd_active = false;
  auto* shm_data = shm_manager_->get();
  double target_gripper_pos = 0.0; // Biến lưu lệnh kẹp

  // Tính FK để lấy vị trí hiện tại
  KDL::Frame current_pose;
  bool fk_ok = fk_solver_->calculate(q_current_, current_pose);

  if (shm_data && fk_ok) {
    // A. Ghi trạng thái ra SHM
    shm_data->ee_pos[0] = current_pose.p.x();
    shm_data->ee_pos[1] = current_pose.p.y();
    shm_data->ee_pos[2] = current_pose.p.z();
    
    double r, p, y;
    current_pose.M.GetRPY(r, p, y);
    shm_data->ee_rpy[0] = r;
    shm_data->ee_rpy[1] = p;
    shm_data->ee_rpy[2] = y;

    for(size_t i=0; i<6; ++i) {
        shm_data->joint_pos[i] = q_current_(i);
    }

    // B. Đọc lệnh từ SHM
    if (shm_data->cmd_active) {
        shm_cmd_active = true;

        // --- B1: XỬ LÝ CÁNH TAY (P-CONTROLLER) ---
        // Lấy tọa độ đích từ SHM
        KDL::Vector target_p(
            shm_data->target_pos[0], 
            shm_data->target_pos[1], 
            shm_data->target_pos[2]
        );
        KDL::Rotation target_M = KDL::Rotation::RPY(
            shm_data->target_rpy[0], 
            shm_data->target_rpy[1], 
            shm_data->target_rpy[2]
        );
        KDL::Frame target_frame(target_M, target_p);

        // Tính sai số vị trí (Error Twist)
        KDL::Twist error = KDL::diff(current_pose, target_frame);

        // Hệ số P-Controller (Tùy chỉnh để mượt hơn)
        double Kp_lin = 4.0; 
        double Kp_ang = 2.0;
        double max_lin = 0.5; // Giới hạn 0.5 m/s
        double max_ang = 1.0; // Giới hạn 1.0 rad/s

        // Tính vận tốc cần thiết = Error * Kp
        for(int i=0; i<3; ++i) {
            v_target_(i)   = std::clamp(error.vel(i) * Kp_lin, -max_lin, max_lin);
            v_target_(i+3) = std::clamp(error.rot(i) * Kp_ang, -max_ang, max_ang);
        }
        
        // Deadzone: Nếu sai số quá nhỏ (<1mm) thì dừng hẳn
        if (error.vel.Norm() < 0.001 && error.rot.Norm() < 0.01) {
            v_target_.setZero();
        }

        // --- B2: XỬ LÝ KẸP ---
        target_gripper_pos = shm_data->cmd_gripper;
    }
  }

  // 3. Fallback Topic (Nếu SHM tắt, dùng lại mode vận tốc cũ)
  if (!shm_cmd_active) {
      auto msg = *cmd_vel_buffer_.readFromRT();
      if (msg) {
          v_target_(0) = msg->linear.x;
          v_target_(1) = msg->linear.y;
          v_target_(2) = msg->linear.z;
          v_target_(3) = msg->angular.x;
          v_target_(4) = msg->angular.y;
          v_target_(5) = msg->angular.z;
      } else {
          // Nếu không có lệnh mới từ topic -> Dừng
          // v_target_.setZero(); 
      }
  }

  // 4. Giải bài toán động học nghịch đảo (IK) cho Cánh tay
  // Input: v_target_ (tính từ P-Controller hoặc Topic) -> Output: q_dot_cmd_
  if (kinematics_solver_->convertCartToJnt(q_current_, v_target_, q_dot_cmd_)) {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      command_interfaces_[i].set_value(q_dot_cmd_(i));
    }
  } else {
    // Lỗi IK (Singularity) -> Dừng cánh tay
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces_[i].set_value(0.0);
    }
  }

  // 5. Gửi lệnh cho Kẹp (Gripper)
  // Interface của kẹp nằm ở vị trí cuối cùng (index = 6)
  if (shm_cmd_active && command_interfaces_.size() > 6) {
      command_interfaces_[6].set_value(target_gripper_pos);
  }

  return controller_interface::return_type::OK;
}

void CartesianVelocityController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_buffer_.writeFromNonRT(msg);
}

} 

PLUGINLIB_EXPORT_CLASS(
  my_robot_controllers::CartesianVelocityController, controller_interface::ControllerInterface)