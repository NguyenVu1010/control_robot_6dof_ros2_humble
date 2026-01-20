#include "my_robot_controllers/cartesian_velocity_controller.hpp"

#include "kdl_parser/kdl_parser.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <kdl/chain.hpp> // Cần include để dùng KDL::Chain cục bộ

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

  // 2. Parse URDF to KDL Tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromString(robot_desc, kdl_tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // 3. Validate Chain (Local check)
  // Khai báo biến cục bộ để kiểm tra tính hợp lệ của chuỗi động học
  KDL::Chain kdl_chain; 
  if (!kdl_tree.getChain(base_link_, end_effector_link_, kdl_chain)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get KDL chain from %s to %s", 
                 base_link_.c_str(), end_effector_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  auto chain_joints = kdl_chain.getNrOfJoints();
  auto yaml_joints = joint_names_.size();

  RCLCPP_INFO(get_node()->get_logger(), "--- KDL CHAIN CHECK ---");
  RCLCPP_INFO(get_node()->get_logger(), "Base Link: %s", base_link_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "End Link : %s", end_effector_link_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Joints in URDF Chain: %d", chain_joints);
  RCLCPP_INFO(get_node()->get_logger(), "Joints in YAML Config: %ld", yaml_joints);

  if (chain_joints != yaml_joints) {
      RCLCPP_ERROR(get_node()->get_logger(), 
          "FATAL ERROR: Mismatch! URDF has %d joints but YAML has %ld joints.", 
          chain_joints, yaml_joints);
      RCLCPP_ERROR(get_node()->get_logger(), "Please check 'base_link' and 'end_effector_link' names.");
      return controller_interface::CallbackReturn::ERROR;
  }

  // 4. Init Algo Module
  // Truyền Tree vào để Module tự tạo Chain nội bộ (Tránh lỗi bộ nhớ/ABI conflict)
  kinematics_solver_ = std::make_shared<algo::RobotKinematics>(
      kdl_tree, base_link_, end_effector_link_);
  
  // 5. Resize Variables
  // Lấy số khớp chính xác từ module
  auto n_joints = kinematics_solver_->getNrOfJoints();
  
  if (n_joints != joint_names_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Mismatch joints after init! KDL: %d, YAML: %ld", n_joints, joint_names_.size());
      return controller_interface::CallbackReturn::ERROR;
  }

  q_current_.resize(n_joints);
  q_dot_cmd_.resize(n_joints);
  q_dot_cmd_.setZero();
  v_target_.setZero();

  // 6. Setup Subscriber & Buffer
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
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/velocity");
  }
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
  // Reset command buffers
  q_dot_cmd_.setZero();
  v_target_.setZero();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop Robot
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianVelocityController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1. Get Command from Realtime Buffer
  auto msg = *cmd_vel_buffer_.readFromRT();
  if (!msg) return controller_interface::return_type::OK;

  v_target_ << msg->linear.x, msg->linear.y, msg->linear.z,
               msg->angular.x, msg->angular.y, msg->angular.z;

  // 2. Read Feedback
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q_current_(i) = state_interfaces_[i].get_value();
  }

  // 3. Solve Kinematics (Pure Math Calculation)
  if (kinematics_solver_->convertCartToJnt(q_current_, v_target_, q_dot_cmd_)) {
    // 4. Send Command
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      command_interfaces_[i].set_value(q_dot_cmd_(i));
    }
  } else {
    // Hạn chế in log quá nhiều trong vòng lặp realtime
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000, "IK Solver Failed or Limit Reached!");
    
    // Stop Robot on Error
    for (auto & interface : command_interfaces_) interface.set_value(0.0);
  }

  return controller_interface::return_type::OK;
}

void CartesianVelocityController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_buffer_.writeFromNonRT(msg);
}

}  // namespace my_robot_controllers

PLUGINLIB_EXPORT_CLASS(
  my_robot_controllers::CartesianVelocityController, controller_interface::ControllerInterface)