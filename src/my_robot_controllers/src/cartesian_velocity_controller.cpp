#include "my_robot_controllers/cartesian_velocity_controller.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <kdl/chain.hpp> 

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

  // Check Joints count
  if (kdl_chain.getNrOfJoints() != joint_names_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Mismatch joints count!");
      return controller_interface::CallbackReturn::ERROR;
  }

  // 3. Init Algo Modules
  // Inverse Kinematics
  kinematics_solver_ = std::make_shared<algo::RobotKinematics>(
      kdl_tree, base_link_, end_effector_link_);
      
  // Forward Kinematics (Để tính tọa độ gửi ra SHM)
  fk_solver_ = std::make_shared<algo::FkSolver>(
      kdl_tree, base_link_, end_effector_link_);

  // 4. Init Shared Memory (Server Mode = true)
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

  // 6. Subscriber
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
  // 1. Đọc Feedback từ Hardware
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q_current_(i) = state_interfaces_[i].get_value();
  }

  // 2. GIAO TIẾP SHARED MEMORY (Real-time safe)
  bool shm_cmd_active = false;
  auto* shm_data = shm_manager_->get();

  if (shm_data) {
    // A. Ghi trạng thái Joint
    for(size_t i=0; i<joint_names_.size() && i<6; ++i) {
        shm_data->joint_pos[i] = q_current_(i);
    }

    // B. Tính và Ghi FK (Thêm Log Debug)
    KDL::Frame pose;
    if (fk_solver_->calculate(q_current_, pose)) {
        // Ghi vào SHM
        shm_data->ee_pos[0] = pose.p.x();
        shm_data->ee_pos[1] = pose.p.y();
        shm_data->ee_pos[2] = pose.p.z();
        
        double r, p, y;
        pose.M.GetRPY(r, p, y);
        shm_data->ee_rpy[0] = r;
        shm_data->ee_rpy[1] = p;
        shm_data->ee_rpy[2] = y;

        // DEBUG: In ra terminal 1 lần mỗi giây để kiểm tra
        static int print_count = 0;
        if (print_count++ % 100 == 0) { // 100Hz -> 1s in 1 lần
            //  printf("[Controller] FK OK: X=%.3f Y=%.3f Z=%.3f\n", 
            //         pose.p.x(), pose.p.y(), pose.p.z());
        }
    } else {
        printf("[Controller] FK Calculation FAILED!\n");
    }

    // B. Đọc lệnh từ SHM (Nếu GUI đang active)
    if (shm_data->cmd_active) {
        v_target_ << shm_data->cmd_linear[0], shm_data->cmd_linear[1], shm_data->cmd_linear[2],
                      shm_data->cmd_angular[0], shm_data->cmd_angular[1], shm_data->cmd_angular[2];
        shm_cmd_active = true;
    }
  }

  // 3. Nếu SHM không gửi lệnh, lấy từ Topic (Backup)
  if (!shm_cmd_active) {
      auto msg = *cmd_vel_buffer_.readFromRT();
      if (msg) {
          v_target_ << msg->linear.x, msg->linear.y, msg->linear.z,
                       msg->angular.x, msg->angular.y, msg->angular.z;
      }
  }

  // 4. Giải bài toán động học nghịch đảo
  if (kinematics_solver_->convertCartToJnt(q_current_, v_target_, q_dot_cmd_)) {
    // Gửi lệnh xuống khớp
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      command_interfaces_[i].set_value(q_dot_cmd_(i));
    }
  } else {
    // Dừng robot nếu lỗi
    for (auto & interface : command_interfaces_) interface.set_value(0.0);
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