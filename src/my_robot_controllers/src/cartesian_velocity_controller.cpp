#include "my_robot_controllers/cartesian_velocity_controller.hpp"
#include "my_robot_controllers/visualizer.hpp"
#include "my_robot_controllers/path_tracker.hpp"
#include "simple_trajectory_lib/ompl_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

// Thêm thư viện để xử lý góc và đọc URDF
#include <angles/angles.h>
#include <urdf/model.h>

namespace my_robot_controllers
{

    CartesianVelocityController::CartesianVelocityController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn CartesianVelocityController::on_init()
    {
        try
        {
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
            auto_declare<std::string>("base_link", "base_link");
            auto_declare<std::string>("end_effector_link", "tool_center_point");
            auto_declare<std::string>("robot_description", "");
        }
        catch (...)
        {
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CartesianVelocityController::on_configure(const rclcpp_lifecycle::State &)
    {
        joint_names_ = get_node()->get_parameter("joints").as_string_array();
        base_link_ = get_node()->get_parameter("base_link").as_string();
        end_effector_link_ = get_node()->get_parameter("end_effector_link").as_string();
        std::string robot_desc = get_node()->get_parameter("robot_description").as_string();

        if (joint_names_.empty() || robot_desc.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Joint names or robot description empty!");
            return controller_interface::CallbackReturn::ERROR;
        }

        // --- LOGIC: ĐỌC GIỚI HẠN TỪ URDF ---
        urdf::Model model;
        if (!model.initString(robot_desc))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF for limits");
            return controller_interface::CallbackReturn::ERROR;
        }

        joint_limits_min_.clear();
        joint_limits_max_.clear();
        for (const auto &name : joint_names_)
        {
            auto joint = model.getJoint(name);
            if (joint && joint->limits)
            {
                joint_limits_min_.push_back(joint->limits->lower);
                joint_limits_max_.push_back(joint->limits->upper);
                RCLCPP_INFO(get_node()->get_logger(), "Joint %s limit: [%f, %f]", name.c_str(), joint->limits->lower, joint->limits->upper);
            }
            else
            {
                // Mặc định nếu không tìm thấy limit (tránh lỗi chia 0 hoặc crash)
                joint_limits_min_.push_back(-M_PI * 2.0);
                joint_limits_max_.push_back(M_PI * 2.0);
            }
        }

        kinematics_core_ = std::make_shared<srk::KinematicsCore>();
        if (!kinematics_core_->init(robot_desc, base_link_, end_effector_link_))
            return controller_interface::CallbackReturn::ERROR;

        planner_ = std::make_shared<JointPathPlannerOMPL>(kinematics_core_);
        obs_center_ = Eigen::Vector3d(0.0, 0.0, 0.2);
        obs_radius_ = 0.15;
        planner_->setObstacle(obs_center_, obs_radius_);

        shm_manager_ = std::make_shared<shm::ShmManager>(shm::ROBOT_SHM_NAME, true);
        shm_manager_->init();

        visualizer_ = std::make_shared<Visualizer>(get_node(), base_link_);
        path_tracker_ = std::make_shared<PathTracker>();

        q_current_.resize(kinematics_core_->getNrOfJoints());
        q_dot_cmd_.resize(kinematics_core_->getNrOfJoints());

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type CartesianVelocityController::update(const rclcpp::Time & /*now*/, const rclcpp::Duration &period)
    {
        // 1. Đọc trạng thái khớp hiện tại
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            q_current_(i) = state_interfaces_[i].get_value();
        }

        srk::Frame current_pose;
        kinematics_core_->solveFK(q_current_, current_pose);

        auto *shm_data = shm_manager_->get();
        if (!shm_data)
            return controller_interface::return_type::OK;

        // 2. Feedback lên GUI (Sử dụng normalize_angle để hiển thị góc đẹp trong khoảng -PI đến PI)
        shm_data->ee_pos[0] = current_pose.translation().x();
        shm_data->ee_pos[1] = current_pose.translation().y();
        shm_data->ee_pos[2] = current_pose.translation().z();
        Eigen::Matrix3d R = current_pose.linear();
        double roll = atan2(R(2, 1), R(2, 2));
        double pitch = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
        double yaw = atan2(R(1, 0), R(0, 0));

        shm_data->ee_rpy[0] = roll;
        shm_data->ee_rpy[2] = yaw;
        for (size_t i = 0; i < 6; ++i)
        {
            shm_data->joint_pos[i] = angles::normalize_angle(q_current_(i));
        }

        q_dot_cmd_.setZero();
        double dt = period.seconds();

        // 3. Tính toán vận tốc điều khiển
        if (shm_data->cmd_active)
        {
            if (!prev_cmd_active_)
                visualizer_->publishWorkspaceAndObstacles(obs_center_, obs_radius_);

            if (shm_data->control_mode == shm::MODE_TRAJECTORY)
            {
                static int last_traj_trigger = 0;
                if (shm_data->traj_start_trigger != last_traj_trigger)
                {
                    last_traj_trigger = shm_data->traj_start_trigger;

                    srk::Frame target_goal = srk::Frame::Identity();
                    target_goal.translation() << shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2];
                    target_goal.linear() = (Eigen::AngleAxisd(shm_data->target_rpy[2], Eigen::Vector3d::UnitZ()) *  // Yaw
                                            Eigen::AngleAxisd(shm_data->target_rpy[1], Eigen::Vector3d::UnitY()) *  // Pitch
                                            Eigen::AngleAxisd(shm_data->target_rpy[0], Eigen::Vector3d::UnitX()))  // Roll
                                            .toRotationMatrix();

                    srk::JntArray q_goal = q_current_;
                    for (int i = 0; i < 50; ++i)
                    {
                        srk::JntArray dq;
                        kinematics_core_->solveIK_Position(q_goal, target_goal, dq);
                        for (int j = 0; j < 6; ++j)
                            q_goal(j) += dq(j) * 0.1;
                    }

                    auto joint_path = planner_->plan(q_current_, q_goal);
                    if (!joint_path.empty())
                    {
                        std::vector<stl::Frame> waypoints;
                        std::vector<Eigen::Vector3d> viz_points;
                        for (const auto &q_s : joint_path)
                        {
                            srk::Frame f;
                            kinematics_core_->solveFK(q_s, f);
                            waypoints.push_back(f);
                            viz_points.push_back(f.translation());
                        }
                        visualizer_->publishPath(viz_points, target_goal.linear());
                        path_tracker_->setPath(waypoints, shm_data->traj_duration);
                    }
                }

                if (path_tracker_->isRunning())
                {
                    srk::Frame ghost;
                    Eigen::VectorXd v_cart = path_tracker_->computeControlCommand(dt, current_pose, ghost);
                    kinematics_core_->solveIK_Velocity(q_current_, v_cart, q_dot_cmd_);
                }
                else
                {
                    srk::Frame target_hold = srk::Frame::Identity();
                    target_hold.translation() << shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2];
                    target_hold.linear() = (Eigen::AngleAxisd(shm_data->target_rpy[2], Eigen::Vector3d::UnitZ()) *  // Yaw
                                            Eigen::AngleAxisd(shm_data->target_rpy[1], Eigen::Vector3d::UnitY()) *  // Pitch
                                            Eigen::AngleAxisd(shm_data->target_rpy[0], Eigen::Vector3d::UnitX()))  // Roll
                                            .toRotationMatrix();
                    kinematics_core_->solveIK_Position(q_current_, target_hold, q_dot_cmd_);
                }
            }
            else if (shm_data->control_mode == shm::MODE_POSE)
            {
                path_tracker_->stop();
                srk::Frame target_pose = srk::Frame::Identity();
                target_pose.translation() << shm_data->target_pos[0], shm_data->target_pos[1], shm_data->target_pos[2];
                target_pose.linear() = (Eigen::AngleAxisd(shm_data->target_rpy[2], Eigen::Vector3d::UnitZ()) *  // Yaw
                                        Eigen::AngleAxisd(shm_data->target_rpy[1], Eigen::Vector3d::UnitY()) *  // Pitch
                                        Eigen::AngleAxisd(shm_data->target_rpy[0], Eigen::Vector3d::UnitX()))  // Roll
                                           .toRotationMatrix();
                kinematics_core_->solveIK_Position(q_current_, target_pose, q_dot_cmd_);
            }
            else if (shm_data->control_mode == shm::MODE_JOINT)
            {
                path_tracker_->stop();
                for (int i = 0; i < 6; ++i)
                    q_dot_cmd_(i) = shm_data->manual_joint_vel[i];
            }
        }

        prev_cmd_active_ = shm_data->cmd_active;

        // 4. KIỂM TRA GIỚI HẠN VÀ GỬI LỆNH (Ép vận tốc về 0 nếu chạm biên)
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            double cmd_vel = shm_data->cmd_active ? q_dot_cmd_(i) : 0.0;
            double current_pos = q_current_(i);

            // Chặn giới hạn dưới: nếu vị trí <= min và vận tốc muốn đi âm tiếp -> dừng
            if (current_pos <= joint_limits_min_[i] && cmd_vel < 0)
            {
                cmd_vel = 0.0;
            }
            // Chặn giới hạn trên: nếu vị trí >= max và vận tốc muốn đi dương tiếp -> dừng
            else if (current_pos >= joint_limits_max_[i] && cmd_vel > 0)
            {
                cmd_vel = 0.0;
            }

            command_interfaces_[i].set_value(cmd_vel);
        }

        // Điều khiển Gripper (nếu có)
        if (command_interfaces_.size() > joint_names_.size())
        {
            command_interfaces_[joint_names_.size()].set_value(shm_data->cmd_active ? shm_data->cmd_gripper : 0.0);
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::InterfaceConfiguration CartesianVelocityController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (const auto &joint : joint_names_)
            config.names.push_back(joint + "/velocity");
        config.names.push_back("gripper_right_joint/position");
        return config;
    }

    controller_interface::InterfaceConfiguration CartesianVelocityController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (const auto &joint : joint_names_)
            config.names.push_back(joint + "/position");
        return config;
    }

    controller_interface::CallbackReturn CartesianVelocityController::on_activate(const rclcpp_lifecycle::State &)
    {
        path_tracker_->stop();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        for (auto &interface : command_interfaces_)
            interface.set_value(0.0);
        return controller_interface::CallbackReturn::SUCCESS;
    }

} // namespace my_robot_controllers

PLUGINLIB_EXPORT_CLASS(my_robot_controllers::CartesianVelocityController, controller_interface::ControllerInterface)