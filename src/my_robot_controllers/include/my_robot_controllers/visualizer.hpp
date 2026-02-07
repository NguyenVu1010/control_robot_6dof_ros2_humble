#pragma once
#include "rclcpp/rclcpp.hpp"
// 1. Thêm thư viện Lifecycle Node
#include "rclcpp_lifecycle/lifecycle_node.hpp" 
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <string>

namespace my_robot_controllers {

class Visualizer {
public:
    // 2. Sửa constructor để nhận LifecycleNode::SharedPtr
    Visualizer(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const std::string& base_link)
        : node_(node), base_link_(base_link) {
        
        auto rviz_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        
        // Lifecycle Node cũng có hàm create_publisher nhưng trả về LifecyclePublisher. 
        // Tuy nhiên ở đây ta dùng nó như publisher thường để vẽ RViz cho đơn giản.
        // Ta ép kiểu về Node thường để dùng create_publisher cơ bản hoặc dùng trực tiếp.
        workspace_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/robot_workspace", rviz_qos);
        path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("~/planned_path", 10);
    }

    void publishWorkspaceAndObstacles(const Eigen::Vector3d& obs_center, double obs_radius) {
        visualization_msgs::msg::MarkerArray ma;
        auto now = node_->now();

        // 1. Workspace Cube
        visualization_msgs::msg::Marker workspace;
        workspace.header.frame_id = base_link_;
        workspace.header.stamp = now;
        workspace.ns = "workspace";
        workspace.id = 0;
        workspace.type = visualization_msgs::msg::Marker::CUBE;
        workspace.action = visualization_msgs::msg::Marker::ADD;
        
        // Cấu hình kích thước workspace
        workspace.pose.position.x = 0.0; workspace.pose.position.y = 0.0; workspace.pose.position.z = 0.9;
        workspace.scale.x = 2.0; workspace.scale.y = 2.0; workspace.scale.z = 2.0;
        workspace.color.r = 0.0f; workspace.color.g = 1.0f; workspace.color.b = 0.0f; workspace.color.a = 0.1f;
        ma.markers.push_back(workspace);

        // 2. Obstacle Sphere
        visualization_msgs::msg::Marker obstacle;
        obstacle.header.frame_id = base_link_;
        obstacle.header.stamp = now;
        obstacle.ns = "obstacles";
        obstacle.id = 1;
        obstacle.type = visualization_msgs::msg::Marker::SPHERE;
        obstacle.pose.position.x = obs_center.x();
        obstacle.pose.position.y = obs_center.y();
        obstacle.pose.position.z = obs_center.z();
        obstacle.scale.x = obs_radius * 2.0;
        obstacle.scale.y = obs_radius * 2.0;
        obstacle.scale.z = obs_radius * 2.0;
        obstacle.color.r = 1.0f; obstacle.color.g = 0.0f; obstacle.color.b = 0.0f; obstacle.color.a = 0.5f;
        ma.markers.push_back(obstacle);

        workspace_pub_->publish(ma);
    }

    void publishPath(const std::vector<Eigen::Vector3d>& path_3d, const Eigen::Matrix3d& orientation) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = node_->now();
        path_msg.header.frame_id = base_link_;

        Eigen::Quaterniond q(orientation);

        for (const auto& p : path_3d) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = p.x();
            pose.pose.position.y = p.y();
            pose.pose.position.z = p.z();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            path_msg.poses.push_back(pose);
        }
        path_pub_->publish(path_msg);
    }

private:
    // 3. Cập nhật kiểu dữ liệu của biến node_
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::string base_link_;
    
    // Lưu ý: LifecycleNode create_publisher mặc định cũng tương thích với SharedPtr của Publisher thường 
    // nếu không dùng tính năng activate/deactivate topic.
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr workspace_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

} // namespace my_robot_controllers