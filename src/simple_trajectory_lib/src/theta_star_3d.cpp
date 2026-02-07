#include "simple_trajectory_lib/theta_star_3d.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace stl {

// -----------------------------------------------------------------------------
// GRID 3D IMPLEMENTATION
// -----------------------------------------------------------------------------

Grid3D::Grid3D(int x, int y, int z, double r, Eigen::Vector3d o) 
    : nx(x), ny(y), nz(z), res(r), origin(o) {
    data.resize(nx * ny * nz, false);
}

int Grid3D::index(const Eigen::Vector3i& p) const {
    return p.x() + p.y() * nx + p.z() * nx * ny;
}

bool Grid3D::isOccupied(const Eigen::Vector3i& p) const {
    // 1. Kiểm tra biên (Bounds Check)
    if (p.x() < 0 || p.x() >= nx || 
        p.y() < 0 || p.y() >= ny || 
        p.z() < 0 || p.z() >= nz) {
        return true; // Coi như vật cản nếu nằm ngoài vùng Grid
    }
    // 2. Kiểm tra dữ liệu vật cản
    return data[index(p)];
}

void Grid3D::addSphereObstacle(Eigen::Vector3d center, double radius) {
    // Chuyển tọa độ tâm và bán kính sang đơn vị ô lưới để giới hạn vùng duyệt
    // Sử dụng floor để xử lý chính xác cả tọa độ âm
    Eigen::Vector3i min_idx(
        std::floor((center.x() - radius - origin.x()) / res),
        std::floor((center.y() - radius - origin.y()) / res),
        std::floor((center.z() - radius - origin.z()) / res)
    );
    Eigen::Vector3i max_idx(
        std::floor((center.x() + radius - origin.x()) / res),
        std::floor((center.y() + radius - origin.y()) / res),
        std::floor((center.z() + radius - origin.z()) / res)
    );

    // Kẹp trong biên Grid an toàn
    min_idx = min_idx.cwiseMax(Eigen::Vector3i(0, 0, 0));
    max_idx = max_idx.cwiseMin(Eigen::Vector3i(nx - 1, ny - 1, nz - 1));

    for (int x = min_idx.x(); x <= max_idx.x(); ++x) {
        for (int y = min_idx.y(); y <= max_idx.y(); ++y) {
            for (int z = min_idx.z(); z <= max_idx.z(); ++z) {
                // Tính tọa độ World của tâm ô lưới hiện tại
                Eigen::Vector3d world_p = origin + Eigen::Vector3d(x, y, z).cast<double>() * res;
                // Nếu khoảng cách đến tâm vật cản nhỏ hơn bán kính -> Đánh dấu là vật cản
                if ((world_p - center).norm() <= radius) {
                    data[index({x, y, z})] = true;
                }
            }
        }
    }
}

// -----------------------------------------------------------------------------
// THETA* 3D IMPLEMENTATION
// -----------------------------------------------------------------------------

ThetaStar3D::ThetaStar3D(const Grid3D& grid) : grid_(grid) {}

bool ThetaStar3D::lineOfSight(Eigen::Vector3i s1, Eigen::Vector3i s2) {
    Eigen::Vector3d p1 = s1.cast<double>();
    Eigen::Vector3d p2 = s2.cast<double>();
    double dist = (p2 - p1).norm();
    
    if (dist < 0.1) return true;

    Eigen::Vector3d dir = (p2 - p1) / dist;
    // Bước nhảy kiểm tra va chạm (nên nhỏ hơn resolution một chút, ví dụ res=0.05 thì step=0.03)
    double step = grid_.res * 0.8; 

    for (double d = 0; d <= dist; d += step) {
        // Sử dụng round để lấy ô lưới gần nhất với điểm đang xét trên đường thẳng
        Eigen::Vector3i check = (p1 + dir * (d / grid_.res)).array().round().cast<int>();
        if (grid_.isOccupied(check)) return false;
    }
    // Kiểm tra điểm cuối cùng
    return !grid_.isOccupied(s2);
}

std::vector<Eigen::Vector3d> ThetaStar3D::findPath(Eigen::Vector3d start_w, Eigen::Vector3d goal_w) {
    // CHUYỂN ĐỔI WORLD -> GRID (Sử dụng floor thay vì cast trực tiếp để xử lý số âm)
    Eigen::Vector3i start(
        std::floor((start_w.x() - grid_.origin.x()) / grid_.res),
        std::floor((start_w.y() - grid_.origin.y()) / grid_.res),
        std::floor((start_w.z() - grid_.origin.z()) / grid_.res)
    );
    Eigen::Vector3i goal(
        std::floor((goal_w.x() - grid_.origin.x()) / grid_.res),
        std::floor((goal_w.y() - grid_.origin.y()) / grid_.res),
        std::floor((goal_w.z() - grid_.origin.z()) / grid_.res)
    );

    // Kiểm tra tính hợp lệ của Start/Goal
    if (grid_.isOccupied(start)) {
        std::cout << "[Planner] FAIL: Start " << start_w.transpose() << " is Out of Bounds or Obstacle!" << std::endl;
        return {};
    }
    if (grid_.isOccupied(goal)) {
        std::cout << "[Planner] FAIL: Goal " << goal_w.transpose() << " is Out of Bounds or Obstacle!" << std::endl;
        return {};
    }

    if (start == goal) return {start_w, goal_w};

    // Priority Queue (Min-Heap) dựa trên giá trị f
    std::priority_queue<Node3D, std::vector<Node3D>, std::greater<Node3D>> open;
    std::unordered_map<int, Node3D> all;

    // Khởi tạo Node bắt đầu
    Node3D s_node(start);
    s_node.g = 0;
    s_node.f = (goal - start).cast<double>().norm();
    s_node.parent = start;
    
    open.push(s_node);
    all[grid_.index(start)] = s_node;

    while (!open.empty()) {
        Node3D curr = open.top();
        open.pop();

        // Đã đến đích
        if (curr.pos == goal) {
            std::vector<Eigen::Vector3d> path;
            Eigen::Vector3i backtrack = goal;
            
            while (backtrack != start) {
                // GRID -> WORLD: Chuyển ngược lại tọa độ mét
                path.push_back(grid_.origin + backtrack.cast<double>() * grid_.res);
                backtrack = all[grid_.index(backtrack)].parent;
            }
            path.push_back(grid_.origin + start.cast<double>() * grid_.res);
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Duyệt 26 lân cận (3x3x3 khối)
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;

                    Eigen::Vector3i nb_pos = curr.pos + Eigen::Vector3i(dx, dy, dz);
                    if (grid_.isOccupied(nb_pos)) continue;

                    int idx = grid_.index(nb_pos);
                    // Nếu chưa khám phá, Node3D sẽ được khởi tạo với g = Infinity
                    if (all.find(idx) == all.end()) {
                        all[idx] = Node3D(nb_pos);
                    }
                    
                    Node3D& nb = all[idx];
                    Eigen::Vector3i p_curr = curr.parent;
                    bool updated = false;

                    // --- LOGIC THETA*: Line of Sight Check ---
                    if (lineOfSight(p_curr, nb.pos)) {
                        double dist = (nb.pos - p_curr).cast<double>().norm();
                        double g_parent = all[grid_.index(p_curr)].g;
                        
                        if (g_parent + dist < nb.g) {
                            nb.g = g_parent + dist;
                            nb.parent = p_curr;
                            updated = true;
                        }
                    } 
                    // --- LOGIC A*: Nối từ node hiện tại ---
                    else {
                        double dist = (nb.pos - curr.pos).cast<double>().norm();
                        if (curr.g + dist < nb.g) {
                            nb.g = curr.g + dist;
                            nb.parent = curr.pos;
                            updated = true;
                        }
                    }

                    if (updated) {
                        nb.f = nb.g + (nb.pos - goal).cast<double>().norm(); // f = g + h
                        open.push(nb);
                    }
                }
            }
        }
    }
    
    return {}; // Không tìm thấy đường
}

} // namespace stl