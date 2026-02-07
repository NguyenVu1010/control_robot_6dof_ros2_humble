#pragma once
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <queue>
#include <limits> // Cần thiết cho Infinity

namespace stl {

// Định nghĩa Node3D đúng chuẩn
struct Node3D {
    Eigen::Vector3i pos; // Tọa độ Grid (x, y, z)
    Eigen::Vector3i parent;
    double g;
    double f;

    // Constructor mặc định phải set g và f là Vô cực
    Node3D(Eigen::Vector3i p = Eigen::Vector3i(0,0,0)) 
        : pos(p), parent(p), 
          g(std::numeric_limits<double>::infinity()), 
          f(std::numeric_limits<double>::infinity()) {}

    // Toán tử so sánh cho Priority Queue (Min-Heap)
    // std::greater sẽ trả về true nếu a > b. 
    // Khi dùng trong priority_queue, phần tử "nhỏ nhất" sẽ lên đầu.
    bool operator>(const Node3D& other) const {
        return f > other.f;
    }
};

class Grid3D {
public:
    int nx, ny, nz;
    double res;
    Eigen::Vector3d origin;
    std::vector<bool> data;

    Grid3D(int x, int y, int z, double r, Eigen::Vector3d o);
    int index(const Eigen::Vector3i& p) const;
    bool isOccupied(const Eigen::Vector3i& p) const;
    void addSphereObstacle(Eigen::Vector3d center, double radius);
};

class ThetaStar3D {
public:
    ThetaStar3D(const Grid3D& grid);
    std::vector<Eigen::Vector3d> findPath(Eigen::Vector3d start_w, Eigen::Vector3d goal_w);

private:
    Grid3D grid_;
    bool lineOfSight(Eigen::Vector3i s1, Eigen::Vector3i s2);
};

} // namespace stl