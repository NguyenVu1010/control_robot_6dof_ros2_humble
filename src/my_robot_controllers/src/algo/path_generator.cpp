#include "my_robot_controllers/algo/path_generator.hpp"

namespace my_robot_controllers {
namespace algo {

PathGenerator::PathGenerator() : current_time_(0.0) {}

void PathGenerator::reset() {
    current_time_ = 0.0;
}

void PathGenerator::update(double dt, int type, Vector6d& v_out) {
    current_time_ += dt;
    v_out.setZero();

    // TYPE 0: Hình tròn trên mặt phẳng XY
    if (type == 0) {
        double omega = (2.0 * M_PI) / period_; // Tốc độ góc của quỹ đạo
        
        // Phương trình tham số hình tròn: 
        // x = R*cos(wt), y = R*sin(wt)
        // Đạo hàm (Vận tốc):
        // vx = -R*w*sin(wt)
        // vy =  R*w*cos(wt)
        
        v_out(0) = -radius_ * omega * std::sin(omega * current_time_); // vx
        v_out(1) =  radius_ * omega * std::cos(omega * current_time_); // vy
        v_out(2) = 0.0; // vz
        // v_out(5) = omega; // Nếu muốn đầu robot xoay theo hướng tiếp tuyến (tùy chọn)
    }
    
    // TYPE 1: Đi thẳng theo trục Z (Ví dụ)
    else if (type == 1) {
        // Đi lên xuống hình sin
        v_out(2) = 0.05 * std::sin(current_time_); 
    }
}

} }