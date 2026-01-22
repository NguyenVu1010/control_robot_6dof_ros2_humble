#ifndef MY_ROBOT_CONTROLLERS__ALGO__PATH_GENERATOR_HPP_
#define MY_ROBOT_CONTROLLERS__ALGO__PATH_GENERATOR_HPP_

#include "types.hpp"
#include <cmath>

namespace my_robot_controllers {
namespace algo {

class PathGenerator {
public:
    PathGenerator();

    // Reset bộ đếm thời gian
    void reset();

    // Tính toán vận tốc tại thời điểm dt (delta time)
    // type: 0=Circle, 1=Line, ...
    void update(double dt, int type, Vector6d& v_out);

private:
    double current_time_;
    
    // Tham số quỹ đạo (Ví dụ hình tròn)
    double radius_ = 0.1; // Bán kính 10cm
    double period_ = 5.0; // Chu kỳ 5 giây 1 vòng
};

} }
#endif