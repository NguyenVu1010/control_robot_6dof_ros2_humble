#ifndef MY_ROBOT_CONTROLLERS__ALGO__MATH_UTILS_HPP_
#define MY_ROBOT_CONTROLLERS__ALGO__MATH_UTILS_HPP_

#include "types.hpp"

namespace my_robot_controllers {
namespace algo {

class DlsMath {
public:
    DlsMath(unsigned int num_joints);

    // Giải bài toán động học nghịch vi phân: q_dot = J_dls_inverse * v_cart
    // Sử dụng Damped Least Squares để xử lý điểm kỳ dị (Singularity)
    void solve(const Matrix6Xd& jacobian, const Vector6d& v_cart, VectorXd& q_dot_out);

private:
    double lambda_sq_ = 0.0001; // Hệ số damping (lambda^2). Tăng lên nếu robot rung lắc gần điểm kỳ dị.
    
    // Biến Pre-allocated để đảm bảo Real-time (NO MALLOC)
    Matrix6d identity6_; 
    Matrix6d tmp_matrix_6x6_;
    Eigen::MatrixXd jacobian_pinv_; // Kích thước: num_joints x 6
};

} // namespace algo
} // namespace my_robot_controllers

#endif