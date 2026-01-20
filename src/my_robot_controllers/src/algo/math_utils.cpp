#include "my_robot_controllers/algo/math_utils.hpp"
#include <eigen3/Eigen/LU> // Dùng cho hàm .inverse()

namespace my_robot_controllers {
namespace algo {

DlsMath::DlsMath(unsigned int num_joints) {
    // Cấp phát bộ nhớ 1 lần duy nhất
    identity6_.setIdentity();
    tmp_matrix_6x6_.setZero();
    jacobian_pinv_.resize(num_joints, 6);
    jacobian_pinv_.setZero();
}

void DlsMath::solve(const Matrix6Xd& jacobian, const Vector6d& v_cart, VectorXd& q_dot_out) {
    // Công thức Damped Least Squares (DLS):
    // J_pinv = J^T * (J * J^T + lambda^2 * I)^-1
    
    // 1. Tính J * J^T (Kết quả là ma trận 6x6)
    tmp_matrix_6x6_ = jacobian * jacobian.transpose();
    
    // 2. Cộng hệ số Damping: (JJ^T + lambda^2*I)
    // tmp_matrix_6x6_.noalias() giúp Eigen tối ưu không tạo biến tạm
    tmp_matrix_6x6_.noalias() += lambda_sq_ * identity6_;
    
    // 3. Tính nghịch đảo ma trận 6x6 (Khá nhanh) và nhân J^T
    // J_pinv = J.T * inv(JJT + lam*I)
    jacobian_pinv_ = jacobian.transpose() * tmp_matrix_6x6_.inverse();

    // 4. Tính vận tốc khớp: q_dot = J_pinv * v_cart
    q_dot_out = jacobian_pinv_ * v_cart;
}

} // namespace algo
} // namespace my_robot_controllers