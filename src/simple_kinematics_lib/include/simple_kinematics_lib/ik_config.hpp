#ifndef SKL_IK_CONFIG_HPP_
#define SKL_IK_CONFIG_HPP_

namespace srk {

// Cấu hình các hệ số IK — có thể tune từ YAML parameter
struct IKConfig {
    // SVD per-axis damping
    double sigma_threshold = 0.02;   // Ngưỡng singular value bắt đầu damping
    double lambda_max = 0.1;         // Damping tối đa cho hướng gần singularity

    // Direction guard (hậu kiểm hướng)
    double direction_cos_threshold = 0.866;  // cos(30°), ngưỡng cosine hợp lệ
    double direction_min_scale = 0.1;        // Scale tối thiểu khi sai hướng

    // Safety
    double max_joint_speed = 3.0;    // Giới hạn vận tốc khớp (rad/s)

    // P-Controller gains (dùng trong solveIK_Position)
    double Kp_pos = 10.0;           // Gain vị trí tuyến tính
    double Kp_rot = 5.0;            // Gain vị trí góc

    // Cartesian velocity limits
    double max_lin_vel = 0.5;        // Vận tốc tuyến tính tối đa (m/s)
    double max_rot_vel = 1.0;        // Vận tốc góc tối đa (rad/s)
};

} // namespace srk

#endif // SKL_IK_CONFIG_HPP_
