#ifndef SKL_CHAIN_HPP_
#define SKL_CHAIN_HPP_

#include "types.hpp"
#include <vector>
#include <string>

namespace srk {

enum JointType { FIXED, REVOLUTE, PRISMATIC };

struct Segment {
    std::string name;
    JointType type;
    Frame T_origin;       // Transform từ Link Cha -> Khớp
    Eigen::Vector3d axis; // Trục quay của khớp

    // Chỉ lấy phần biến đổi do khớp gây ra (Xoay hoặc Tịnh tiến)
    Frame jointTransform(double q) const {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        if (type == REVOLUTE) T.linear() = Eigen::AngleAxisd(q, axis).toRotationMatrix();
        else if (type == PRISMATIC) T.translation() = axis * q;
        return T;
    }
};

class Chain {
public:
    void addSegment(const Segment& seg) {
        segments_.push_back(seg);
        if (seg.type != FIXED) nr_joints_++;
    }
    const std::vector<Segment>& getSegments() const { return segments_; }
    unsigned int getNrOfJoints() const { return nr_joints_; }

private:
    std::vector<Segment> segments_;
    unsigned int nr_joints_ = 0;
};

}
#endif