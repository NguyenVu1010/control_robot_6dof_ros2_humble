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
    Frame T_origin;       // Transform từ cha đến khớp
    Eigen::Vector3d axis; // Trục quay

    // Tính transform dựa trên giá trị khớp q
    Frame pose(double q) const {
        if (type == FIXED) return T_origin;
        
        Eigen::Isometry3d T_joint = Eigen::Isometry3d::Identity();
        if (type == REVOLUTE) T_joint.linear() = Eigen::AngleAxisd(q, axis).toRotationMatrix();
        else if (type == PRISMATIC) T_joint.translation() = axis * q;
        
        return T_origin * T_joint;
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