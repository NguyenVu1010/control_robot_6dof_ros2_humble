#include "simple_kinematics_lib/chain.hpp"
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <algorithm>
#include <iostream>

namespace srk {

// Hàm helper chuyển đổi từ URDF sang Eigen Frame
Frame toFrame(const urdf::Pose& p) {
    Frame f = Frame::Identity();
    f.translation() << p.position.x, p.position.y, p.position.z;
    double qx, qy, qz, qw;
    p.rotation.getQuaternion(qx, qy, qz, qw);
    f.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
    return f;
}

bool parseURDF(const std::string& xml, const std::string& base, const std::string& tip, Chain& chain) {
    auto model = urdf::parseURDF(xml);
    if (!model) return false;

    auto link = model->getLink(tip);
    if (!link) return false;

    std::vector<Segment> segments;

    while (link && link->name != base) {
        auto joint = link->parent_joint;
        if (!joint) break;

        Segment seg;
        seg.name = joint->name;
        seg.T_origin = toFrame(joint->parent_to_joint_origin_transform);
        seg.axis << joint->axis.x, joint->axis.y, joint->axis.z;
        seg.axis.normalize();

        if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS)
            seg.type = REVOLUTE;
        else if (joint->type == urdf::Joint::PRISMATIC)
            seg.type = PRISMATIC;
        else
            seg.type = FIXED;

        segments.push_back(seg);
        link = model->getLink(joint->parent_link_name);
    }

    if (!link || link->name != base) return false;

    std::reverse(segments.begin(), segments.end());
    for (const auto& s : segments) chain.addSegment(s);
    
    return true;
}

}