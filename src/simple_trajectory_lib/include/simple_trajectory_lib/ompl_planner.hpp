#pragma once
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <Eigen/Dense>
#include "simple_kinematics_lib/kinematics_core.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace my_robot_controllers {

class JointPathPlannerOMPL {
public:
    JointPathPlannerOMPL(std::shared_ptr<srk::KinematicsCore> kinematics)
        : kinematics_(kinematics) {
        
        // 1. Không gian cấu hình 6 chiều (cho 6 khớp)
        auto space = std::make_shared<ob::RealVectorStateSpace>(6);

        // 2. Thiết lập giới hạn khớp (từ -PI đến PI hoặc theo thực tế robot)
        ob::RealVectorBounds bounds(6);
        bounds.setLow(-15.0); // -2 * PI
        bounds.setHigh(15.0); // 2 * PI
        space->setBounds(bounds);

        si_ = std::make_shared<ob::SpaceInformation>(space);

        // 3. Hàm kiểm tra va chạm (Collision Detection)
        si_->setStateValidityChecker([this](const ob::State* state) {
            return isStateValid(state);
        });

        si_->setup();
    }

    // Thiết lập vật cản động từ Controller
    void setObstacle(const Eigen::Vector3d& center, double radius) {
        obs_center_ = center;
        obs_radius_ = radius;
    }

    bool isStateValid(const ob::State* state) {
        const auto* joints = state->as<ob::RealVectorStateSpace::StateType>();
        Eigen::VectorXd q(6);
        for (int i = 0; i < 6; ++i) q(i) = (*joints)[i];

        srk::Frame pose;
        kinematics_->solveFK(q, pose);
        Eigen::Vector3d pos = pose.translation();

        // 1. Kiểm tra va chạm sàn (Z < 0.01)
        if (pos.z() < 0.01) {
            // std::cout << "Invalid: Too close to floor Z=" << pos.z() << std::endl;
            return false;
        }

        // 2. Kiểm tra va chạm vật cản
        double dist = (pos - obs_center_).norm();
        if (dist < (obs_radius_ + 0.05)) { // 0.05 là khoảng an toàn (safety margin)
            // std::cout << "Invalid: Collision with Sphere! Dist=" << dist << std::endl;
            return false;
        }

        return true;
    }

    std::vector<Eigen::VectorXd> plan(const Eigen::VectorXd& start_q, const Eigen::VectorXd& goal_q) {
        auto pdef = std::make_shared<ob::ProblemDefinition>(si_);

        ob::ScopedState<> start(si_->getStateSpace());
        ob::ScopedState<> goal(si_->getStateSpace());
        for(int i=0; i<6; ++i) {
            start[i] = start_q(i);
            goal[i] = goal_q(i);
        }

        pdef->setStartAndGoalStates(start, goal);

        // Sử dụng RRT* - Thuật toán tìm đường tối ưu chuyên nghiệp
        auto planner = std::make_shared<og::RRTstar>(si_);
        planner->setProblemDefinition(pdef);
        planner->setup();

        ob::PlannerStatus solved = planner->ob::Planner::solve(1.0); // Tìm trong 1 giây

        if (solved) {
            og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
            // Làm mượt đường đi
            og::PathSimplifier simplifier(si_);
            simplifier.simplifyMax(*path);
            simplifier.smoothBSpline(*path);

            std::vector<Eigen::VectorXd> result;
            for (size_t i = 0; i < path->getStateCount(); ++i) {
                const auto* s = path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
                Eigen::VectorXd res_q(6);
                for(int j=0; j<6; ++j) res_q(j) = (*s)[j];
                result.push_back(res_q);
            }
            return result;
        }
        return {};
    }

private:
    std::shared_ptr<ob::SpaceInformation> si_;
    std::shared_ptr<srk::KinematicsCore> kinematics_;
    Eigen::Vector3d obs_center_;
    double obs_radius_;
};

} // namespace