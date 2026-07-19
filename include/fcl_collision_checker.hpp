#pragma once

#include <memory>
#include <vector>

#include <fcl/fcl.h>
#include <ompl/base/State.h>
#include "robot_mechanism.hpp"

namespace motion_planning_examples
{

struct SquareObstacle
{
    double cx{0.0};
    double cy{0.0};
    double size{0.25};
};

class FCLCollisionChecker
{
public:
    FCLCollisionChecker(std::shared_ptr<RobotMechanism> arm,
                        double objectHeight,
                        const std::vector<SquareObstacle>& obstacles);

    [[nodiscard]] bool isStateValid(const ompl::base::State* state) const;

private:
    static bool inCollision(const fcl::CollisionObjectd& a, const fcl::CollisionObjectd& b);

    std::shared_ptr<RobotMechanism> arm_;
    std::vector<fcl::CollisionObjectd> obstacleObjects_; 
};

}  // namespace motion_planning_examples
