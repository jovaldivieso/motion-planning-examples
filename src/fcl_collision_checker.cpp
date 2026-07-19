#include "fcl_collision_checker.hpp"

namespace motion_planning_examples
{

FCLCollisionChecker::FCLCollisionChecker(std::shared_ptr<RobotMechanism> arm,
                                         double objectHeight,
                                         const std::vector<SquareObstacle>& obstacles)
    : arm_(std::move(arm))
{
    for (const auto& obstacle : obstacles)
    {
        auto geometry = std::make_shared<fcl::Boxd>(obstacle.size, obstacle.size, objectHeight);
        fcl::Transform3d transform = fcl::Transform3d::Identity();
        transform.translation() << obstacle.cx, obstacle.cy, 0.0;
        obstacleObjects_.emplace_back(geometry, transform);
    }
}

bool FCLCollisionChecker::isStateValid(const ompl::base::State* state) const
{
    std::vector<fcl::Transform3d> transforms;
    arm_->computeForwardKinematics(state, transforms);
    
    auto geometries = arm_->getCollisionGeometries();

    for (std::size_t i = 0; i < transforms.size(); ++i)
    {
        fcl::CollisionObjectd link(geometries[i], transforms[i]);
        for (const auto& obstacle : obstacleObjects_)
        {
            if (inCollision(link, obstacle))
            {
                return false;
            }
        }
    }
    return true;
}

bool FCLCollisionChecker::inCollision(const fcl::CollisionObjectd& a, const fcl::CollisionObjectd& b)
{
    fcl::CollisionRequestd request;
    request.num_max_contacts = 1;
    fcl::CollisionResultd result;
    return fcl::collide(&a, &b, request, result) > 0;
}

}  // namespace motion_planning_examples
