#include "fcl_collision_checker.hpp"

namespace motion_planning_examples
{

FCLCollisionChecker::FCLCollisionChecker(std::shared_ptr<RobotMechanism> arm,
                                         double objectHeight,
                                         const std::vector<SquareObstacle>& obstacles)
    : arm_(std::move(arm))
{
    // Pre-allocate the exact capacity needed based on the robot's collision bodies
    fkTransformsCache_.reserve(arm_->getCollisionGeometries().size());

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
    // Clear simply resets the size to 0 but keeps the capacity, ensuring zero heap allocations
    fkTransformsCache_.clear();
    arm_->computeForwardKinematics(state, fkTransformsCache_);
    
    auto geometries = arm_->getCollisionGeometries();

    for (std::size_t i = 0; i < fkTransformsCache_.size(); ++i)
    {
        fcl::CollisionObjectd link(geometries[i], fkTransformsCache_[i]);
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

bool FCLCollisionChecker::isManifoldStateValid(const JointManifoldState &state) const
{
    fkTransformsCache_.clear();
    arm_->computeForwardKinematicsFromManifoldState(state, fkTransformsCache_);

    auto geometries = arm_->getCollisionGeometries();

    for (std::size_t i = 0; i < fkTransformsCache_.size(); ++i)
    {
        fcl::CollisionObjectd link(geometries[i], fkTransformsCache_[i]);
        for (const auto &obstacle : obstacleObjects_)
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