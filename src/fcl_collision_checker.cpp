#include "fcl_collision_checker.hpp"

namespace motion_planning_examples
{

FCLCollisionChecker::FCLCollisionChecker(std::shared_ptr<TwoDOFPlanarArm> arm,
                                         double objectHeight,
                                         const std::vector<SquareObstacle>& obstacles)
    : arm_(std::move(arm))
{
    // Obstacles remain completely static, so we can store them directly as objects
    for (const auto& obstacle : obstacles)
    {
        auto geometry = std::make_shared<fcl::Boxd>(obstacle.size, obstacle.size, objectHeight);
        fcl::Transform3d transform = fcl::Transform3d::Identity();
        transform.translation() << obstacle.cx, obstacle.cy, 0.0;
        
        // Store as actual objects, avoiding extra pointers where possible
        obstacleObjects_.emplace_back(geometry, transform);
    }
}

bool FCLCollisionChecker::isStateValid(const ompl::base::State* state) const
{
    fcl::Transform3d tf1, tf2;
    arm_->computeForwardKinematics(state, tf1, tf2);

    // CRITICAL FIX: Allocate on the stack. No std::make_shared in the hot loop.
    fcl::CollisionObjectd link1(arm_->getLink1Geometry(), tf1);
    fcl::CollisionObjectd link2(arm_->getLink2Geometry(), tf2);

    for (const auto& obstacle : obstacleObjects_)
    {
        if (inCollision(link1, obstacle) || inCollision(link2, obstacle))
        {
            return false;
        }
    }
    return true;
}

bool FCLCollisionChecker::inCollision(const fcl::CollisionObjectd& a, const fcl::CollisionObjectd& b)
{
    fcl::CollisionRequestd request;
    request.num_max_contacts = 1;
    fcl::CollisionResultd result;
    // Note: passing memory addresses of stack objects to FCL
    return fcl::collide(&a, &b, request, result) > 0;
}

}  // namespace motion_planning_examples
