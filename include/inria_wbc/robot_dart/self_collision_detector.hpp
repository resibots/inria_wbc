#ifndef IWBC_ROBORT_DART_SELF_COLLISIONS_HPP_
#define IWBC_ROBORT_DART_SELF_COLLISIONS_HPP_

#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>

namespace inria_wbc {
    namespace robot_dart {
        class SelfCollisionDetector {
        public:
            SelfCollisionDetector(const std::shared_ptr<::robot_dart::Robot>& robot)
                : _robot(robot),
                  _collision_detector(dart::collision::CollisionDetector::getFactory()->create("bullet")),
                  _collision_group(_collision_detector->createCollisionGroup(robot->skeleton().get())),
                  _collision_option(false, 10000u, std::make_shared<dart::collision::BodyNodeCollisionFilter>())
            {
            }
            const std::vector<std::string>& collide()
            {
                _robot->skeleton()->enableSelfCollisionCheck();
                _robot->skeleton()->disableAdjacentBodyCheck();
                _collision_result.clear();
                _collision_detector->collide(_collision_group.get(), _collision_option, &_collision_result);
                auto& bodies = _collision_result.getCollidingBodyNodes();
                _collision_names.clear();
                for (auto& b : bodies)
                    _collision_names.push_back(b->getName());
                _robot->skeleton()->disableSelfCollisionCheck();
                return _collision_names;
            }

        protected:
            std::shared_ptr<::robot_dart::Robot> _robot;
            std::shared_ptr<dart::collision::CollisionDetector> _collision_detector;
            std::unique_ptr<dart::collision::CollisionGroup> _collision_group;
            dart::collision::CollisionOption _collision_option;
            dart::collision::CollisionResult _collision_result;
            std::vector<std::string> _collision_names;
        };
    } // namespace robot_dart
} // namespace inria_wbc
#endif