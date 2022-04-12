#ifndef IWBC_ROBORT_DART_EXTERNAL_COLLISIONS_HPP_
#define IWBC_ROBORT_DART_EXTERNAL_COLLISIONS_HPP_

#include "inria_wbc/exceptions.hpp"
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>

namespace inria_wbc {
    namespace robot_dart {
        class ExternalCollisionDetector {
        public:
            //check collisions between robot and external_robot
            ExternalCollisionDetector(
                const std::shared_ptr<::robot_dart::Robot>& robot,
                const std::shared_ptr<::robot_dart::Robot>& external_robot,
                const std::map<std::string, std::string> filter_body_names_pairs = {})
                : _robot(robot),
                  _external_robot(external_robot),
                  _collision_detector(dart::collision::CollisionDetector::getFactory()->create("fcl")),
                  _collision_group(_collision_detector->createCollisionGroup(robot->skeleton().get())),
                  _external_collision_group(_collision_detector->createCollisionGroup(external_robot->skeleton().get())),
                  _collision_filter(std::make_shared<dart::collision::BodyNodeCollisionFilter>()),
                  _collision_option(false, 10000u, _collision_filter)
            {

                for (auto& pair : filter_body_names_pairs) {
                    auto bd1 = robot->body_node(pair.first);
                    auto bd2 = external_robot->body_node(pair.second);
                    IWBC_ASSERT(bd1, "ExternalCollisionDetector check filter_body_names_pairs: ", pair.first, "is not in robot");
                    IWBC_ASSERT(bd2, "ExternalCollisionDetector check filter_body_names_pairs: ", pair.second, "is not in external_robot");
                    _collision_filter->addBodyNodePairToBlackList(bd1, bd2);
                }
            }
            const std::vector<std::string>& collide()
            {
                _collision_result.clear();
                _collision_detector->collide(_collision_group.get(), _external_collision_group.get(), _collision_option, &_collision_result);
                auto& bodies = _collision_result.getCollidingBodyNodes();
                _collision_names.clear();
                for (auto& b : bodies)
                    _collision_names.push_back(b->getName());
                return _collision_names;
            }

        protected:
            std::shared_ptr<::robot_dart::Robot> _robot, _external_robot;
            std::shared_ptr<dart::collision::CollisionDetector> _collision_detector;
            std::unique_ptr<dart::collision::CollisionGroup> _collision_group, _external_collision_group;
            std::shared_ptr<dart::collision::BodyNodeCollisionFilter> _collision_filter;
            dart::collision::CollisionOption _collision_option;
            dart::collision::CollisionResult _collision_result;
            std::vector<std::string> _collision_names;
        };
    } // namespace robot_dart
} // namespace inria_wbc
#endif