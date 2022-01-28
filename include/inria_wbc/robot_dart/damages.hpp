#ifndef IWBC_ROBORT_DART_DAMAGES_HPP_
#define IWBC_ROBORT_DART_DAMAGES_HPP_

#include <inria_wbc/exceptions.hpp>

#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/sensor/sensor.hpp>

#include <dart/dynamics/BodyNode.hpp>

namespace inria_wbc {
    namespace robot_dart {

        static constexpr int CUT = 0;
        static constexpr int LOCKED = 1;
        static constexpr int PASSIVE = 2;
        static constexpr int MASS = 3;
        static constexpr int LIMIT = 4;

        class RobotDamages {
        public:
            struct damage {
                int damage_type;
                std::vector<std::string> affected_joints;
                std::vector<std::shared_ptr<::robot_dart::sensor::Sensor>> affected_sensors;
            };

            RobotDamages(const std::shared_ptr<::robot_dart::Robot>& robot,
                const std::shared_ptr<::robot_dart::RobotDARTSimu>& simu,
                const std::vector<std::string> controllable_dofs,
                const std::vector<std::string> all_dofs)
                : _robot(robot), _simu(simu), _controllable_dofs(controllable_dofs), _all_dofs(all_dofs)
            {
                _damaged = false;
                _sensors = _simu->sensors();
                _active_joints_mask.resize(_all_dofs.size());
                _active_joints_mask.setConstant(1.0); //1 if joint in robot, 0 if damaged
                _active_dofs = _all_dofs;
                _active_dofs_controllable = _controllable_dofs;
            }

            void cut(const std::string& link_name)
            {
                auto vec = _robot->body_names();
                if (std::find(vec.begin(), vec.end(), link_name) == vec.end())
                    IWBC_ERROR("cut ", link_name, " is not a valid robot body name");

                _damaged = true;
                std::vector<std::string> affected_joints;
                std::vector<std::shared_ptr<::robot_dart::sensor::Sensor>> affected_sensors;

                auto body_node = _robot->skeleton()->getBodyNode(link_name);
                auto tmp_skel = body_node->split("tmp");

                for (size_t i = 0; i < tmp_skel->getNumJoints(); ++i) {
                    auto name = tmp_skel->getJoint(i)->getName();
                    _removed_joints.push_back(name);
                    affected_joints.push_back(name);
                    update_sensor(name, affected_sensors);
                }

                _robot->update_joint_dof_maps();
                update_joint_lists();

                damage cut_damage = {CUT, affected_joints, affected_sensors};
                _damage_list.push_back(cut_damage);
            }

            void motor_damage(const std::string& joint_name, int damage_type)
            {
                if (_robot->joint_map().find(joint_name) == _robot->joint_map().end())
                    IWBC_ERROR("motor_damage ", joint_name, " not in the robot joint map");

                _damaged = true;
                std::vector<std::string> affected_joints = {joint_name};
                std::vector<std::shared_ptr<::robot_dart::sensor::Sensor>> affected_sensors;
                _removed_joints.push_back(joint_name);
                update_sensor(joint_name, affected_sensors);
                update_joint_lists();

                auto jt = _robot->skeleton()->getJoint(joint_name);
                if (damage_type == LOCKED)
                    _robot->set_actuator_type("locked", joint_name);
                if (damage_type == PASSIVE)
                    _robot->set_actuator_type("passive", joint_name);

                _robot->update_joint_dof_maps();
                damage motor_damage = {damage_type, affected_joints, affected_sensors};
                _damage_list.push_back(motor_damage);
            }

            void update_sensor(const std::string& joint_name, std::vector<std::shared_ptr<::robot_dart::sensor::Sensor>>& affected_sensors)
            {
                for (auto& s : _sensors) {
                    if (s->active() && joint_name == s->attached_to()) {
                        s->detach();
                        _removed_sensors.push_back(s);
                        affected_sensors.push_back(s);
                    }
                }
            }

            void update_joint_lists()
            {
                _active_dofs_controllable.clear();
                _active_dofs.clear();

                for (auto& x : _controllable_dofs)
                    if (std::find(_removed_joints.begin(), _removed_joints.end(), x) == _removed_joints.end())
                        _active_dofs_controllable.push_back(x);
                for (int i = 0; i < _all_dofs.size(); i++) {
                    if (std::find(_removed_joints.begin(), _removed_joints.end(), _all_dofs[i]) == _removed_joints.end()) {
                        _active_dofs.push_back(_all_dofs[i]);
                    }
                    else {
                        _active_joints_mask[i] = 0;
                    }
                }
            }

            // TO DO
            // void change_mass(const std::string& joint_name);
            // void change_limit(const std::string& joint_name, int limit_type);

            bool is_damaged() { return _damaged; };
            std::vector<damage> damage_list() { return _damage_list; };

            std::vector<std::string> active_dofs() { return _active_dofs; };
            std::vector<std::string> active_dofs_controllable() { return _active_dofs_controllable; };
            std::vector<std::string> removed_joints() { return _removed_joints; };
            Eigen::VectorXd active_joints_mask() { return _active_joints_mask; };

        protected:
            bool _damaged;
            std::vector<damage> _damage_list;

            std::shared_ptr<::robot_dart::Robot> _robot;
            std::shared_ptr<::robot_dart::RobotDARTSimu> _simu;

            std::vector<std::shared_ptr<::robot_dart::sensor::Sensor>> _sensors;
            std::vector<std::shared_ptr<::robot_dart::sensor::Sensor>> _removed_sensors;

            std::vector<std::string> _all_dofs;
            std::vector<std::string> _controllable_dofs;

            std::vector<std::string> _active_dofs;
            std::vector<std::string> _active_dofs_controllable;

            std::vector<std::string> _removed_joints;
            Eigen::VectorXd _active_joints_mask;
        };
    } // namespace robot_dart
} // namespace inria_wbc
#endif