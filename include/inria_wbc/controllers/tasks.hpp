#ifndef IWBC_TASKS_HPP
#define IWBC_TASKS_HPP

#include <yaml-cpp/yaml.h>

#include <pinocchio/spatial/se3.hpp>

#include <tsid/contacts/contact-6d-ext.hpp>
#include <tsid/contacts/contact-base.hpp>
#include <tsid/contacts/contact-point.hpp>
#include <tsid/measuredForces/measured-force-base.hpp>
#include <tsid/measuredForces/measured-force-6Dwrench.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/math/fwd.hpp>
#include <tsid/math/utils.hpp>
#include <tsid/robots/fwd.hpp>
#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/tasks/task-contact-force-equality.hpp>
#include <tsid/trajectories/trajectory-base.hpp>

#include <inria_wbc/utils/factory.hpp>

namespace inria_wbc {

    namespace tasks {
        namespace cst {
            static constexpr double w_force_feet = 1e-3; // regularization force for contacts TODO CHANGE FOR STABILIZATION!
        }

        using FactoryYAML = utils::Factory<
            tsid::tasks::TaskBase, // we create generic task (and downcast if needed)
            std::shared_ptr<tsid::robots::RobotWrapper>, // robot
            std::shared_ptr<tsid::InverseDynamicsFormulationAccForce>, // tsid
            std::string, //the name of the task
            YAML::Node, // the task node to parse
            YAML::Node, // the controller node to parse
            std::unordered_map<std::string, std::shared_ptr<tsid::contacts::ContactBase>>, //already added contacts,
            std::unordered_map<std::string, std::shared_ptr<tsid::measuredForces::MeasuredForceBase>>
            >;
        template <typename T>
        using RegisterYAML = FactoryYAML::AutoRegister<T>;

        // contacts cannot be in the same factory
        std::shared_ptr<tsid::contacts::Contact6dExt> make_contact_task(
            const std::shared_ptr<tsid::robots::RobotWrapper>& robot,
            const std::shared_ptr<tsid::InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node);

        // measured forces cannot be in the same factory
        std::shared_ptr<tsid::measuredForces::MeasuredForce6Dwrench> make_measured_force(
            const std::shared_ptr<tsid::robots::RobotWrapper>& robot,
            const std::shared_ptr<tsid::InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node);

    } // namespace tasks
} // namespace inria_wbc

#endif
