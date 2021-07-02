#ifndef IWBC_TASKS_HPP
#define IWBC_TASKS_HPP

#include <yaml-cpp/yaml.h>

#include <pinocchio/spatial/se3.hpp>

#include <tsid/contacts/contact-6d-ext.hpp>
#include <tsid/contacts/contact-point.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/math/fwd.hpp>
#include <tsid/math/utils.hpp>
#include <tsid/robots/fwd.hpp>
#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/trajectories/trajectory-base.hpp>

#include <inria_wbc/utils/factory.hpp>

namespace inria_wbc {

    inline tsid::trajectories::TrajectorySample to_sample(const Eigen::VectorXd& ref)
    {
        tsid::trajectories::TrajectorySample sample;
        sample.pos = ref;
        sample.vel.setZero(ref.size());
        sample.acc.setZero(ref.size());
        return sample;
    }

    inline tsid::trajectories::TrajectorySample to_sample(const pinocchio::SE3& ref)
    {
        tsid::trajectories::TrajectorySample sample;
        sample.resize(12, 6);
        tsid::math::SE3ToVector(ref, sample.pos);
        return sample;
    }

    namespace tasks {
        namespace cst {
            static constexpr double w_force_feet = 1e-3; // regularization force for contacts
        }

        using FactoryYAML = utils::Factory<
            tsid::tasks::TaskBase, // we create generic task (and downcast if needed)
            std::shared_ptr<tsid::robots::RobotWrapper>, // robot
            std::shared_ptr<tsid::InverseDynamicsFormulationAccForce>, // tsid
            std::string, //the name of the task
            YAML::Node, // the task node to parse
            YAML::Node // the controller node to parse
            >;
        template <typename T>
        using RegisterYAML = FactoryYAML::AutoRegister<T>;

        // contacts cannot be in the same factory
        std::shared_ptr<tsid::contacts::Contact6dExt> make_contact_task(
            const std::shared_ptr<tsid::robots::RobotWrapper>& robot,
            const std::shared_ptr<tsid::InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node);
    } // namespace tasks
} // namespace inria_wbc

#endif
