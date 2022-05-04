#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-actuation-equality.hpp>
#include <tsid/tasks/task-angular-momentum-equality.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-cop-equality.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-momentum-equality.hpp>
#include <tsid/tasks/task-se3-equality.hpp>

#include "inria_wbc/controllers/tasks.hpp"
#include "inria_wbc/trajs/utils.hpp"
#include "tsid/tasks/task-self-collision.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace tasks {
        // note on levels: everything is level 1 except the constraints:
        // - the bounds (level 0)
        // - the contacts (level 0)
        // future versions of TSID might have more levels (but not for now, 2020)

        template <int S>
        Eigen::Array<double, S, 1> convert_mask(const std::string& mask_str)
        {
            assert(S == Eigen::Dynamic || mask_str.size() == S);
            Eigen::Array<double, S, 1> mask;
            mask.resize(mask_str.size()); // useful for dynamic
            for (int i = 0; i < mask_str.size(); ++i)
                mask[i] = mask_str[i] == '0' ? 0 : 1;
            return mask;
        }

        ////// SE3 (warning: add the task to TSID!) //////
        std::shared_ptr<tsid::tasks::TaskBase> make_se3(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node)
        {

            // retrieve parameters from YAML
            double kp = IWBC_CHECK(node["kp"].as<double>());
            auto tracked = IWBC_CHECK(node["tracked"].as<std::string>());
            auto mask_str = IWBC_CHECK(node["mask"].as<std::string>());
            auto weight = IWBC_CHECK(node["weight"].as<double>());

            // convert the mask
            IWBC_ASSERT(mask_str.size() == 6, "SE3 masks needs to be 6D (x y z r p y), here:", mask_str);
            auto mask = convert_mask<6>(mask_str);

            // create the task
            assert(tsid);
            assert(robot);
            auto task = std::make_shared<tsid::tasks::TaskSE3Equality>(task_name, *robot, tracked);
            task->Kp(kp * Vector::Ones(6));
            task->Kd(2.0 * task->Kp().cwiseSqrt());
            task->setMask(mask.matrix());

            // set the reference
            // we need to check if this is a joint or a frame
            bool joint = robot->model().existJointName(tracked);
            bool body = robot->model().existBodyName(tracked);
            if (joint && body)
                throw IWBC_EXCEPTION("Ambiguous name to track for task ", task_name, ": this is both a joint and a frame [", tracked, "]");
            if (!joint && !body)
                throw IWBC_EXCEPTION("Unknown frame or joint [", tracked, "]");
            pinocchio::SE3 ref;

            if (joint)
                ref = robot->position(tsid->data(), robot->model().getJointId(tracked));
            else
                ref = robot->framePosition(tsid->data(), robot->model().getFrameId(tracked));
            auto sample = trajs::to_sample(ref);
            task->setReference(sample);

            // add the task to TSID (side effect, be careful)
            tsid->addMotionTask(*task, weight, 1);

            return task;
        }

        RegisterYAML<tsid::tasks::TaskSE3Equality> __register_se3_equality("se3", make_se3);

        ////// CoM (center of mass) //////
        std::shared_ptr<tsid::tasks::TaskBase> make_com(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node)
        {
            assert(tsid);
            assert(robot);

            // parse yaml
            double kp = IWBC_CHECK(node["kp"].as<double>());
            auto mask_str = IWBC_CHECK(node["mask"].as<std::string>());
            auto weight = IWBC_CHECK(node["weight"].as<double>());

            IWBC_ASSERT(mask_str.size() == 3, "CoM masks needs to be 3D (x y z), here:", mask_str);
            auto mask = convert_mask<3>(mask_str);

            // create the task
            auto task = std::make_shared<tsid::tasks::TaskComEquality>(task_name, *robot);
            task->Kp(kp * Vector::Ones(3));
            task->Kd(2.0 * task->Kp().cwiseSqrt());
            task->setMask(mask);

            // set the reference
            task->setReference(trajs::to_sample(robot->com(tsid->data())));

            // add to TSID
            tsid->addMotionTask(*task, weight, 1);

            return task;
        }

        RegisterYAML<tsid::tasks::TaskComEquality> __register_com_equality("com", make_com);

        ////// Momentum //////
        std::shared_ptr<tsid::tasks::TaskBase> make_momentum(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node)
        {
            assert(tsid);
            assert(robot);

            // parse yaml
            double kp = node["kp"].as<double>();
            auto weight = node["weight"].as<double>();

            auto mask_str = IWBC_CHECK(node["mask"].as<std::string>());
            IWBC_ASSERT(mask_str.size() == 6, "Momentum mask needs to be 6D (angular_x, angular_y, angular_z, linear_x, linear_y, linear_z), here:", mask_str);
            auto mask = convert_mask<6>(mask_str);

            // create the task
            auto task = std::make_shared<tsid::tasks::TaskMEquality>(task_name, *robot);
            task->Kp(kp * Vector::Ones(6));
            task->Kd(2.0 * task->Kp().cwiseSqrt());
            task->setMask(mask);

            // set the reference
            Eigen::VectorXd ref = Eigen::VectorXd::Zero(6);
            task->setReference(trajs::to_sample(ref));

            // add to TSID
            tsid->addMotionTask(*task, weight, 1);

            return task;
        }
        RegisterYAML<tsid::tasks::TaskMEquality> __register_linear_momentum_equality("momentum", make_momentum);

        ////// COP task //////
        std::shared_ptr<tsid::tasks::TaskBase> make_cop(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node)
        {
            assert(tsid);
            assert(robot);

            // parse yaml
            auto weight = node["weight"].as<double>();

            // create the task
            auto task = std::make_shared<tsid::tasks::TaskCopEquality>(task_name, *robot);

            // set the reference
            task->setReference(Eigen::Vector3d(0, 0, 0));

            // add to TSID
            tsid->addForceTask(*task, weight, 1);

            return task;
        }
        RegisterYAML<tsid::tasks::TaskCopEquality> __register_cop_equality("cop", make_cop);

        ////// Posture //////
        std::shared_ptr<tsid::tasks::TaskBase> make_posture(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node)
        {
            assert(tsid);
            assert(robot);

            // parse yaml
            double kp = IWBC_CHECK(node["kp"].as<double>());
            auto weight = IWBC_CHECK(node["weight"].as<double>());
            auto ref_name = IWBC_CHECK(node["ref"].as<std::string>());

            IWBC_ASSERT(robot->model().referenceConfigurations.count(ref_name) == 1, "Reference name ", ref_name, " not found");
            auto ref_q = robot->model().referenceConfigurations[ref_name];

            bool floating_base_flag = (robot->na() == robot->nv()) ? false : true;
            int n_actuated = floating_base_flag ? robot->nv() - 6 : robot->nv();

            // create the task
            auto task = std::make_shared<tsid::tasks::TaskJointPosture>(task_name, *robot);

            task->Kp(kp * Vector::Ones(n_actuated));
            task->Kd(2.0 * task->Kp().cwiseSqrt());
            Vector mask_post(n_actuated);
            if (!node["mask"]) {
                mask_post = Vector::Ones(n_actuated);
            }
            else {
                auto mask = IWBC_CHECK(node["mask"].as<std::string>());
                IWBC_ASSERT(mask.size() == mask_post.size(), "wrong size in posture mask, expected:", mask_post.size(), " got:", mask.size());
                mask_post = convert_mask<Eigen::Dynamic>(mask);
            }
            task->setMask(mask_post);

            // set the reference to the current position of the robot
            task->setReference(trajs::to_sample(ref_q.tail(robot->na())));

            // add the task
            tsid->addMotionTask(*task, weight, 1);

            return task;
        }
        RegisterYAML<tsid::tasks::TaskJointPosture> __register_posture("posture", make_posture);

        ////// Torques //////
        std::shared_ptr<tsid::tasks::TaskBase> make_torque(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node)
        {
            assert(tsid);
            assert(robot);

            // parse yaml
            auto weight = IWBC_CHECK(node["weight"].as<double>());

            bool floating_base_flag = (robot->na() == robot->nv()) ? false : true;
            int n_actuated = floating_base_flag ? robot->nv() - 6 : robot->nv();

            // create the task
            auto task = std::make_shared<tsid::tasks::TaskActuationEquality>(task_name, *robot);

            Vector mask_post(n_actuated);
            if (!node["mask"]) {
                mask_post = Vector::Ones(n_actuated);
            }
            else {
                auto mask = IWBC_CHECK(node["mask"].as<std::string>());
                IWBC_ASSERT(mask.size() == mask_post.size(), "wrong size in torque mask, expected:", mask_post.size(), " got:", mask.size());
                mask_post = convert_mask<Eigen::Dynamic>(mask);
            }
            task->mask(mask_post);

            if (node["scaling"]){
                auto scaling = IWBC_CHECK(node["scaling"].as<std::vector<double>>());
                IWBC_ASSERT(scaling.size() == n_actuated, "wrong size in torque scaling, expected:", n_actuated, " got:", scaling.size());
                Eigen::VectorXd scaling_post = Eigen::VectorXd::Map(scaling.data(), scaling.size());
                task->setWeightVector(scaling_post);
            }

            // set the reference to the zero
            Vector ref = Vector::Zero(n_actuated);
            task->setReference(ref);

            // add the task
            tsid->addActuationTask(*task, weight, 1);

            return task;
        }
        RegisterYAML<tsid::tasks::TaskActuationEquality> __register_torque("torque", make_torque);

        ////// Bounds //////
        std::shared_ptr<tsid::tasks::TaskBase> make_bounds(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node)
        {
            assert(tsid);
            assert(robot);

            // parse yaml
            auto weight = IWBC_CHECK(node["weight"].as<double>());
            auto dt = IWBC_CHECK(controller_node["CONTROLLER"]["dt"].as<double>());

            // create the task
            auto task = std::make_shared<tsid::tasks::TaskJointPosVelAccBounds>(task_name, *robot, dt, false);
            auto dq_max = robot->model().velocityLimit.tail(robot->na());
            auto ddq_max = dq_max / dt;
            task->setVelocityBounds(dq_max);
            task->setAccelerationBounds(ddq_max);
            auto q_lb = robot->model().lowerPositionLimit.tail(robot->na());
            auto q_ub = robot->model().upperPositionLimit.tail(robot->na());
            task->setPositionBounds(q_lb, q_ub);
            // add the task
            tsid->addMotionTask(*task, weight, 0);

            return task;
        }
        RegisterYAML<tsid::tasks::TaskJointPosVelAccBounds> __register_bounds("bounds", make_bounds);

        ////// Actuation Bounds //////
        std::shared_ptr<tsid::tasks::TaskBase> make_actuation_bounds(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node)
        {
            assert(tsid);
            assert(robot);

            // parse yaml
            auto weight = IWBC_CHECK(node["weight"].as<double>());

            // create the task
            auto task = std::make_shared<tsid::tasks::TaskActuationBounds>(task_name, *robot);
            auto tau_max = robot->model().effortLimit.tail(robot->na());
            task->setBounds(-tau_max, tau_max);

            // add the task
            tsid->addActuationTask(*task, weight, 0);

            return task;
        }
        RegisterYAML<tsid::tasks::TaskActuationBounds> __register_actuation_bounds("actuation-bounds", make_actuation_bounds);

        /*
        ////// Contacts //////
        /// this looks like a task, but this does not derive from tsid::task::TaskBase
        std::shared_ptr<tsid::contacts::Contact6dExt> make_contact_task(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node)
        {
            assert(tsid);
            assert(robot);

            // parse yaml
            auto kp = IWBC_CHECK(node["kp"].as<double>());
            auto joint_name = IWBC_CHECK(node["joint"].as<std::string>());
            auto lxn = IWBC_CHECK(node["lxn"].as<double>());
            auto lyn = IWBC_CHECK(node["lyn"].as<double>());
            auto lxp = IWBC_CHECK(node["lxp"].as<double>());
            auto lyp = IWBC_CHECK(node["lyp"].as<double>());
            auto lz = IWBC_CHECK(node["lz"].as<double>());
            auto mu = IWBC_CHECK(node["mu"].as<double>());
            auto normal = IWBC_CHECK(node["normal"].as<std::vector<double>>());
            auto fmin = IWBC_CHECK(node["fmin"].as<double>());
            auto fmax = IWBC_CHECK(node["fmax"].as<double>());
            IWBC_ASSERT(normal.size() == 3, "normal size:", normal.size());
            IWBC_ASSERT(robot->model().existFrame(joint_name), joint_name, " does not exist!");
            auto activate = IWBC_CHECK(node["activate"].as<bool>());
            auto x = IWBC_CHECK(node["x"].as<double>());
            auto y = IWBC_CHECK(node["y"].as<double>());
            auto z = IWBC_CHECK(node["z"].as<double>());
    
            Matrix3x contact_points(3, 4);
            Eigen::Vector3d contact_normal(normal.data());
            bool horizontal;
            if (normal.at(0) == 0. && normal.at(1) == 0){
                // Horizontal plan (default for the foot contacts)
                horizontal = true;
                contact_points <<   -lxn, -lxn, lxp, lxp,
                                -lyn, lyp, -lyn, lyp,
                                lz, lz, lz, lz;  // z constant = horizontal
            } else {
                horizontal = false;
                Eigen::Vector3d vertical = {0., 0., 1.};
                Eigen::Vector3d u = vertical.cross(contact_normal);
                u.normalize();
                Eigen::Vector3d v = contact_normal.cross(u);
                Eigen::Vector3d tl = -lxn * u + lyp * v;
                Eigen::Vector3d tr = lxp * u + lyp * v;
                Eigen::Vector3d br = lxp * u - lyn * v;
                Eigen::Vector3d bl = -lxn * u - lyn * v;
                contact_points << bl[0],  tl[0],  br[0],   bl[0], 
                                bl[1],  tl[1],  br[1],   bl[1],
                                bl[2],  tl[2],  br[2],   bl[2];
            }

            auto contact_task = std::make_shared<tsid::contacts::Contact6dExt>(task_name, *robot, joint_name, contact_points, contact_normal, mu, fmin, fmax);
            contact_task->Kp(kp * Vector::Ones(6));
            contact_task->Kd(2.0 * contact_task->Kp().cwiseSqrt());
            auto contact_ref = robot->framePosition(tsid->data(), robot->model().getFrameId(joint_name));
            if (!horizontal){
                contact_ref.translation()[0] = x;
                contact_ref.translation()[1] = y;
                contact_ref.translation()[2] = z;
            }
            // hand orientation
            Eigen::Quaterniond q;
            if (!horizontal) {
                auto mask = contact_task->getMotionTask().getMask();
                mask[0] = 1.;
                mask[1] = 1.;
                mask[2] = 1.;
                mask[3] = 0.;
                mask[4] = 0.;
                mask[5] = 0.;
                contact_task->setMask(mask);
            }

            contact_task->Contact6d::setReference(contact_ref);
            // add the task
            if (activate)
                tsid->addRigidContact(*contact_task, cst::w_force_feet);

            return contact_task;
        }*/

        ////// Contacts //////
        /// this looks like a task, but this does not derive from tsid::task::TaskBase
        std::shared_ptr<tsid::contacts::Contact6dExt> make_contact_task(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node)
        {
            assert(tsid);
            assert(robot);

            // parse yaml
            auto kp = IWBC_CHECK(node["kp"].as<double>());
            auto joint_name = IWBC_CHECK(node["joint"].as<std::string>());
            auto lxn = IWBC_CHECK(node["lxn"].as<double>());
            auto lyn = IWBC_CHECK(node["lyn"].as<double>());
            auto lxp = IWBC_CHECK(node["lxp"].as<double>()); 
            auto lyp = IWBC_CHECK(node["lyp"].as<double>());
            auto lz = IWBC_CHECK(node["lz"].as<double>());
            auto mu = IWBC_CHECK(node["mu"].as<double>());
            auto normal = IWBC_CHECK(node["normal"].as<std::vector<double>>());
            auto fmin = IWBC_CHECK(node["fmin"].as<double>());
            auto fmax = IWBC_CHECK(node["fmax"].as<double>());
            IWBC_ASSERT(normal.size() == 3, "normal size:", normal.size());
            IWBC_ASSERT(robot->model().existFrame(joint_name), joint_name, " does not exist!");
            auto activate = IWBC_CHECK(node["activate"].as<bool>());
            auto x = IWBC_CHECK(node["x"].as<double>());
            auto y = IWBC_CHECK(node["y"].as<double>());
            auto z = IWBC_CHECK(node["z"].as<double>());

            Matrix3x contact_points(3, 4);
            Eigen::Vector3d contact_normal(normal.data());
            bool horizontal;
            if (normal.at(0) == 0. && normal.at(1) == 0){
                // Horizontal plan (default for the foot contacts)
                horizontal = true;
                contact_points <<   -lxn, -lxn, lxp, lxp,
                                -lyn, lyp, -lyn, lyp,
                                lz, lz, lz, lz;  // z constant = horizontal
            } else {
                horizontal = false;
                Eigen::Vector3d vertical = {0., 0., 1.};
                Eigen::Vector3d u = vertical.cross(contact_normal);
                u.normalize();
                Eigen::Vector3d v = contact_normal.cross(u);
                Eigen::Vector3d tl = -lxn * u + lyp * v;
                Eigen::Vector3d tr = lxp * u + lyp * v;
                Eigen::Vector3d br = lxp * u - lyn * v;
                Eigen::Vector3d bl = -lxn * u - lyn * v;
                contact_points << bl[0],  tl[0],  br[0],   bl[0], 
                                bl[1],  tl[1],  br[1],   bl[1],
                                bl[2],  tl[2],  br[2],   bl[2];
            }

            auto contact_task = std::make_shared<tsid::contacts::Contact6dExt>(task_name, *robot, joint_name, contact_points, contact_normal, mu, fmin, fmax);
            contact_task->Kp(kp * Vector::Ones(6));
            contact_task->Kd(2.0 * contact_task->Kp().cwiseSqrt());
            auto contact_ref = robot->framePosition(tsid->data(), robot->model().getFrameId(joint_name));
            if (!horizontal){
                contact_ref.translation()[0] = x;
                contact_ref.translation()[1] = y;
                contact_ref.translation()[2] = z;
            }
            // hand orientation
            Eigen::Quaterniond q;
            if (!horizontal) {
                auto mask = contact_task->getMotionTask().getMask();
                mask[0] = 1.;
                mask[1] = 1.;
                mask[2] = 1.;
                mask[3] = 0.;
                mask[4] = 0.;
                mask[5] = 0.;
                contact_task->setMask(mask);
            }

            contact_task->Contact6d::setReference(contact_ref);
            // add the task
            if (activate)
                tsid->addRigidContact(*contact_task, cst::w_force_feet);

            return contact_task;
        }

        ////// Self-collision (warning: add the task to TSID!) //////
        std::shared_ptr<tsid::tasks::TaskBase> make_self_collision(
            const std::shared_ptr<robots::RobotWrapper>& robot,
            const std::shared_ptr<InverseDynamicsFormulationAccForce>& tsid,
            const std::string& task_name, const YAML::Node& node, const YAML::Node& controller_node)
        {

            // retrieve parameters from YAML
            double kp = IWBC_CHECK(node["kp"].as<double>());
            auto tracked = IWBC_CHECK(node["tracked"].as<std::string>());
            auto weight = IWBC_CHECK(node["weight"].as<double>());
            auto p = IWBC_CHECK(node["p"].as<double>());
            auto radius = IWBC_CHECK(node["radius"].as<double>());

            std::unordered_map<std::string, double> avoided;
            for (const auto& a : IWBC_CHECK(node["avoided"])) {
                IWBC_ASSERT(robot->model().existFrame(a.first.as<std::string>()), "Frame ", a.first.as<std::string>(), " in ", task_name, " does not exists.");
                avoided[a.first.as<std::string>()] = a.second.as<double>();
            }

            // create the task
            assert(tsid);
            assert(robot);
            auto task = std::make_shared<tsid::tasks::TaskSelfCollision>(task_name, *robot, tracked, avoided, radius, p);
            task->Kp(kp);
            task->Kd(2.0 * sqrt(task->Kp()));

            // add the task to TSID (side effect, be careful)
            tsid->addMotionTask(*task, weight, 1);

            return task;
        }
        RegisterYAML<tsid::tasks::TaskSelfCollision> __register_self_collision("self-collision", make_self_collision);

    } // namespace tasks
} // namespace inria_wbc
