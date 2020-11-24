/* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "Eigen/Core"
#include "map"
#include "vector"
#include <iomanip>
#include <memory>
#include <utility>

#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/utils/stop-watch.hpp>

#include "inria_wbc/controllers/franka_cartesian_pos_tracking.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {

        static Register<FrankaCartPosTracking> __franka_cartesian_pos_tracking("franka-cartesian-pos-tracking");

        FrankaCartPosTracking::FrankaCartPosTracking(const Params& params) : Controller(params)
        {
            if (!params.sot_config_path.empty())
                parse_configuration_yaml(params.sot_config_path);

            set_stack_configuration(); //~~????
        }

        void FrankaCartPosTracking::set_default_opt_params(std::map<std::string, double>& p)
        {
            p["w_ee"] = 1000.0;
            p["kp_ee"] = 30.0; 
        }

        //~~???? HERE
        void TalosPosTracking::parse_configuration_yaml(const std::string& sot_config_path)
        {
            std::ifstream yamlConfigFile(sot_config_path);
            if (verbose_)
                std::cout << "[controller] Taking the parameters in " << sot_config_path << std::endl;

            // for the opt_params (task weight and gains) we keep the value if we have it
            // if not found, we look at the YAML file
            // if not found, we take the default value from the map (set_defautlt_opt_params)
            opt_params_t p;
            set_default_opt_params(p);
            YAML::Node config = YAML::LoadFile(sot_config_path);
            for (auto& x : p)
                if (params_.opt_params.find(x.first) == params_.opt_params.end())
                    if (!utils::parse(params_.opt_params[x.first], x.first, config, "CONTROLLER", verbose_))
                        params_.opt_params[x.first] = p[x.first];

            utils::parse(ref_config_, "ref_config", config, "CONTROLLER", verbose_);
            if (verbose_)
                std::cout << "[controller] Taking the reference configuration from " << ref_config_ << std::endl;

            // stabilizer
            inria_wbc::utils::parse(_use_stabilizer, "activated", config, "STABILIZER");
            inria_wbc::utils::parse(_stabilizer_p(0), "p_x", config, "STABILIZER");
            inria_wbc::utils::parse(_stabilizer_p(1), "p_y", config, "STABILIZER");
            inria_wbc::utils::parse(_stabilizer_d(0), "d_x", config, "STABILIZER");
            inria_wbc::utils::parse(_stabilizer_d(1), "d_y", config, "STABILIZER");
            int history = _cop_estimator.history_size();
            inria_wbc::utils::parse(history, "filter_size", config, "STABILIZER");
            _cop_estimator.set_history_size(history);

            if (verbose_) {
                std::cout << "Stabilizer:" << _use_stabilizer << std::endl;
                std::cout << "P:" << _stabilizer_p.transpose() << std::endl;
                std::cout << "D:" << _stabilizer_d.transpose() << std::endl;
            }
        }

        void TalosPosTracking::set_stack_configuration()
        {

            const opt_params_t& p = params_.opt_params;
            if (verbose_)
                for (auto& x : p)
                    std::cout << x.first << " => " << params_.opt_params[x.first] << std::endl;

            ////////////////////Gather Initial Pose //////////////////////////////////////
            q_tsid = robot_->model().referenceConfigurations[ref_config_];
            q0_=q_tsid;

            ////////////////////Create the inverse-dynamics formulation///////////////////
            tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);

            ////////////////////Create an HQP solver /////////////////////////////////////
            using solver_t = std::shared_ptr<solvers::SolverHQPBase>;
            solver_ = solver_t(solvers::SolverHQPFactory::createNewSolver(solvers::SOLVER_HQP_EIQUADPROG_FAST, "solver-eiquadprog"));
            solver_->resize(tsid_->nVar(), tsid_->nEq(), tsid_->nIn());

            ////////////////////Compute Problem Data at init /////////////////////////////
            const uint nv = robot_->nv();
            tsid_->computeProblemData(dt_, q_tsid_, Vector::Zero(nv));

            assert(tsid_);
            assert(robot_);
            ////////////////////Compute Tasks, Bounds and Contacts ///////////////////////
            
            cartesian_ee = make_se3_joint_task(
                "end_effector",
                cst::endEffector_joint_name,
                p.at("kp_ee"), //~~  change that kp
                se3_mask::all);
            if (p.at("w_ee") > 0) //~~  change that w
                tsid_->addMotionTask(*cartesian_ee, p.at("w_torso"), 1);
            se3_tasks_[cartesian_ee->name()] = cartesian_ee;

        }

        void TalosPosTracking::update(const SensorData& sensor_data)
        {
            auto com_ref = com_task_->getReference().pos;
              pinocchio::SE3 x = pinocchio::vectorToSE3( //~~ ag find that in the pinocchio sources to fond out how to build a SE3 type

            // estimate the CoP / ZMP
            bool cop_ok = _cop_estimator.update(com_ref.head(2),
                model_joint_pos("leg_left_6_joint").translation(),
                model_joint_pos("leg_right_6_joint").translation(),
                sensor_data.lf_torque, sensor_data.lf_force,
                sensor_data.rf_torque, sensor_data.rf_force);
            // modify the CoM reference (stabilizer) if the CoP is valid
            if (_use_stabilizer && cop_ok && !std::isnan(_cop_estimator.cop_filtered()(0)) && !std::isnan(_cop_estimator.cop_filtered()(1))) {
                // the expected zmp given CoM in x is x - z_c / g \ddot{x} (LIPM equations)
                // CoM = CoP+zc/g \ddot{x}
                // see Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point
                // see eq.24 of Biped Walking Stabilization Based on Linear Inverted Pendulum Tracking
                // see eq. 21 of Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control
                Eigen::Vector2d a = tsid_->data().acom[0].head<2>();
                Eigen::Vector3d com = tsid_->data().com[0];
                Eigen::Vector2d ref = com.head<2>() - com(2) / 9.81 * a; //com because this is the target
                auto cop = _cop_estimator.cop_filtered();
                Eigen::Vector2d cor = _stabilizer_p.array() * (ref.head(2) - _cop_estimator.cop_filtered()).array();

                // [not classic] we correct by the velocity of the CoM instead of the CoP because we have an IMU for this
                Eigen::Vector2d cor_v = _stabilizer_d.array() * sensor_data.velocity.head(2).array();
                cor += cor_v;

                Eigen::VectorXd ref_m = com_ref - Eigen::Vector3d(cor(0), cor(1), 0);
                set_com_ref(ref_m);
            }

            // solve everything
            _solve();

            // set the CoM back (useful if the behavior does not the set the ref at each timestep)
            set_com_ref(com_ref);
        }

        pinocchio::SE3 FrankaCartPosTracking::get_se3_ref(const std::string& task_name)
        {
            auto it = se3_tasks_.find(task_name);
            IWBC_ASSERT(it != se3_tasks_.end(), " task ", task_name, " not found");
            pinocchio::SE3 se3;
            auto pos = it->second->getReference().pos;
            tsid::math::vectorToSE3(pos, se3);
            return se3;
        }

        void FrankaCartPosTracking::set_se3_ref(const pinocchio::SE3& ref, const std::string& task_name)
        {
            auto it = se3_tasks_.find(task_name);
            IWBC_ASSERT(it != se3_tasks_.end(), " task ", task_name, " not found");
            auto sample = to_sample(ref);
            it->second->setReference(sample);
        }

        void FrankaCartPosTracking::set_com_ref(const Vector3& ref)
        {
            com_task_->setReference(to_sample(ref));
        }

        void FrankaCartPosTracking::set_posture_ref(const tsid::math::Vector& ref)
        {
            posture_task_->setReference(to_sample(ref));
        }

        void FrankaCartPosTracking::remove_task(const std::string& task_name, double transition_duration)
        {
            bool res = tsid_->removeTask(task_name, transition_duration);
            IWBC_ASSERT(res, "Cannot remove an unknown task: ", task_name);
        }

        std::shared_ptr<tsid::tasks::TaskBase> FrankaCartPosTracking::task(const std::string& task_name)
        {
            auto it = se3_tasks_.find(task_name);
            if (it != se3_tasks_.end())
                return se3_tasks_[task_name];
            else
                throw IWBC_EXCEPTION("Unknown task:", task_name);
        }

    } // namespace controllers
} // namespace inria_wbc
