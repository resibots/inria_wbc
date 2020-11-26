#include <Eigen/Core>
#include <iomanip>
#include <map>
#include <memory>
#include <utility>
#include <vector>

/* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/utils/stop-watch.hpp>

#include <boost/filesystem.hpp>

#include "inria_wbc/controllers/talos_pos_tracking.hpp"
#include "inria_wbc/controllers/tasks.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {
        static Register<TalosPosTracking> __talos_pos_tracking("talos-pos-tracking");

        TalosPosTracking::TalosPosTracking(const Params& params) : Controller(params)
        {
            if (params.sot_config_path.empty())
                throw IWBC_EXCEPTION("empty configuration path! (we expect a YAML file)");

            YAML::Node config = YAML::LoadFile(params.sot_config_path);
            utils::parse(ref_config_, "ref_config", config, "CONTROLLER", verbose_);

            ////////////////////Gather Initial Pose //////////////////////////////////////
            //q_tsid_ is of size 37 (pos+quat+nactuated)
            q_tsid_ = robot_->model().referenceConfigurations[ref_config_];
            //q0_ is in "Dart format" for the floating base
            Eigen::Quaterniond quat(q_tsid_(6), q_tsid_(3), q_tsid_(4), q_tsid_(5));
            Eigen::AngleAxisd aaxis(quat);
            q0_ << q_tsid_.head(3), aaxis.angle() * aaxis.axis(), q_tsid_.tail(robot_->na());

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

            auto task_file = config["CONTROLLER"]["tasks"].as<std::string>();
            auto p = boost::filesystem::path(params.sot_config_path).parent_path()
                / boost::filesystem::path(task_file);

            parse_tasks(p.string());
            parse_configuration_yaml(params.sot_config_path);

            set_stack_configuration();
        }

        void TalosPosTracking::set_default_opt_params(std::map<std::string, double>& p)
        {
            p["w_com"] = 10.0; //# weight of center of mass task
            p["w_posture"] = 0.75; //# weight of joint posture task
            p["w_forceRef_feet"] = 1e-3; //# weight of force regularization task
            p["w_forceRef_hands"] = 1e-3; //# weight of force regularization task
            p["w_floatingb"] = 20.0; //# weight of floatingb task
            p["w_velocity"] = 1.0; //# weight of velocity bounds
            p["w_rh"] = 10.0; //# weight of right hand  task
            p["w_lh"] = 10.0; //# weight of left hand  task
            p["w_rf"] = 1.0; //# weight of right foot  task
            p["w_lf"] = 1.0; //# weight of left foot  task
            p["w_torso"] = 1000.0; //# weight of torso task

            p["kp_contact"] = 30.0; //# proportional gain of contact constraint
            p["kp_com"] = 3000.0; //# proportional gain of center of mass task
            p["kp_posture"] = 30.0; //# proportional gain of joint posture task
            p["kp_floatingb"] = 3000.0; //# proportional gain of floatingb task
            p["kp_rh"] = 300.0; //# proportional gain of right hand task
            p["kp_lh"] = 300.0; //# proportional gain of left hand task
            p["kp_rf"] = 30.0; //# proportional gain of right foot task
            p["kp_lf"] = 30.0; //# proportional gain of left foot task
            p["kp_torso"] = 30.0; //# proportional gain of the torso task
        }

        void TalosPosTracking::parse_tasks(const std::string& path)
        {
            std::cout << "parsing:" << path << std::endl;
            YAML::Node task_list = YAML::LoadFile(path);
            for (auto it = task_list.begin(); it != task_list.end(); ++it) {
                auto name = it->first.as<std::string>();
                auto type = it->second["type"].as<std::string>();
                if (type == "contact") {
                    // the task is added to tsid by make_contact
                    auto task = tasks::make_contact_task(robot_, tsid_, name, it->second);
                    contacts_[name] = task;
                }
                else {
                    // the task is added automatically to TSID by the factory
                    auto task = tasks::FactoryYAML::instance().create(type, robot_, tsid_, name, it->second);
                    tasks_[name] = task;
                }
            }
        }
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

        std::shared_ptr<contacts::Contact6d> TalosPosTracking::make_contact_task(const std::string& name, const std::string frame_name, double kp) const
        {
            assert(tsid_);
            assert(robot_);
            Matrix3x contact_points(3, 4);
            contact_points << -cst::lxn, -cst::lxn, cst::lxp, cst::lxp,
                -cst::lyn, cst::lyp, -cst::lyn, cst::lyp,
                cst::lz, cst::lz, cst::lz, cst::lz;
            auto contact_task = std::make_shared<contacts::Contact6d>(name, *robot_, frame_name, contact_points, cst::contact_normal, cst::mu, cst::fMin, cst::fMax);
            contact_task->Kp(kp * Vector::Ones(6));
            contact_task->Kd(2.0 * contact_task->Kp().cwiseSqrt());
            auto contact_ref = robot_->position(tsid_->data(), robot_->model().getJointId(frame_name));
            contact_task->setReference(contact_ref);
            return contact_task;
        }

        void TalosPosTracking::set_stack_configuration()
        {

            const opt_params_t& p = params_.opt_params;
            if (verbose_)
                for (auto& x : p)
                    std::cout << x.first << " => " << params_.opt_params[x.first] << std::endl;

            ////////////////////Compute Tasks, Bounds and Contacts ///////////////////////
            // contactRF_ = make_contact_task("contact_rfoot", cst::rf_joint_name, p.at("kp_contact"));
            // if (p.at("w_forceRef_feet") > 0)
            //     tsid_->addRigidContact(*contactRF_, p.at("w_forceRef_feet"));

            // contactLF_ = make_contact_task("contact_lfoot", cst::lf_joint_name, p.at("kp_contact"));
            // if (p.at("w_forceRef_feet") > 0)
            //     tsid_->addRigidContact(*contactLF_, p.at("w_forceRef_feet"));

            // com_task_ = make_com_task("com", p.at("kp_com"));
            // if (p.at("w_com") > 0)
            //     tsid_->addMotionTask(*com_task_, p.at("w_com"), 1);

            // posture_task_ = make_posture_task("posture", p.at("kp_posture"));
            // if (p.at("w_posture") > 0)
            //     tsid_->addMotionTask(*posture_task_, p.at("w_posture"), 1);

            // bounds_task_ = make_bound_task("task-posVelAcc-bounds");
            // if (p.at("w_velocity") > 0)
            //     tsid_->addMotionTask(*bounds_task_, p.at("w_velocity"), 0);

            // auto vert_torso_task = make_se3_frame_task("torso", cst::torso_frame_name, p.at("kp_torso"), se3_mask::roll + se3_mask::pitch);
            // if (p.at("w_torso") > 0)
            //     tsid_->addMotionTask(*vert_torso_task, p.at("w_torso"), 1);
            // se3_tasks_[vert_torso_task->name()] = vert_torso_task;

            // auto floatingb_task = make_se3_joint_task("floatingb", fb_joint_name_, p.at("kp_floatingb"), inria_wbc::se3_mask::rpy);
            // if (p.at("w_floatingb") > 0)
            //     tsid_->addMotionTask(*floatingb_task, p.at("w_floatingb"), 1);
            // se3_tasks_[floatingb_task->name()] = floatingb_task;

            // auto lh_task = make_se3_joint_task("lh", cst::lh_joint_name, p.at("kp_lh"), inria_wbc::se3_mask::xyz);
            // if (p.at("w_lh") > 0)
            //     tsid_->addMotionTask(*lh_task, p.at("w_lh"), 1);
            // se3_tasks_[lh_task->name()] = lh_task;

            // auto rh_task = make_se3_joint_task("rh", cst::rh_joint_name, p.at("kp_rh"), inria_wbc::se3_mask::xyz);
            // if (p.at("w_rh") > 0)
            //     tsid_->addMotionTask(*rh_task, p.at("w_rh"), 1);
            // se3_tasks_[rh_task->name()] = rh_task;

            // auto lf_task = make_se3_joint_task("lf", cst::lf_joint_name, p.at("kp_lf"), inria_wbc::se3_mask::all);
            // if (p.at("w_lf") > 0)
            //     tsid_->addMotionTask(*lf_task, p.at("w_lf"), 1);
            // se3_tasks_[lf_task->name()] = lf_task;

            // auto rf_task = make_se3_joint_task("rf", cst::rf_joint_name, p.at("kp_rf"), inria_wbc::se3_mask::all);
            // if (p.at("w_rf") > 0)
            //     tsid_->addMotionTask(*rf_task, p.at("w_rf"), 1);
            // se3_tasks_[rf_task->name()] = rf_task;
        }

        void TalosPosTracking::update(const SensorData& sensor_data)
        {
            auto com_ref = com_task()->getReference().pos;

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

        pinocchio::SE3 TalosPosTracking::get_se3_ref(const std::string& task_name)
        {
            auto task = se3_task(task_name);
            pinocchio::SE3 se3;
            auto pos = task->getReference().pos;
            tsid::math::vectorToSE3(pos, se3);
            return se3;
        }

        void TalosPosTracking::set_se3_ref(const pinocchio::SE3& ref, const std::string& task_name)
        {
            auto task = se3_task(task_name);
            auto sample = to_sample(ref);
            task->setReference(sample);
        }


        void TalosPosTracking::remove_contact(const std::string& contact_name)
        {
            bool res = tsid_->removeRigidContact(contact_name);
            IWBC_ASSERT(res, " contact ", contact_name, " not found");
        }

        void TalosPosTracking::add_contact(const std::string& contact_name)
        {
            auto c = contact(contact_name);
            tsid_->addRigidContact(*c, tasks::cst::w_force_feet);
            // const opt_params_t& p = params_.opt_params;
            // const pinocchio::Data& data = tsid_->data();
            // if (contact_name == "contact_rfoot") {
            //     auto contact_rf_ref = robot_->position(data, robot_->model().getJointId(cst::rf_joint_name));
            //     contactRF_->setReference(contact_rf_ref);
            //     tsid_->addRigidContact(*contactRF_, p.at("w_forceRef_feet"));
            // }
            // else if (contact_name == "contact_lfoot") {
            //     auto contact_lf_ref = robot_->position(data, robot_->model().getJointId(cst::lf_joint_name));
            //     contactLF_->setReference(contact_lf_ref);
            //     tsid_->addRigidContact(*contactLF_, p.at("w_forceRef_feet"));
            // }
            // else {
            //     throw IWBC_EXCEPTION("Cannot add an unknown contact: ", contact_name);
            // }
        }

        void TalosPosTracking::remove_task(const std::string& task_name, double transition_duration)
        {
            bool res = tsid_->removeTask(task_name, transition_duration);
            IWBC_ASSERT(res, "Cannot remove an unknown task: ", task_name);
        }

    } // namespace controllers
} // namespace inria_wbc
