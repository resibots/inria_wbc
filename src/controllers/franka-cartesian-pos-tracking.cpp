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

        void FrankaCartPosTracking::parse_configuration_yaml(const std::string& sot_config_path)
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

        }

        void FrankaCartPosTracking::set_stack_configuration()
        {

            const opt_params_t& p = params_.opt_params;
            if (verbose_)
                for (auto& x : p)
                    std::cout << x.first << " => " << params_.opt_params[x.first] << std::endl;

            ////////////////////Gather Initial Pose //////////////////////////////////////
            q_tsid_ = robot_->model().referenceConfigurations[ref_config_];
            q0_=q_tsid_;

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
            
            //cartesian_ee_ = make_se3_joint_task(
            //    "end_effector",
            //    cst::endEffector_joint_name,
            //    p.at("kp_ee"), 
            //    se3_mask::all);
            //if (p.at("w_ee") > 0) 
            //    tsid_->addMotionTask(*cartesian_ee_, p.at("w_ee"), 1);
            //se3_tasks_[cartesian_ee_->name()] = cartesian_ee_;
            
            posture_task_ = make_posture_task("posture", p.at("kp_ee"));
            if (p.at("w_ee") > 0)
                tsid_->addMotionTask(*posture_task_, p.at("w_ee"), 1);

        }

        void FrankaCartPosTracking::update(const SensorData& sensor_data)
        {
            //Eigen::Vector3d ref_xyz;
            //ref_xyz << 2.8,2.8,2.8;
            //pinocchio::SE3 ref_ee;
            //ref_ee.translation( ref_xyz );
            //ref_ee.rotation(Eigen::Matrix<double,3,3>::Identity()); 
            //set_se3_ref( ref_ee, "end_effector");

            //~~ DEBUG BEGIN: gives expected numbers
            //pinocchio::SE3 ref_given  = get_se3_ref( "end_effector");
            //ref_given.disp_impl(std::cout);
            //~~ DEBUG END
    
            Eigen::VectorXd ref_posture(9);
            ref_posture << 0., M_PI / 4., 0., -M_PI / 4, 0., M_PI / 2., 0., 0., 0.;
            set_posture_ref( ref_posture);



            pinocchio::SE3 ee_pose = robot_->position(tsid_->data(), robot_->model().getJointId(cst::endEffector_joint_name));
            Eigen::Vector3d ee_xyz =  ee_pose.rotation() * ee_pose.translation();

            std::cout<<"____________________________________"<< cst::endEffector_joint_name<<" position :"<< ee_xyz<< std::endl;
            _solve();
        }

        void FrankaCartPosTracking::set_posture_ref(const tsid::math::Vector& ref)
        {
            posture_task_->setReference(to_sample(ref));
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
