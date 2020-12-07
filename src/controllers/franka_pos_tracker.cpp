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

#include "inria_wbc/controllers/franka_pos_tracker.hpp"
#include "inria_wbc/controllers/tasks.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {
        static Register<FrankaPosTracker> __franka_pos_tracker("pos-tracker");

        FrankaPosTracker::FrankaPosTracker(const Params& params) : Controller(params)
        {
            if (params.sot_config_path.empty()) {
                throw IWBC_EXCEPTION("empty configuration path! (we expect a YAML file)");
            }

            if (verbose_)
                std::cout << "loading main YAML file:" << params.sot_config_path << std::endl;

            // all the file paths are relative to the main config file
            auto path = boost::filesystem::path(params.sot_config_path).parent_path();

            YAML::Node config = YAML::LoadFile(params.sot_config_path)["CONTROLLER"];

            ////////////////////Gather Initial Pose //////////////////////////////////////
            //the srdf contains initial joint positions
            auto srdf_file = config["configurations"].as<std::string>();
            auto ref_config = config["ref_config"].as<std::string>();
            auto p_srdf = path / boost::filesystem::path(srdf_file);
            pinocchio::srdf::loadReferenceConfigurations(robot_->model(), p_srdf.string(), verbose_);

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

            auto task_file = config["tasks"].as<std::string>();
            auto p = path / boost::filesystem::path(task_file);
            parse_tasks(p.string());

            if (verbose_)
                std::cout << "Franka pos tracker initialized" << std::endl;
        }

        void FrankaPosTracker::update(const SensorData& sensor_data)
        {

            Eigen::Vector3d ref_xyz;
            ref_xyz << 0.6,0.0,0.6;
            pinocchio::SE3 ref_ee;
            ref_ee.translation( ref_xyz );
            Eigen::Matrix3d rot_ee;
            const double phi =  -M_PI/6;
            const double theta =  M_PI/6+ M_PI/2;
            const double psi =  0.;

            Eigen::Matrix3d R_x;
            R_x << 1., 0., 0.,
                   0., cos(phi), -sin(phi),
                   0., sin(phi), cos(phi);
            Eigen::Matrix3d R_y;
            R_y << cos(theta), 0., sin(theta),
                   0., 1., 0.,
                   -sin(theta), 0., cos(theta);
            Eigen::Matrix3d R_z;
            R_z << cos(psi), -sin(psi), 0.,
                   sin(psi), cos(psi), 0.,
                   0., 0., 1.;
            rot_ee = R_z * R_y * R_x;
            ref_ee.rotation(rot_ee); 
            set_se3_ref( ref_ee, "end_effector");

            //~~ DEBUG BEGIN: gives expected numbers
            //pinocchio::SE3 ref_given  = get_se3_ref( "end_effector");
            //ref_given.disp_impl(std::cout);
            //~~ DEBUG END
    
            //Eigen::VectorXd ref_posture(9);
            //ref_posture << 0., M_PI / 4., 0., -M_PI / 4, 0., M_PI / 2., 0., 0., 0.;
            //set_posture_ref( ref_posture);
            
            //~~ here, find out how to get the posture task form the tasks container, check the headers of this class ::se3_task(
            set_posture_ref( robot_->model().referenceConfigurations[ref_config_]) ;

            _solve();

        }


        void FrankaPosTracker::parse_tasks(const std::string& path)
        {
            if (verbose_)
                std::cout << "parsing task file:" << path << std::endl;
            YAML::Node task_list = YAML::LoadFile(path);
            for (auto it = task_list.begin(); it != task_list.end(); ++it) {
                auto name = it->first.as<std::string>();
                auto type = it->second["type"].as<std::string>();

                // the task is added automatically to TSID by the factory
                auto task = tasks::FactoryYAML::instance().create(type, robot_, tsid_, name, it->second);
                tasks_[name] = task;

                if (verbose_)
                    std::cout << "added task:" << name << " type:" << type << std::endl;
            }
        }

        pinocchio::SE3 FrankaPosTracker::get_se3_ref(const std::string& task_name)
        {
            auto task = se3_task(task_name);
            pinocchio::SE3 se3;
            auto pos = task->getReference().pos;
            tsid::math::vectorToSE3(pos, se3);
            return se3;
        }

        void FrankaPosTracker::set_se3_ref(const pinocchio::SE3& ref, const std::string& task_name)
        {
            auto task = se3_task(task_name);
            auto sample = to_sample(ref);
            task->setReference(sample);
        }

        void FrankaPosTracker::remove_task(const std::string& task_name, double transition_duration)
        {
            if (verbose_)
                std::cout << "removing task:" << task_name << std::endl;
            IWBC_ASSERT(tasks_.find(task_name) != tasks_.end(), "Trying to remove an unknown task:", task_name);
            bool res = tsid_->removeTask(task_name, transition_duration);
            IWBC_ASSERT(res, "Cannot remove an unknown task: ", task_name);
        }

        size_t FrankaPosTracker::num_task_weights() const
        {
            // we count all the tasks with a priority >= 1 (since it does not matter for 0)
            size_t k = 0;
            for (auto& x : tsid_->m_taskMotions)
                if (x->priority > 0)
                    ++k;
            return k;
        }
        void FrankaPosTracker::update_task_weights(const std::vector<double>& new_weights)
        {
            int i = 0;
            for (auto& x : tsid_->m_taskMotions) {
                if (x->priority > 0) {
                    IWBC_ASSERT(i < new_weights.size(), i, " vs ", new_weights.size());
                    tsid_->updateTaskWeight(x->task.name(), new_weights[i]);
                    ++i;
                }
            }
        }
    } // namespace controllers
} // namespace inria_wbc
