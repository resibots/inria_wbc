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
        static Register<FrankaPosTracker> __franka_pos_tracker("franka-pos-tracker");

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
            m_ref_config = config["ref_config"].as<std::string>();
            auto p_srdf = path / boost::filesystem::path(srdf_file);
            pinocchio::srdf::loadReferenceConfigurations(robot_->model(), p_srdf.string(), verbose_);

            ////////////////////Gather Initial Pose //////////////////////////////////////
            q_tsid_ = robot_->model().referenceConfigurations[m_ref_config];
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

        void FrankaPosTracker::set_task_ref(const tsid::math::Vector& ref, const std::string& task_name)
        {
            auto curr_task = task<tsid::tasks::TaskJointPosture>( task_name );
            auto sample = to_sample(ref);
            curr_task->setReference(sample);
        }

        void FrankaPosTracker::set_task_ref(const pinocchio::SE3& ref, const std::string& task_name)
        {
            auto curr_task = task<tsid::tasks::TaskSE3Equality>( task_name );
            auto sample = to_sample(ref);
            curr_task->setReference(sample);
        }

        void FrankaPosTracker::get_task_ref(const std::string& task_name, tsid::math::Vector& vec)
        {
            auto curr_task = task<tsid::tasks::TaskJointPosture>( task_name );
            vec  = curr_task->getReference().pos;
            //vec  = curr_task->getReference(); //~~ i don't know the task API
        }

        void FrankaPosTracker::get_task_ref(const std::string& task_name, pinocchio::SE3& se3)
        {
            auto curr_task = task<tsid::tasks::TaskSE3Equality>( task_name );
            auto pos = curr_task->getReference().pos;
            tsid::math::vectorToSE3(pos, se3);
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
