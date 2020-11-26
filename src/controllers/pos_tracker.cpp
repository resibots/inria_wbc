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

#include "inria_wbc/controllers/pos_tracker.hpp"
#include "inria_wbc/controllers/tasks.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {
        static Register<PosTracker> __generic_pos_tracker("pos-tracker");

        PosTracker::PosTracker(const Params& params) : Controller(params)
        {
            if (params.sot_config_path.empty())
                throw IWBC_EXCEPTION("empty configuration path! (we expect a YAML file)");

            if (verbose_)
                std::cout << "loading YAML file:" << params.sot_config_path << std::endl;
            ;

            YAML::Node config = YAML::LoadFile(params.sot_config_path)["CONTROLLER"];

            // all the file paths are relative to the main config file
            
            ////////////////////Gather Initial Pose //////////////////////////////////////
            //the srdf contains initial joint positions
            auto srdf_file = config["configurations"].as<std::string>();
            auto ref_config = config["ref_config"].as<std::string>();
            auto path = boost::filesystem::path(params.sot_config_path).parent_path();
            auto p_srdf = path / boost::filesystem::path(srdf_file);
            pinocchio::srdf::loadReferenceConfigurations(robot_->model(), p_srdf.string(), verbose_);

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

            auto task_file = config["tasks"].as<std::string>();
            auto p = path / boost::filesystem::path(task_file);
            parse_tasks(p.string());

            if (verbose_)
                std::cout << "position tracker initializer" << std::endl;
            ;
        }

        void PosTracker::parse_tasks(const std::string& path)
        {
            if (verbose_)
                std::cout << "parsing task file:" << path << std::endl;
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
                if (verbose_)
                    std::cout << "added task/contact:" << name << " type:" << type << std::endl;
            }
        }

        pinocchio::SE3 PosTracker::get_se3_ref(const std::string& task_name)
        {
            auto task = se3_task(task_name);
            pinocchio::SE3 se3;
            auto pos = task->getReference().pos;
            tsid::math::vectorToSE3(pos, se3);
            return se3;
        }

        void PosTracker::set_se3_ref(const pinocchio::SE3& ref, const std::string& task_name)
        {
            auto task = se3_task(task_name);
            auto sample = to_sample(ref);
            task->setReference(sample);
        }

        void PosTracker::remove_contact(const std::string& contact_name)
        {
            if (verbose_)
                std::cout << "removing contact:" << contact_name << std::endl;
            IWBC_ASSERT(contacts_.find(contact_name) != contacts_.end(), "Trying to remove an contact:", contact_name);
            bool res = tsid_->removeRigidContact(contact_name);
            IWBC_ASSERT(res, " contact ", contact_name, " not found");
        }

        void PosTracker::add_contact(const std::string& contact_name)
        {
            auto c = contact(contact_name);
            tsid_->addRigidContact(*c, tasks::cst::w_force_feet);
        }

        void PosTracker::remove_task(const std::string& task_name, double transition_duration)
        {
            if (verbose_)
                std::cout << "removing task:" << task_name << std::endl;
            IWBC_ASSERT(tasks_.find(task_name) != tasks_.end(), "Trying to remove an unknown task:", task_name);
            bool res = tsid_->removeTask(task_name, transition_duration);
            IWBC_ASSERT(res, "Cannot remove an unknown task: ", task_name);
        }

    } // namespace controllers
} // namespace inria_wbc
