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

#ifdef TSID_QPMAD_FOUND
#include <tsid/solvers/solver-HQP-qpmad.hpp>
#endif

#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/utils/stop-watch.hpp>

#include <boost/filesystem.hpp>

#include "inria_wbc/controllers/pos_tracker.hpp"
#include "inria_wbc/controllers/tasks.hpp"
#include "inria_wbc/trajs/utils.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {
        static Register<PosTracker> __generic_pos_tracker("pos-tracker");

        PosTracker::PosTracker(const YAML::Node& config) : Controller(config)
        {
            // we only care about the CONTROLLER section
            YAML::Node c = IWBC_CHECK(config["CONTROLLER"]);

            // qp solver to be used (eiquadprog or qpmad)
            solver_to_use_ = IWBC_CHECK(c["solver"].as<std::string>());

            // all the file paths are relative to base_path
            auto path = IWBC_CHECK(c["base_path"].as<std::string>());

            // create additional frames if needed (optional)
            if (c["frames"]) {
                auto p_frames = path + "/" + c["frames"].as<std::string>();
                parse_frames(p_frames);
            }

            ////////////////////Gather Initial Pose //////////////////////////////////////
            //the srdf contains initial joint positions
            auto srdf_file = IWBC_CHECK(c["configurations"].as<std::string>());
            auto ref_config = IWBC_CHECK(c["ref_config"].as<std::string>());
            auto p_srdf = path + "/" + srdf_file;
            pinocchio::srdf::loadReferenceConfigurations(robot_->model(), p_srdf, verbose_);

            //q_tsid_ for talos is of size 37 (pos+quat+nactuated)
            auto ref_map = robot_->model().referenceConfigurations;
            IWBC_ASSERT(ref_map.find(ref_config) != ref_map.end(), "The following reference config is not in ref_map : ", ref_config);
            q_tsid_ = ref_map[ref_config];
            if (verbose_)
                std::cout << "q_tsid ref:" << q_tsid_.transpose() << "[" << q_tsid_.size() << "]" << std::endl;

            if (floating_base_) {
                //q0_ is in "Dart format" for the floating base
                Eigen::Quaterniond quat(q_tsid_(6), q_tsid_(3), q_tsid_(4), q_tsid_(5));
                Eigen::AngleAxisd aaxis(quat);
                q0_ << q_tsid_.head(3), aaxis.angle() * aaxis.axis(), q_tsid_.tail(robot_->na());
                q_tsid_.segment(3, 4) << quat.normalized().coeffs();
            }
            else {
                q0_ = q_tsid_;
            }

            ////////////////////Create the inverse-dynamics formulation///////////////////
            tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);

            ////////////////////Create an HQP solver /////////////////////////////////////
            using solver_t = std::shared_ptr<solvers::SolverHQPBase>;

            if (solver_to_use_ == "qpmad") {
#ifdef TSID_QPMAD_FOUND
                solver_ = solver_t(solvers::SolverHQPFactory::createNewSolver(solvers::SOLVER_HQP_QPMAD, "solver-qpmad"));
#else
                IWBC_ERROR("'qpmad' solver is not available in tsid or in the system.");
#endif
            }
            else if (solver_to_use_ == "eiquadprog") {
                solver_ = solver_t(solvers::SolverHQPFactory::createNewSolver(solvers::SOLVER_HQP_EIQUADPROG_FAST, "solver-eiquadprog"));
            }
            else {
                IWBC_ERROR("solver in configuration file must be either 'eiquadprog' or 'qpmad'.");
            }

            solver_->resize(tsid_->nVar(), tsid_->nEq(), tsid_->nIn());

            ////////////////////Compute Problem Data at init /////////////////////////////
            const uint nv = robot_->nv();
            tsid_->computeProblemData(dt_, q_tsid_, Vector::Zero(nv));

            assert(tsid_);
            assert(robot_);

            auto task_file = IWBC_CHECK(c["tasks"].as<std::string>());
            auto p = path / boost::filesystem::path(task_file);
            parse_tasks(p.string(), config);

            ///////////// check if joint range of motion has to be reduced //////////////////////////
            if (c["joint_range_reduction"]) {
                double reduction_rads = IWBC_CHECK(c["joint_range_reduction"].as<double>()) / 180 * M_PI;
                if (reduction_rads < 0)
                    IWBC_ERROR("Joint range reduction: reduction must be positive.");

                auto q_lb = robot_->model().lowerPositionLimit.tail(robot_->na());
                auto q_ub = robot_->model().upperPositionLimit.tail(robot_->na());

                if (verbose_)
                    std::cout << "Joints' range of motion reduced by: " << reduction_rads << " rads." << std::endl;

                auto q = q0_.tail(robot_->na());

                for (size_t i = 0; i < robot_->na(); ++i) {
                    if (q_ub[i] - q_lb[i] < 2 * reduction_rads)
                        IWBC_ERROR("Joint range reduction: reduction cannot be greater than actual range of motion");

                    if (verbose_)
                        std::cout << "change bounds for " << pinocchio_joint_names()[i + 2]
                                  << " from (" << q_lb[i] << "," << q_ub[i] << ") ";

                    q_lb[i] += reduction_rads;
                    q_ub[i] -= reduction_rads;

                    if (verbose_)
                        std::cout << "to (" << q_lb[i] << "," << q_ub[i] << "). q=" << q[i] << std::endl;

                    if (q[i] < q_lb[i] || q[i] > q_ub[i])
                        IWBC_ERROR("Joint range reduction(", pinocchio_joint_names()[i + 2], "): q0 falls outside reduced joint space.");
                }

                bound_task()->setPositionBounds(q_lb, q_ub);
            }

            if (verbose_) {
                std::cout << "--------- Solver size info ---------" << std::endl;
                std::cout << "Solver : " << solver_to_use_ << std::endl;
                std::cout << "total number of variable (acceleration + contact-force) : " << tsid_->nVar() << std::endl;
                std::cout << "number of equality constraints : " << tsid_->nEq() << std::endl;
                std::cout << "number of inequality constraints : " << tsid_->nIn() << std::endl;
                std::cout << "--------- ------------- ---------" << std::endl;
                std::cout << "position tracker initializer" << std::endl;
            }
        }

        void PosTracker::parse_tasks(const std::string& path, const YAML::Node& config)
        {
            int task_count = 0;
            if (verbose_)
                std::cout << "parsing task file:" << path << std::endl;
            YAML::Node task_list = IWBC_CHECK(YAML::LoadFile(path));
            for (auto it = task_list.begin(); it != task_list.end(); ++it) {
                auto name = IWBC_CHECK(it->first.as<std::string>());
                auto type = IWBC_CHECK(it->second["type"].as<std::string>());
                if (type == "contact") {
                    // the task is added to tsid by make_contact
                    auto task = tasks::make_contact_task(robot_, tsid_, name, it->second, config);
                    contacts_[name] = task;
                    activated_contacts_.push_back(name);
                    all_contacts_.push_back(name);
                }
                else if (type.find("contact-") == std::string::npos) {
                    // the task is added automatically to TSID by the factory
                    auto task = tasks::FactoryYAML::instance().create(type, robot_, tsid_, name, it->second, config, {});
                    tasks_[name] = task;
                    activated_tasks_.push_back(name);
                }
                if (verbose_)
                    std::cout << "added task/contact:" << name << " type:" << type << std::endl;
                task_count++;
            }
            for (auto it = task_list.begin(); it != task_list.end(); ++it) {
                auto name = IWBC_CHECK(it->first.as<std::string>());
                auto type = IWBC_CHECK(it->second["type"].as<std::string>());
                if (type.find("contact-") != std::string::npos) {
                    auto task = tasks::FactoryYAML::instance().create(type, robot_, tsid_, name, it->second, config, contacts_);
                    tasks_[name] = task;
                    activated_tasks_.push_back(name);
                }
                if (verbose_)
                    std::cout << "added task/contact:" << name << " type:" << type << std::endl;
                task_count++;

                if (verbose_)
                    std::cout << "Number of parsed tasks " << task_count << std::endl;
            }
        }

        void PosTracker::parse_frames(const std::string& path)
        {
            if (verbose_)
                std::cout << "Parsing virtual frame file:" << path << std::endl;
            YAML::Node node = IWBC_CHECK(YAML::LoadFile(path));
            for (auto it = node.begin(); it != node.end(); ++it) {
                auto name = IWBC_CHECK(it->first.as<std::string>());
                auto ref = IWBC_CHECK(it->second["ref"].as<std::string>());
                auto pos = IWBC_CHECK(it->second["pos"].as<std::vector<double>>());

                pinocchio::SE3 p(1);
                p.translation() = pinocchio::SE3::LinearType(pos[0], pos[1], pos[2]);
                auto parent_frame_id = robot_->model().getFrameId(ref);
                auto& frame = robot_->model().frames[parent_frame_id];
                robot_->model().addFrame(pinocchio::Frame(name, frame.parent, parent_frame_id,
                    frame.placement * p, pinocchio::FIXED_JOINT));
                assert(robot_->model().existFrame(name));
            }
        }

        pinocchio::SE3 PosTracker::get_se3_ref(const std::string& task_name)
        {
            auto task = se3_task(task_name);
            pinocchio::SE3 se3;
            auto pos = task->getReference().getValue();
            tsid::math::vectorToSE3(pos, se3);
            return se3;
        }

        void PosTracker::set_se3_ref(const pinocchio::SE3& ref, const std::string& task_name)
        {
            auto task = se3_task(task_name);
            auto sample = trajs::to_sample(ref);
            task->setReference(sample);
        }

        void PosTracker::set_contact_se3_ref(const pinocchio::SE3& ref, const std::string& contact_name)
        {
            auto c = std::dynamic_pointer_cast<tsid::contacts::Contact6dExt>(contact(contact_name));
            auto sample = trajs::to_sample(ref);
            c->setReference(sample);
        }

        void PosTracker::set_se3_ref(tsid::trajectories::TrajectorySample& sample, const std::string& task_name)
        {
            auto task = se3_task(task_name);
            task->setReference(sample);
        }

        void PosTracker::set_contact_se3_ref(tsid::trajectories::TrajectorySample& sample, const std::string& contact_name)
        {
            auto c = std::dynamic_pointer_cast<tsid::contacts::Contact6dExt>(contact(contact_name));
            c->setReference(sample);
        }

        void PosTracker::remove_contact(const std::string& contact_name)
        {
            if (verbose_)
                std::cout << "removing contact:" << contact_name << std::endl;
            IWBC_ASSERT(contacts_.find(contact_name) != contacts_.end(), "Trying to remove an contact:", contact_name);
            bool res = tsid_->removeRigidContact(contact_name);
            IWBC_ASSERT(res, " contact ", contact_name, " not found");
            activated_contacts_.erase(std::remove(activated_contacts_.begin(), activated_contacts_.end(), contact_name), activated_contacts_.end());
        }

        void PosTracker::add_contact(const std::string& contact_name)
        {
            if (verbose_)
                std::cout << "adding contact:" << contact_name << std::endl;
            auto c = contact(contact_name);
            tsid_->addRigidContact(*c, tasks::cst::w_force_feet);
            activated_contacts_.push_back(contact_name);
        }
        //returns output = [fx,fy,fz,tau_x,tau_y,tau_z] from  tsid solution
        //when we supress foot mass it corresponds to F/T sensor data
        Eigen::VectorXd PosTracker::force_torque_from_solution(const std::string& contact_name, bool remove_foot_mass, const std::string& sole_frame)
        {
            Eigen::Matrix<double, 6, 1> force_tsid;
            force_tsid.setZero();

            IWBC_ASSERT(activated_contacts_forces_.find(contact_name) != activated_contacts_forces_.end(), contact_name, " not in activated_contacts_forces_");
            auto contact_force = activated_contacts_forces_[contact_name];
            auto generatorMatrix = std::dynamic_pointer_cast<tsid::contacts::Contact6dExt>(contact(contact_name))->Contact6d::getForceGeneratorMatrix();
            force_tsid = generatorMatrix * contact_force;

            if (remove_foot_mass) {
                // we retrieve the tracked frame from the contact task
                auto ankle_frame = robot_->model().frames[contact(contact_name)->getMotionTask().frame_id()].name;
                IWBC_ASSERT(robot_->model().existFrame(ankle_frame), "Frame " + ankle_frame + " does not exist in model.");
                IWBC_ASSERT(robot_->model().existFrame(sole_frame), "Frame " + sole_frame + " does not exist in model.");

                auto ankle_world = tsid_->data().oMi[robot_->model().getJointId(ankle_frame)];
                auto sole_world = tsid_->data().oMf[robot_->model().getFrameId(sole_frame)];

                auto foot_mass = robot_->model().inertias[robot_->model().getJointId(ankle_frame)].mass();
                // auto contact_normal = contact(contact_name)->getContactNormal();
                force_tsid.head(3) += foot_mass * robot_->model().gravity.linear();
                Eigen::Vector3d tmp = force_tsid.head(3);
                force_tsid.tail(3) -= (ankle_world.translation() - sole_world.translation()).cross(tmp);
            }
            return force_tsid;
        }

        void PosTracker::remove_task(const std::string& task_name, double transition_duration)
        {
            if (verbose_)
                std::cout << "removing task:" << task_name << std::endl;
            IWBC_ASSERT(tasks_.find(task_name) != tasks_.end(), "Trying to remove an unknown task:", task_name);
            bool res = tsid_->removeTask(task_name, transition_duration);
            IWBC_ASSERT(res, "Cannot remove an unknown task: ", task_name);
            activated_tasks_.erase(std::remove(activated_tasks_.begin(), activated_tasks_.end(), task_name), activated_tasks_.end());
        }

        size_t PosTracker::num_task_weights() const
        {
            // we count all the tasks with a priority >= 1 (since it does not matter for 0)
            size_t k = 0;
            for (auto& x : tsid_->m_taskMotions)
                if (x->priority > 0)
                    ++k;
            return k;
        }
        void PosTracker::update_task_weights(const std::vector<double>& new_weights)
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
