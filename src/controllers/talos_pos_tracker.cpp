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

#include <boost/filesystem.hpp>
#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/utils/stop-watch.hpp>

#include "inria_wbc/controllers/talos_pos_tracker.hpp"
#include "inria_wbc/controllers/tasks.hpp"
#include "inria_wbc/utils/utils.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {
        static Register<TalosPosTracker> __talos_pos_tracking("talos-pos-tracker");

        TalosPosTracker::TalosPosTracker(const Params& params) : PosTracker(params)
        {
            parse_configuration_yaml(params.sot_config_path);
            if (verbose_)
                std::cout << "Talos pos tracker initialized" << std::endl;
        }

        void TalosPosTracker::parse_configuration_yaml(const std::string& sot_config_path)
        {
            // init stabilizer
            {
                YAML::Node c = IWBC_CHECK(YAML::LoadFile(sot_config_path)["CONTROLLER"]["stabilizer"]);
                _use_stabilizer = IWBC_CHECK(c["activated"].as<bool>());
                _torso_max_roll = IWBC_CHECK(c["torso_max_roll"].as<double>());

                _stabilizer_p.resize(6);
                _stabilizer_d.resize(6);
                _stabilizer_p_ankle.resize(6);
                _stabilizer_p_ffda.resize(3);

                _stabilizer_p.setZero();
                _stabilizer_d.setZero();
                _stabilizer_p_ankle.setZero();
                _stabilizer_p_ffda.setZero();

                IWBC_ASSERT(IWBC_CHECK(c["p"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in p for the stabilizer");
                IWBC_ASSERT(IWBC_CHECK(c["d"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in d for the stabilizer");
                IWBC_ASSERT(IWBC_CHECK(c["p_ankle"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in p_ankle for the stabilizer");
                IWBC_ASSERT(IWBC_CHECK(c["p_ffda"].as<std::vector<double>>()).size() == 3, "you need 3 coefficient in p_ffda for the stabilizer");

                _stabilizer_p = Eigen::VectorXd::Map(IWBC_CHECK(c["p"].as<std::vector<double>>()).data(), _stabilizer_p.size());
                _stabilizer_d = Eigen::VectorXd::Map(IWBC_CHECK(c["d"].as<std::vector<double>>()).data(), _stabilizer_d.size());
                _stabilizer_p_ankle = Eigen::VectorXd::Map(IWBC_CHECK(c["p_ankle"].as<std::vector<double>>()).data(), _stabilizer_p_ankle.size());
                _stabilizer_p_ffda = Eigen::VectorXd::Map(IWBC_CHECK(c["p_ffda"].as<std::vector<double>>()).data(), _stabilizer_p_ffda.size());

                auto history = c["filter_size"].as<int>();
                _cop_estimator.set_history_size(history);
            }

            // init collision detection
            {
                YAML::Node c = IWBC_CHECK(YAML::LoadFile(sot_config_path)["CONTROLLER"]["collision_detection"]);
                _use_torque_collision_detection = IWBC_CHECK(c["activated"].as<bool>());
                auto filter_window_size = IWBC_CHECK(c["filter_size"].as<int>());
                auto max_invalid = IWBC_CHECK(c["max_invalid"].as<int>());

                _torque_collision_joints = {
                    "leg_left_1_joint", "leg_left_2_joint", "leg_left_3_joint", "leg_left_4_joint", "leg_left_5_joint", "leg_left_6_joint",
                    "leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint",
                    "torso_1_joint", "torso_2_joint",
                    "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint",
                    "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint"};

                auto filtered_dof_names = this->all_dofs(true); // filter out mimics
                for (const auto& joint : _torque_collision_joints) {
                    auto it = std::find(filtered_dof_names.begin(), filtered_dof_names.end(), joint);
                    _torque_collision_joints_ids.push_back(std::distance(filtered_dof_names.begin(), it));
                }

                _torque_collision_threshold.resize(_torque_collision_joints.size());
                _torque_collision_threshold << 3.5e+05, 3.9e+05, 2.9e+05, 4.4e+05, 5.7e+05, 2.4e+05,
                    3.5e+05, 3.9e+05, 2.9e+05, 4.4e+05, 5.7e+05, 2.4e+05,
                    1e+01, 1e+01,
                    1e+01, 1e+01, 1e+01, 1e+01,
                    1e+01, 1e+01, 1e+01, 1e+01;

                // update thresholds from file (if any)
                if (c["thresholds"]) {
                    auto path = boost::filesystem::path(sot_config_path).parent_path();
                    auto p_thresh = IWBC_CHECK(path / boost::filesystem::path(c["thresholds"].as<std::string>()));
                    parse_collision_thresholds(p_thresh.string());
                }

                _torque_collision_filter = std::make_shared<estimators::MovingAverageFilter>(_torque_collision_joints.size(), filter_window_size);

                _torque_collision_detection = safety::TorqueCollisionDetection(_torque_collision_threshold);
                _torque_collision_detection.set_max_consecutive_invalid(max_invalid);
                _torque_collision_detection.set_filter(_torque_collision_filter);
            }

            if (verbose_) {
                std::cout << "Stabilizer:" << _use_stabilizer << std::endl;
                std::cout << "P:" << _stabilizer_p.transpose() << std::endl;
                std::cout << "D:" << _stabilizer_d.transpose() << std::endl;
                std::cout << "P ANKLE:" << _stabilizer_p_ankle.transpose() << std::endl;
                std::cout << "P FFDA:" << _stabilizer_p_ffda.transpose() << std::endl;

                std::cout << "Collision detection:" << _use_torque_collision_detection << std::endl;
                std::cout << "with thresholds" << std::endl;
                for (size_t id = 0; id < _torque_collision_joints.size(); ++id)
                    std::cout << _torque_collision_joints[id] << ": " << _torque_collision_threshold(id) << std::endl;
            }
        }

        void TalosPosTracker::parse_collision_thresholds(const std::string& config_path)
        {
            YAML::Node config = IWBC_CHECK(YAML::LoadFile(config_path));
            for (size_t jid = 0; jid < _torque_collision_joints.size(); ++jid) {
                std::string joint = _torque_collision_joints[jid];
                if (config[joint])
                    _torque_collision_threshold(jid) = IWBC_CHECK(config[joint].as<double>());
            }

            return;
        }

        void TalosPosTracker::update(const SensorData& sensor_data)
        {

            auto ac = activated_contacts_;
            for (auto& contact_name : ac) {
                auto pos = contact(contact_name)->getMotionTask().getReference().pos;
                pinocchio::SE3 se3;
                tsid::math::vectorToSE3(pos, se3);
                _contact_ref[contact_name] = se3;
            }
            auto com_ref = com_task()->getReference();
            auto left_ankle_ref = get_se3_ref("lf");
            auto right_ankle_ref = get_se3_ref("rf");
            auto torso_ref = get_se3_ref("torso");

            if (_use_stabilizer) {
                IWBC_ASSERT(sensor_data.find("lf_torque") != sensor_data.end(), "the stabilizer needs the LF torque");
                IWBC_ASSERT(sensor_data.find("rf_torque") != sensor_data.end(), "the stabilizer needs the RF torque");
                IWBC_ASSERT(sensor_data.find("velocity") != sensor_data.end(), "the stabilizer needs the velocity");

                // estimate the CoP / ZMP
                bool cop_ok = _cop_estimator.update(com_ref.pos.head(2),
                    model_joint_pos("leg_left_6_joint").translation(),
                    model_joint_pos("leg_right_6_joint").translation(),
                    sensor_data.at("lf_torque"), sensor_data.at("lf_force"),
                    sensor_data.at("rf_torque"), sensor_data.at("rf_force"));

                tsid::trajectories::TrajectorySample se3_sample, contact_sample, com_sample, torso_sample;
                // modify the CoM reference (stabilizer) if the CoP is valid
                if (cop_ok
                    && !std::isnan(_cop_estimator.cop_filtered()(0))
                    && !std::isnan(_cop_estimator.cop_filtered()(1))) {

                    com_admittance(_stabilizer_p, _stabilizer_d, sensor_data.at("velocity"), _cop_estimator.cop_filtered(), com_ref, tsid_->data(), com_sample);
                    set_com_ref(com_sample);
                }

                if (cop_ok
                    && !std::isnan(_cop_estimator.lcop_filtered()(0))
                    && !std::isnan(_cop_estimator.lcop_filtered()(1))
                    && std::find(ac.begin(), ac.end(), "contact_lfoot") != ac.end()) {

                    ankle_admittance(dt_, "l", _cop_estimator.lcop_filtered(), _stabilizer_p_ankle, left_ankle_ref, _contact_ref, contact_sample, se3_sample);
                    set_se3_ref(se3_sample, "lf");
                    contact("contact_lfoot")->setReference(contact_sample);
                }

                if (cop_ok
                    && !std::isnan(_cop_estimator.rcop_filtered()(0))
                    && !std::isnan(_cop_estimator.rcop_filtered()(1))
                    && std::find(ac.begin(), ac.end(), "contact_rfoot") != ac.end()) {

                    ankle_admittance(dt_, "r", _cop_estimator.rcop_filtered(), _stabilizer_p_ankle, right_ankle_ref, _contact_ref, contact_sample, se3_sample);
                    set_se3_ref(se3_sample, "rf");
                    contact("contact_rfoot")->setReference(contact_sample);
                }

                if (activated_contacts_forces_.find("contact_rfoot") != activated_contacts_forces_.end()
                    && activated_contacts_forces_.find("contact_lfoot") != activated_contacts_forces_.end()) {

                    //normal force of the contacts from tsid
                    double lf_normal_force = contact("contact_lfoot")->Contact6d::getNormalForce(activated_contacts_forces_["contact_lfoot"]);
                    double rf_normal_force = contact("contact_rfoot")->Contact6d::getNormalForce(activated_contacts_forces_["contact_rfoot"]);

                    foot_force_difference_admittance(dt_, _torso_max_roll, _stabilizer_p_ffda, torso_ref, lf_normal_force, rf_normal_force, sensor_data.at("lf_force"), sensor_data.at("rf_force"), torso_sample, activated_contacts_forces_);
                    set_se3_ref(torso_sample, "torso");
                }
            }

            if (_use_torque_collision_detection) {
                IWBC_ASSERT(sensor_data.find("joints_torque") != sensor_data.end(), "torque collision detection requires torque sensor data");
                IWBC_ASSERT(sensor_data.at("joints_torque").size() == _torque_collision_joints.size(), "torque sensor data has a wrong size. call torque_sensor_joints() for needed values");

                auto tsid_tau = utils::slice_vec(this->tau(), _torque_collision_joints_ids);
                _collision_detected = (false == _torque_collision_detection.check(tsid_tau, sensor_data.at("joints_torque")));
            }

            // solve everything
            _solve();

            // set the CoM back (useful if the behavior does not the set the ref at each timestep)
            set_com_ref(com_ref);
            set_se3_ref(left_ankle_ref, "lf");
            set_se3_ref(right_ankle_ref, "rf");
            set_se3_ref(torso_ref, "torso");
            for (auto& contact_name : ac) {
                contact(contact_name)->Contact6d::setReference(_contact_ref[contact_name]);
            }
        }

        void TalosPosTracker::com_admittance(
            const Eigen::VectorXd& p,
            const Eigen::VectorXd& d,
            const Eigen::MatrixXd& velocity,
            const Eigen::Vector2d& cop_filtered,
            const tsid::trajectories::TrajectorySample& com_ref,
            tsid::InverseDynamicsFormulationAccForce::Data data,
            tsid::trajectories::TrajectorySample& se3_sample)
        {
            // the expected zmp given CoM in x is x - z_c / g \ddot{x} (LIPM equations)
            // CoM = CoP+zc/g \ddot{x}
            // see Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point
            // see eq.24 of Biped Walking Stabilization Based on Linear Inverted Pendulum Tracking
            // see eq. 21 of Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control
            Eigen::Vector2d a = data.acom[0].head<2>();
            Eigen::Vector3d com = data.com[0];
            Eigen::Vector2d ref = com.head<2>() - com(2) / 9.81 * a; //com because this is the target
            Eigen::Vector2d cor = ref.head(2) - cop_filtered;

            // [not classic] we correct by the velocity of the CoM instead of the CoP because we have an IMU for this
            Eigen::Vector2d cor_v = velocity.block<2, 1>(0, 0);

            Eigen::Vector2d error = p.block(0, 0, 1, 2).array() * cor.array() + d.block(0, 0, 1, 2).array() * cor_v.array();
            Eigen::VectorXd ref_m = com_ref.pos - Eigen::Vector3d(error(0), error(1), 0);

            error = p.block(2, 0, 1, 2).array() * cor.array() + d.block(2, 0, 1, 2).array() * cor_v.array();
            Eigen::VectorXd vref_m = com_ref.vel - (Eigen::Vector3d(error(0), error(1), 0) / dt_);

            error = p.block(4, 0, 1, 2).array() * cor.array() + d.block(4, 0, 1, 2).array() * cor_v.array();
            Eigen::VectorXd aref_m = com_ref.acc - (Eigen::Vector3d(error(0), error(1), 0) / (dt_ * dt_));

            se3_sample.pos = ref_m;
            se3_sample.vel = vref_m;
            se3_sample.acc = aref_m;
        }

        void TalosPosTracker::ankle_admittance(
            double dt,
            const std::string& foot,
            const Eigen::Vector2d& cop_foot,
            const Eigen::VectorXd& p,
            pinocchio::SE3 ankle_ref,
            std::map<std::string, pinocchio::SE3> contact_ref,
            tsid::trajectories::TrajectorySample& contact_sample,
            tsid::trajectories::TrajectorySample& se3_sample)
        {
            Eigen::Vector3d cop_ankle_ref = ankle_ref.translation();

            double pitch = -p[0] * (cop_foot(0) - cop_ankle_ref(0));
            double roll = +p[1] * (cop_foot(1) - cop_ankle_ref(1));

            auto euler = ankle_ref.rotation().eulerAngles(0, 1, 2);
            euler[0] += roll;
            euler[1] += pitch;
            auto q = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());

            Eigen::VectorXd vel_ref = Eigen::VectorXd::Zero(6);
            vel_ref(4) = p[2] * pitch / dt;
            vel_ref(3) = p[3] * roll / dt;

            Eigen::VectorXd acc_ref = Eigen::VectorXd::Zero(6);
            acc_ref(4) = p[4] * pitch / (dt * dt);
            acc_ref(3) = p[5] * roll / (dt * dt);

            ankle_ref.rotation() = q.toRotationMatrix();
            se3_sample = to_sample(ankle_ref);
            se3_sample.vel = vel_ref;
            se3_sample.acc = acc_ref;

            contact_ref["contact_" + foot + "foot"].rotation() = q.toRotationMatrix();
            contact_sample = to_sample(contact_ref["contact_" + foot + "foot"]);
            contact_sample.vel = vel_ref;
            contact_sample.acc = acc_ref;
        }

        //FROM :
        //Biped Walking Stabilization Based on Linear Inverted Pendulum Tracking
        //by Shuuji Kajita, Mitsuharu Morisawa, Kanako Miura, Shin’ichiro Nakaoka,Kensuke Harada, Kenji Kaneko, Fumio Kanehiro and Kazuhito Yokoi
        //ALSO INSPIRED BY :
        //Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control
        //by Stéphane Caron, Abderrahmane Kheddar, Olivier Tempier
        //https://github.com/stephane-caron/lipm_walking_controller/blob/master/src/Stabilizer.cpp
        void TalosPosTracker::foot_force_difference_admittance(
            double dt,
            float torso_max_roll,
            Eigen::VectorXd p_ffda,
            pinocchio::SE3 torso_ref,
            double lf_normal_force,
            double rf_normal_force,
            const Eigen::Vector3d& lf_sensor_force,
            const Eigen::Vector3d& rf_sensor_force,
            tsid::trajectories::TrajectorySample& torso_sample,
            std::unordered_map<std::string, tsid::math::Vector> ac_forces)
        {

            double zctrl = (lf_normal_force - rf_normal_force) - (lf_sensor_force(2) - rf_sensor_force(2));

            auto torso_roll = -p_ffda[0] * zctrl;
            if (torso_roll > torso_max_roll)
                torso_roll = torso_max_roll;
            if (torso_roll < -torso_max_roll)
                torso_roll = -torso_max_roll;

            auto euler = torso_ref.rotation().eulerAngles(0, 1, 2);
            euler[0] += torso_roll;

            if (euler[0] >= torso_max_roll && euler[0] <= M_PI / 2) {
                euler[0] = torso_max_roll;
            }
            if (euler[0] >= M_PI / 2 && euler[0] <= M_PI - torso_max_roll) {
                euler[0] = M_PI - torso_max_roll;
            }

            auto q = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());

            Eigen::VectorXd vel_ref = Eigen::VectorXd::Zero(6);
            if (p_ffda[1] > p_ffda[0]) // to be sure not to go above torso_max_roll
                p_ffda[1] = p_ffda[0];

            vel_ref(3) = p_ffda[1] * torso_roll / dt;

            if (p_ffda[2] > p_ffda[1]) // to be sure not to go above torso_max_roll
                p_ffda[2] = p_ffda[1];

            Eigen::VectorXd acc_ref = Eigen::VectorXd::Zero(6);
            acc_ref(3) = p_ffda[2] * torso_roll / (dt * dt);

            torso_ref.rotation() = q.toRotationMatrix();
            torso_sample = to_sample(torso_ref);
            torso_sample.vel = vel_ref;
            torso_sample.acc = acc_ref;

            //If you want to use the anke height strategy (tested but seems less efficient)
            // lf_ankle_ref.translation()(2) += p_ffda[0] * 0.5 * zctrl;
            // rf_ankle_ref.translation()(2) += -p_ffda[0] * 0.5 * zctrl;
            // set_se3_ref(lf_ankle_ref, "lf");
            // set_se3_ref(rf_ankle_ref, "rf");
            // contact("contact_lfoot")->setReference(to_sample(lf_ankle_ref));
            // contact("contact_rfoot")->setReference(to_sample(rf_ankle_ref));
        }

        void TalosPosTracker::clear_collision_detection()
        {
            _torque_collision_detection.reset();
            _torque_collision_filter->reset();
            _collision_detected = false;
        }

    } // namespace controllers
} // namespace inria_wbc
