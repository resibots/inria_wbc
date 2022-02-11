#include "inria_wbc/stabilizers/stabilizer.hpp"

#include "inria_wbc/exceptions.hpp"
#include <tsid/math/utils.hpp>

namespace inria_wbc {
    namespace stabilizer {

        Eigen::Vector2d com_to_zmp(const tsid::trajectories::TrajectorySample& com_ref)
        {
            // the expected zmp given CoM in x is x - z_c / g \ddot{x} (LIPM equations)
            // CoM = CoP+zc/g \ddot{x}
            // see Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point
            // see eq.24 of Biped Walking Stabilization Based on Linear Inverted Pendulum Tracking
            // see eq. 21 of Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control
            Eigen::Vector2d a = com_ref.getSecondDerivative().head<2>();
            Eigen::Vector3d com = com_ref.getValue();
            Eigen::Vector2d zmp = com.head<2>() - com(2) / 9.81 * a; //com because this is the target
            return zmp;
        }

        tsid::trajectories::TrajectorySample data_to_sample(const tsid::InverseDynamicsFormulationAccForce::Data& data)
        {
            tsid::trajectories::TrajectorySample com_sample;
            com_sample.setValue(data.com[0]);
            com_sample.setDerivative(data.vcom[0]);
            com_sample.setSecondDerivative(data.acom[0]);
            return com_sample;
        }

        void com_admittance(
            double dt,
            const Eigen::VectorXd& p,
            const Eigen::Vector2d& cop_filtered,
            const tsid::trajectories::TrajectorySample& model_current_com,
            const tsid::trajectories::TrajectorySample& com_ref,
            tsid::trajectories::TrajectorySample& se3_sample)
        {
            IWBC_ASSERT("you need 6 coefficient in p for com admittance", p.size() == 6);

            if (std::abs(cop_filtered(0)) >= 10 && std::abs(cop_filtered(1)) >= 10)
                IWBC_ERROR("com_admittance : something is wrong with input cop_filtered, check sensor measurment: ", std::abs(cop_filtered(0)), " ", std::abs(cop_filtered(1)));

            Eigen::Vector2d ref = com_to_zmp(model_current_com); //because this is the target
            Eigen::Vector2d cor = ref.head(2) - cop_filtered;

            Eigen::Vector2d error = p.segment(0, 2).array() * cor.array();
            Eigen::VectorXd ref_m = com_ref.getValue() - Eigen::Vector3d(error(0), error(1), 0);

            error = p.segment(2, 2).array() * cor.array();
            Eigen::VectorXd vref_m = com_ref.getDerivative() - (Eigen::Vector3d(error(0), error(1), 0) / dt);

            error = p.segment(4, 2).array() * cor.array();
            Eigen::VectorXd aref_m = com_ref.getSecondDerivative() - (Eigen::Vector3d(error(0), error(1), 0) / (dt * dt));

            se3_sample.setValue(ref_m);
            se3_sample.setDerivative(vref_m);
            se3_sample.setSecondDerivative(aref_m);
        }

        //computes momentum according to cop, problem is that the robot doesn't go back to initial position after perturbation
        //because momentum task is actually controlling velocity and acceleration and not moment position ?
        //angular momentum vel is found to compensate for cop disturbance and then torso position doesn't go back to normal
        void momentum_com_admittance(
            double dt,
            const Eigen::VectorXd& p,
            const Eigen::VectorXd& d,
            const Eigen::Vector2d& cop_filtered,
            const tsid::trajectories::TrajectorySample& model_current_com,
            const tsid::trajectories::TrajectorySample& momentum_ref,
            tsid::trajectories::TrajectorySample& momentum_sample,
            float max_ref)
        {
            IWBC_ASSERT("you need 6 coefficient in p for com momentum admittance", p.size() == 6);
            IWBC_ASSERT("you need 6 coefficient in d for com momentum admittance", d.size() == 6);

            if (std::abs(cop_filtered(0)) >= 10 && std::abs(cop_filtered(1)) >= 10)
                IWBC_ERROR("com_admittance : something is wrong with input cop_filtered, check sensor measurment: ", std::abs(cop_filtered(0)), " ", std::abs(cop_filtered(1)));

            Eigen::Vector2d ref = com_to_zmp(model_current_com); //because this is the target
            Eigen::Vector2d cor = ref.head(2) - cop_filtered;
            Eigen::VectorXd vec(6);

            vec << p(0) * cor(0), p(1) * cor(1), 0, p(2) * cor(1), p(3) * cor(0), 0;
            Eigen::VectorXd vref_m = momentum_ref.getValue() - vec;

            vec << d(0) * cor(0), d(1) * cor(1), 0, d(2) * cor(1), d(3) * cor(0), 0;
            Eigen::VectorXd aref_m = momentum_ref.getDerivative() - vec / dt;

            for (int i = 0; i < vref_m.size(); i++) {
                if (vref_m[i] > max_ref)
                    vref_m[i] = max_ref;
                if (aref_m[i] > max_ref)
                    aref_m[i] = max_ref;
                if (vref_m[i] < -max_ref)
                    vref_m[i] = -max_ref;
                if (aref_m[i] < -max_ref)
                    aref_m[i] = -max_ref;
            }

            momentum_sample = momentum_ref;
            momentum_sample.setDerivative(vref_m);
            momentum_sample.setSecondDerivative(aref_m);
        }
        
        //computes momentum based on imu angular velocity and model imu_link angular velocity 
        void momentum_imu_admittance(
            double dt,
            const Eigen::VectorXd& p,
            const Eigen::VectorXd& d,
            const Eigen::Vector3d& imu_angular_vel,
            const Eigen::Vector3d& model_angular_vel,
            const tsid::trajectories::TrajectorySample& momentum_ref,
            tsid::trajectories::TrajectorySample& momentum_sample,
            float max_ref)
        {
            IWBC_ASSERT("you need 6 coefficients in p for momentum admittance", p.size() == 6);
            IWBC_ASSERT("you need 6 coefficients in d for momentum admittance", d.size() == 6);

            Eigen::Vector3d error = (model_angular_vel - imu_angular_vel);

            Eigen::VectorXd vec(6);
            vec << error(0), error(1), error(2), error(1), error(0), error(2);
            Eigen::VectorXd vel_ref = momentum_ref.getDerivative().array() - p.array() * vec.array();
            Eigen::VectorXd acc_ref = momentum_ref.getSecondDerivative().array() - d.array() * vec.array();

            for (int i = 0; i < vel_ref.size(); i++) {
                if (vel_ref[i] > max_ref)
                    vel_ref[i] = max_ref;
                if (acc_ref[i] > max_ref)
                    acc_ref[i] = max_ref;
                if (vel_ref[i] < -max_ref)
                    vel_ref[i] = -max_ref;
                if (acc_ref[i] < -max_ref)
                    acc_ref[i] = -max_ref;
            }
            momentum_sample = momentum_ref;
            momentum_sample.setDerivative(vel_ref);
            momentum_sample.setSecondDerivative(acc_ref / dt);
        }

        void com_imu_admittance(
            double dt,
            const Eigen::VectorXd& p,
            const Eigen::MatrixXd& velocity,
            const tsid::trajectories::TrajectorySample& com_ref,
            tsid::trajectories::TrajectorySample& se3_sample)
        {
            IWBC_ASSERT("you need 6 coefficient in p for com admittance", p.size() == 6);

            // [not classic] we correct by the velocity of the CoM instead of the CoP because we have an IMU for this
            Eigen::Vector2d cor_v = velocity.block<2, 1>(0, 0);

            Eigen::Vector2d error = p.block(0, 0, 1, 2).array() * cor_v.array();
            Eigen::VectorXd ref_m = com_ref.getValue() - Eigen::Vector3d(error(0), error(1), 0);

            error = p.block(2, 0, 1, 2).array() * cor_v.array();
            Eigen::VectorXd vref_m = com_ref.getDerivative() - (Eigen::Vector3d(error(0), error(1), 0) / dt);

            error = p.block(4, 0, 1, 2).array() * cor_v.array();
            Eigen::VectorXd aref_m = com_ref.getSecondDerivative() - (Eigen::Vector3d(error(0), error(1), 0) / (dt * dt));

            se3_sample.setValue(ref_m);
            se3_sample.setDerivative(vref_m);
            se3_sample.setSecondDerivative(aref_m);
        }

        void ankle_admittance(
            double dt,
            const Eigen::VectorXd& p,
            const Eigen::Vector2d& cop_foot,
            const pinocchio::SE3& model_current_foot,
            const tsid::trajectories::TrajectorySample& se3_sample_ref,
            const tsid::trajectories::TrajectorySample& contact_sample_ref,
            tsid::trajectories::TrajectorySample& se3_sample,
            tsid::trajectories::TrajectorySample& contact_sample)
        {
            IWBC_ASSERT("you need 6 coefficient in p for ankle admittance", p.size() == 6);

            if (std::abs(cop_foot(0)) >= 10 && std::abs(cop_foot(1)) >= 10)
                IWBC_ERROR("ankle_admittance : something is wrong with input cop_foot, check sensor measurment: ", std::abs(cop_foot(0)), " ", std::abs(cop_foot(1)));

            pinocchio::SE3 ankle_ref;
            auto ankle_pos = se3_sample_ref.getValue();
            tsid::math::vectorToSE3(ankle_pos, ankle_ref);
            Eigen::Vector3d cop_ankle_ref = ankle_ref.translation();
            double pitch = +p[0] * (cop_foot(0) - model_current_foot.translation()(0));
            double roll = +p[1] * (cop_foot(1) - model_current_foot.translation()(1));

            auto euler = ankle_ref.rotation().eulerAngles(0, 1, 2);
            euler[0] += roll;
            euler[1] += pitch;
            auto q = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());

            // to be exact we should convert euler to x_dot and y_dot
            // because of the gains we don't need to be exact
            Eigen::VectorXd vel_ref = se3_sample_ref.getDerivative();
            vel_ref(4) += p[2] * pitch / dt;
            vel_ref(3) += p[3] * roll / dt;

            Eigen::VectorXd acc_ref = se3_sample_ref.getSecondDerivative();
            acc_ref(4) += p[4] * pitch / (dt * dt);
            acc_ref(3) += p[5] * roll / (dt * dt);

            ankle_ref.rotation() = q.toRotationMatrix();
            se3_sample.resize(12, 6);

            Eigen::VectorXd ankle_ref_vec(12);
            tsid::math::SE3ToVector(ankle_ref, ankle_ref_vec);
            se3_sample.setValue(ankle_ref_vec);
            se3_sample.setDerivative(vel_ref);
            se3_sample.setSecondDerivative(acc_ref);

            pinocchio::SE3 contact_ref_se3;
            auto contact_pos = contact_sample_ref.getValue();
            tsid::math::vectorToSE3(contact_pos, contact_ref_se3);
            contact_ref_se3.rotation() = q.toRotationMatrix();
            contact_sample.resize(12, 6);
            Eigen::VectorXd contact_ref_se3_vec(12);
            tsid::math::SE3ToVector(contact_ref_se3, contact_ref_se3_vec);
            contact_sample.setValue(contact_ref_se3_vec);
            contact_sample.setDerivative(vel_ref);
            contact_sample.setSecondDerivative(acc_ref);
        }

        void foot_force_difference_admittance(
            double dt,
            double Mg,
            const Eigen::VectorXd& p_ffda,
            double lf_normal_force,
            double rf_normal_force,
            const Eigen::Vector3d& lf_sensor_force,
            const Eigen::Vector3d& rf_sensor_force,
            const tsid::trajectories::TrajectorySample& torso_sample_ref,
            tsid::trajectories::TrajectorySample& torso_sample)
        {
            IWBC_ASSERT("you need 3 coefficient in p_ffda for ankle admittance", p_ffda.size() == 3);

            if (std::abs(lf_sensor_force(2) + rf_sensor_force(2)) > 10 * Mg)
                IWBC_ERROR("zmp_distributor_admittance : ground force is more than 10*M*g , check sensor measurment: ", std::abs(lf_sensor_force(2) + rf_sensor_force(2)), " > ", 10 * Mg);

            double zctrl = std::abs(lf_normal_force - rf_normal_force) - std::abs(lf_sensor_force(2) - rf_sensor_force(2));
            auto torso_roll = p_ffda[0] * zctrl;

            pinocchio::SE3 torso_ref;
            auto torso_pos = torso_sample_ref.getValue();
            tsid::math::vectorToSE3(torso_pos, torso_ref);

            auto euler = torso_ref.rotation().eulerAngles(0, 1, 2);
            euler[0] += torso_roll;

            auto q = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());

            Eigen::VectorXd vel_ref = torso_sample_ref.getDerivative();
            vel_ref(3) += p_ffda[1] * torso_roll / dt;

            Eigen::VectorXd acc_ref = torso_sample_ref.getSecondDerivative();
            acc_ref(3) += p_ffda[2] * torso_roll / (dt * dt);

            torso_ref.rotation() = q.toRotationMatrix();
            torso_sample.resize(12, 6);
            Eigen::VectorXd torso_ref_vec(12);
            tsid::math::SE3ToVector(torso_ref, torso_ref_vec);
            torso_sample.setValue(torso_ref_vec);
            torso_sample.setDerivative(vel_ref);
            torso_sample.setSecondDerivative(acc_ref);

            //If you want to use the anke height strategy (tested but seems less efficient)
            // lf_ankle_ref.translation()(2) += p_ffda[0] * 0.5 * zctrl;
            // rf_ankle_ref.translation()(2) += -p_ffda[0] * 0.5 * zctrl;
            // set_se3_ref(lf_ankle_ref, "lf");
            // set_se3_ref(rf_ankle_ref, "rf");
            // contact("contact_lfoot")->setReference(to_sample(lf_ankle_ref));
            // contact("contact_rfoot")->setReference(to_sample(rf_ankle_ref));
        }

        double zmp_distributor_admittance(
            double dt,
            const Eigen::VectorXd& p,
            const Eigen::VectorXd& d,
            const double& robot_mass,
            const std::map<std::string, pinocchio::SE3>& contact_ref,
            const std::vector<std::string>& activated_contacts,
            const Eigen::Vector2d& zmp,
            const tsid::trajectories::TrajectorySample& model_current_com,
            Eigen::Matrix<double, 6, 1>& left_fref,
            Eigen::Matrix<double, 6, 1>& right_fref)
        {

            IWBC_ASSERT("you need 6 coefficient in p for zmp_distributor_admittance ", p.size() == 6);

            if (std::abs(zmp(0)) >= 10 && std::abs(zmp(1)) >= 10)
                IWBC_ERROR("zmp_distributor_admittance : something is wrong with input zmp, check sensor measurment: ", std::abs(zmp(0)), " ", std::abs(zmp(1)));

            Eigen::Vector2d zmp_ref = com_to_zmp(model_current_com);

            Eigen::Matrix<double, 6, 1> left_fref_tsid;
            Eigen::Matrix<double, 6, 1> right_fref_tsid;

            zmp_distributor(robot_mass, zmp_ref, contact_ref, activated_contacts, left_fref_tsid, right_fref_tsid);

            Eigen::Matrix<double, 6, 1> left_fref_sensor;
            Eigen::Matrix<double, 6, 1> right_fref_sensor;

            auto alpha = zmp_distributor(robot_mass, zmp, contact_ref, activated_contacts, left_fref_sensor, right_fref_sensor);

            left_fref = left_fref_tsid - p.cwiseProduct(left_fref_tsid - left_fref_sensor) - d.cwiseProduct(left_fref_tsid - left_fref_sensor) / dt;
            right_fref = right_fref_tsid - p.cwiseProduct(right_fref_tsid - right_fref_sensor) - d.cwiseProduct(right_fref_tsid - right_fref_sensor) / dt;

            return alpha;
        }

        double direct_zmp_distributor_admittance(
            double dt,
            const double& robot_mass,
            const Eigen::VectorXd& p,
            const Eigen::VectorXd& d,
            const std::map<std::string, pinocchio::SE3>& contact_ref,
            const std::vector<std::string>& activated_contacts,
            const tsid::trajectories::TrajectorySample& model_current_com,
            const Eigen::Matrix<double, 6, 1>& left_fref_sensor,
            const Eigen::Matrix<double, 6, 1>& right_fref_sensor,
            Eigen::Matrix<double, 6, 1>& left_fref,
            Eigen::Matrix<double, 6, 1>& right_fref)
        {

            Eigen::Vector2d zmp_ref = com_to_zmp(model_current_com);

            Eigen::Matrix<double, 6, 1> left_fref_tsid;
            Eigen::Matrix<double, 6, 1> right_fref_tsid;

            auto alpha = zmp_distributor(robot_mass, zmp_ref, contact_ref, activated_contacts, left_fref_tsid, right_fref_tsid);

            left_fref = left_fref_tsid - p.cwiseProduct(left_fref_tsid - left_fref_sensor) - d.cwiseProduct(left_fref_tsid - left_fref_sensor) / dt;
            right_fref = right_fref_tsid - p.cwiseProduct(right_fref_tsid - right_fref_sensor) - d.cwiseProduct(right_fref_tsid - right_fref_sensor) / dt;

            return alpha;
        }

        double zmp_distributor(
            const double& robot_mass,
            const Eigen::Vector2d& zmp,
            const std::map<std::string, pinocchio::SE3>& contact_ref,
            const std::vector<std::string>& activated_contacts,
            Eigen::Matrix<double, 6, 1>& left_fref,
            Eigen::Matrix<double, 6, 1>& right_fref)
        {
            double alpha = 0;
            auto ct_lf = contact_ref.find("contact_lfoot");
            auto ct_rf = contact_ref.find("contact_rfoot");
            Eigen::Vector2d PR = Eigen::Vector2d::Zero();
            Eigen::Vector2d PL = Eigen::Vector2d::Zero();

            if (std::find(activated_contacts.begin(), activated_contacts.end(), "contact_rfoot") != activated_contacts.end()
                && std::find(activated_contacts.begin(), activated_contacts.end(), "contact_lfoot") != activated_contacts.end()) {

                Eigen::Vector3d zmp3 = Eigen::Vector3d::Zero();
                zmp3.head(2) = zmp;

                IWBC_ASSERT((ct_lf != contact_ref.end()) && (ct_rf != contact_ref.end()), "you need contact_lfoot and contact_rfoot in contact_ref map");
                auto fline = std::make_pair(ct_lf->second.translation(), ct_rf->second.translation());
                PR = ct_rf->second.translation().head(2);
                PL = ct_lf->second.translation().head(2);

                //projection on a flat ground surface
                // TODO porjection on slope thanks to imu
                fline.first(2) = 0;
                fline.second(2) = 0;

                auto proj = closest_point_on_line(zmp3, fline);
                if (std::abs((fline.second - fline.first).norm() - (proj - fline.first).norm() - (fline.second - proj).norm()) < 1e-10) {
                    alpha = (proj - fline.first).norm() / (fline.first - fline.second).norm();
                }
                else if ((proj - fline.first).norm() < (proj - fline.second).norm()) {
                    alpha = 0;
                }
                else if ((proj - fline.first).norm() > (proj - fline.second).norm()) {
                    alpha = 1;
                }
                else {
                    IWBC_ERROR("zmp_distributor this case should not exists");
                }
            }
            else if (std::find(activated_contacts.begin(), activated_contacts.end(), "contact_rfoot") != activated_contacts.end()
                && std::find(activated_contacts.begin(), activated_contacts.end(), "contact_lfoot") == activated_contacts.end()) {
                alpha = 1;
                IWBC_ASSERT(ct_rf != contact_ref.end(), "you need contact_rfoot in contact_ref map");
                PR = ct_rf->second.translation().head(2);
            }
            else if (std::find(activated_contacts.begin(), activated_contacts.end(), "contact_lfoot") != activated_contacts.end()
                && std::find(activated_contacts.begin(), activated_contacts.end(), "contact_rfoot") == activated_contacts.end()) {
                alpha = 0;
                IWBC_ASSERT(ct_lf != contact_ref.end(), "you need contact_lfoot in contact_ref map");
                PL = ct_lf->second.translation().head(2);
            }
            else {
                IWBC_ERROR("No contact_rfoot or contact_rfoot in activated_contacts_ is talos jumping ?");
            }

            left_fref.setZero();
            right_fref.setZero();

            double f_right = -alpha * robot_mass * 9.81;
            double f_left = -(1 - alpha) * robot_mass * 9.81;

            Eigen::Vector2d tau0 = -(PR - zmp) * f_right - (PL - zmp) * f_left;
            Eigen::Vector2d tauL, tauR;
            tauR(1) = alpha * tau0(1);
            tauL(1) = (1 - alpha) * tau0(1);

            if (tau0(0) < 0) {
                tauR(0) = tau0(0);
                tauL(0) = 0;
            }
            else {
                tauR(0) = 0;
                tauL(0) = tau0(0);
            }

            left_fref(2) = f_left;
            left_fref(3) = tauL(1);
            left_fref(4) = -tauL(0);

            right_fref(2) = f_right;
            right_fref(3) = tauR(1);
            right_fref(4) = -tauR(0);

            return alpha;
        }

        Eigen::Vector3d closest_point_on_line(
            const Eigen::Vector3d& point,
            const std::pair<Eigen::Vector3d, Eigen::Vector3d>& line)
        {

            auto unit_line_dir = (line.second - line.first).normalized(); //this needs to be a unit vector
            auto v = point - line.second;
            auto d = v.dot(unit_line_dir);
            return line.second + unit_line_dir * d;
        }

    }; // namespace stabilizer
} // namespace inria_wbc