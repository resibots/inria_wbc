#include "inria_wbc/stabilizers/stabilizer.hpp"

#include "inria_wbc/exceptions.hpp"
#include <tsid/math/utils.hpp>

namespace inria_wbc {
    namespace stabilizer {

        //Make the measured cop match the current desired cop by changing the com ref
        void com_admittance(
            double dt,
            const Eigen::VectorXd& p,
            const Eigen::VectorXd& d,
            const Eigen::MatrixXd& velocity,
            const Eigen::Vector2d& cop_filtered,
            const tsid::trajectories::TrajectorySample& com_ref,
            const tsid::InverseDynamicsFormulationAccForce::Data& data,
            tsid::trajectories::TrajectorySample& se3_sample)
        {
            IWBC_ASSERT("you need 6 coefficient in p for com admittance", p.size() == 6);
            IWBC_ASSERT("you need 6 coefficient in d for com admittance.", d.size() == 6);
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
            Eigen::VectorXd vref_m = com_ref.vel - (Eigen::Vector3d(error(0), error(1), 0) / dt);

            error = p.block(4, 0, 1, 2).array() * cor.array() + d.block(4, 0, 1, 2).array() * cor_v.array();
            Eigen::VectorXd aref_m = com_ref.acc - (Eigen::Vector3d(error(0), error(1), 0) / (dt * dt));

            se3_sample.pos = ref_m;
            se3_sample.vel = vref_m;
            se3_sample.acc = aref_m;
        }

        //correct the roll and pitch angle of the ankle to force the foot cop to be in the middle of the foot
        void ankle_admittance(
            double dt,
            const std::string& foot,
            const Eigen::Vector2d& cop_foot,
            const Eigen::VectorXd& p,
            pinocchio::SE3 ankle_ref,
            std::map<std::string, pinocchio::SE3> contact_ref,
            tsid::trajectories::TrajectorySample& contact_sample,
            tsid::trajectories::TrajectorySample& se3_sample)
        {
            IWBC_ASSERT("you need 6 coefficient in p for ankle admittance", p.size() == 6);

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

            se3_sample.resize(12, 6);
            tsid::math::SE3ToVector(ankle_ref, se3_sample.pos);
            se3_sample.vel = vel_ref;
            se3_sample.acc = acc_ref;

            contact_ref["contact_" + foot + "foot"].rotation() = q.toRotationMatrix();
            contact_sample.resize(12, 6);
            tsid::math::SE3ToVector(contact_ref["contact_" + foot + "foot"], contact_sample.pos);
            contact_sample.vel = vel_ref;
            contact_sample.acc = acc_ref;
        }

        //correct the roll of the torso to have an equal left and right foot force measurment
        //FROM :
        //Biped Walking Stabilization Based on Linear Inverted Pendulum Tracking
        //by Shuuji Kajita, Mitsuharu Morisawa, Kanako Miura, Shin’ichiro Nakaoka,Kensuke Harada, Kenji Kaneko, Fumio Kanehiro and Kazuhito Yokoi
        //ALSO INSPIRED BY :
        //Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control
        //by Stéphane Caron, Abderrahmane Kheddar, Olivier Tempier
        //https://github.com/stephane-caron/lipm_walking_controller/blob/master/src/Stabilizer.cpp
        void foot_force_difference_admittance(
            double dt,
            Eigen::VectorXd p_ffda,
            pinocchio::SE3 torso_ref,
            double lf_normal_force,
            double rf_normal_force,
            const Eigen::Vector3d& lf_sensor_force,
            const Eigen::Vector3d& rf_sensor_force,
            tsid::trajectories::TrajectorySample& torso_sample)
        {
            IWBC_ASSERT("you need 3 coefficient in p_ffda for ankle admittance", p_ffda.size() == 3);

            double zctrl = (lf_normal_force - rf_normal_force) - (lf_sensor_force(2) - rf_sensor_force(2));
            auto torso_roll = -p_ffda[0] * zctrl;
            auto euler = torso_ref.rotation().eulerAngles(0, 1, 2);
            euler[0] += torso_roll;

            auto q = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());

            Eigen::VectorXd vel_ref = Eigen::VectorXd::Zero(6);
            vel_ref(3) = p_ffda[1] * torso_roll / dt;

            Eigen::VectorXd acc_ref = Eigen::VectorXd::Zero(6);
            acc_ref(3) = p_ffda[2] * torso_roll / (dt * dt);

            torso_ref.rotation() = q.toRotationMatrix();
            torso_sample.resize(12, 6);
            tsid::math::SE3ToVector(torso_ref, torso_sample.pos);
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

        double zmp_distributor_admittance(
            double dt,
            const Eigen::VectorXd& p,
            const Eigen::VectorXd& d,
            const double& robot_mass,
            const Eigen::Vector2d& zmp,
            std::map<std::string, pinocchio::SE3> contact_ref,
            const std::vector<std::string>& activated_contacts,
            const tsid::InverseDynamicsFormulationAccForce::Data& data,
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& left_foot_contact_points,
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& right_foot_contact_points,
            Eigen::Matrix<double, 6, 1>& left_fref,
            Eigen::Matrix<double, 6, 1>& right_fref)
        {

            IWBC_ASSERT("you need 6 coefficient in p for zmp_distributor_admittance ", p.size() == 6);

            Eigen::Vector2d a = data.acom[0].head<2>();
            Eigen::Vector3d com = data.com[0];
            Eigen::Vector2d zmp_ref = com.head<2>() - com(2) / 9.81 * a; //com because this is the target

            Eigen::Matrix<double, 6, 1> left_fref_tsid = left_fref;
            Eigen::Matrix<double, 6, 1> right_fref_tsid = right_fref;

            zmp_distributor(robot_mass, zmp_ref, contact_ref, activated_contacts, left_foot_contact_points, right_foot_contact_points, left_fref_tsid, right_fref_tsid);

            Eigen::Matrix<double, 6, 1> left_fref_sensor = left_fref;
            Eigen::Matrix<double, 6, 1> right_fref_sensor = right_fref;

            auto alpha = zmp_distributor(robot_mass, zmp, contact_ref, activated_contacts, left_foot_contact_points, right_foot_contact_points, left_fref_sensor, right_fref_sensor);

            left_fref = left_fref_tsid - p.cwiseProduct(left_fref_tsid - left_fref_sensor) - d.cwiseProduct(left_fref_tsid - left_fref_sensor) / dt;
            right_fref = right_fref_tsid - p.cwiseProduct(right_fref_tsid - right_fref_sensor) - d.cwiseProduct(left_fref_tsid - left_fref_sensor) / dt;

            // std::cout << "right_fref_tsid " << right_fref_tsid.transpose() << std::endl;
            // std::cout << "left_fref_tsid " << left_fref_tsid.transpose() << std::endl;
            // std::cout << "right_fref_sensor " << right_fref_sensor.transpose() << std::endl;
            // std::cout << "left_fref_sensor " << left_fref_sensor.transpose() << std::endl;
            // std::cout << "left_fref " << left_fref.transpose() << std::endl;
            // std::cout << "right_fref " << right_fref.transpose() << std::endl;
            // std::cout << "p " << p.transpose() << std::endl;

            return alpha;
        }

        //ZMP to feet forces and torques
        //FROM :
        //Biped Walking Stabilization Based on Linear Inverted Pendulum Tracking
        //by Shuuji Kajita, Mitsuharu Morisawa, Kanako Miura, Shin’ichiro Nakaoka,Kensuke Harada, Kenji Kaneko, Fumio Kanehiro and Kazuhito Yokoi
        //MODIFIED :
        //I took a simpler heuristic for alpha 
        //Torque sign is different to adjust to Talos xy convention
        double zmp_distributor(
            const double& robot_mass,
            const Eigen::Vector2d& zmp,
            std::map<std::string, pinocchio::SE3> contact_ref,
            const std::vector<std::string>& activated_contacts,
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& left_foot_contact_points,
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& right_foot_contact_points,
            Eigen::Matrix<double, 6, 1>& left_fref,
            Eigen::Matrix<double, 6, 1>& right_fref)
        {
            double alpha = 0;

            if (std::find(activated_contacts.begin(), activated_contacts.end(), "contact_rfoot") != activated_contacts.end()
                && std::find(activated_contacts.begin(), activated_contacts.end(), "contact_lfoot") != activated_contacts.end()) {

                Eigen::Vector3d zmp3 = Eigen::Vector3d::Zero();
                zmp3.head(2) = zmp;

                auto fline = std::make_pair(contact_ref["contact_lfoot"].translation(), contact_ref["contact_rfoot"].translation());

                //projection on a flat ground surface
                // TODO porjection on slope thanks to imu
                fline.first(2) = 0;
                fline.second(2) = 0;

                auto proj = closest_point_on_line(zmp3, fline);

                if ((fline.second - fline.first).norm() == (proj - fline.first).norm() + (fline.second - proj).norm()) {
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
            }
            else if (std::find(activated_contacts.begin(), activated_contacts.end(), "contact_lfoot") != activated_contacts.end()
                && std::find(activated_contacts.begin(), activated_contacts.end(), "contact_rfoot") == activated_contacts.end()) {
                alpha = 0;
            }
            else {
                IWBC_ERROR("No contact_rfoot or contact_rfoot in activated_contacts_ is talos jumping ?");
            }

            left_fref.setZero();
            right_fref.setZero();

            double f_right = -alpha * robot_mass * 9.81;
            double f_left = -(1 - alpha) * robot_mass * 9.81;

            auto PR = contact_ref["contact_rfoot"].translation().head(2);
            auto PL = contact_ref["contact_lfoot"].translation().head(2);
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
            const Eigen::Vector3d point,
            const std::pair<Eigen::Vector3d, Eigen::Vector3d>& line)
        {

            auto unit_line_dir = (line.second - line.first).normalized(); //this needs to be a unit vector
            auto v = point - line.second;
            auto d = v.dot(unit_line_dir);
            return line.second + unit_line_dir * d;
        }

    }; // namespace stabilizer
} // namespace inria_wbc