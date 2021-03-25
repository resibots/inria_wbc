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
            float torso_max_roll,
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

        double zmp_distributor(
            const double& robot_mass,
            const Eigen::Vector2d& cop_filtered,
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

                Eigen::Vector3d cop_filtered3 = Eigen::Vector3d::Zero();
                cop_filtered3.head(2) = cop_filtered;

                // pinocchio::SE3 contact_se3 = contact_ref["contact_lfoot"];
                // auto left_line = closest_line(cop_filtered3, contact_se3, left_foot_contact_points);

                // contact_se3 = contact_ref["contact_rfoot"];
                // auto right_line = closest_line(cop_filtered3, contact_se3, right_foot_contact_points);

                // left_line.first(2) = 0;
                // left_line.second(2) = 0;
                // right_line.first(2) = 0;
                // right_line.second(2) = 0;

                // auto PL_sharp = closest_point_on_line(cop_filtered3, left_line);
                // auto PR_sharp = closest_point_on_line(cop_filtered3, right_line);

                // auto P_alpha = closest_point_on_line(cop_filtered3, std::make_pair(PL_sharp, PR_sharp));

                // alpha = (P_alpha - PL_sharp).norm() / (PL_sharp - PR_sharp).norm();
                // if (std::isnan(alpha))
                //     IWBC_ERROR("alpha is nan");

                auto fline = std::make_pair(contact_ref["contact_lfoot"].translation(), contact_ref["contact_rfoot"].translation());

                //projection on a flat ground surface
                // TODO porjection on slope thanks to imu
                fline.first(2) = 0;
                fline.second(2) = 0;

                auto proj = closest_point_on_line(cop_filtered3, fline);

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
            Eigen::Vector2d tau0 = -(PR - cop_filtered) * f_right - (PL - cop_filtered) * f_left;
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
            left_fref(3) = tauL(0);
            left_fref(4) = tauL(1);

            right_fref(2) = f_right;
            right_fref(3) = tauR(0);
            right_fref(4) = tauR(1);

            // std::cout << "alpha " << alpha << std::endl;
            // std::cout << "left_fref " << left_fref.transpose() << std::endl;
            // std::cout << "right_fref " << right_fref.transpose() << std::endl;

            return alpha;
            // TO DO TAKE IMU AND PROJECT THE POINT ON THE GROUND SURFACE
            // IN DOUBLE SUPPORT PL_sharp, COP, PR_sharp are forming a line with a slope from IMU
            // Then project this on flat surface
        }

        // // fROM https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
        // //    nvert: Number of vertices in the polygon. Whether to repeat the first vertex at the end has been discussed in the article referred above.
        // //   vertx, verty: Arrays containing the x- and y-coordinates of the polygon's vertices.
        // //  testx, testy: X- and y-coordinate of the test point.
        // int pnpoly(int nvert, float* vertx, float* verty, float testx, float testy)
        // {
        //     int i, j, c = 0;
        //     for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        //         if (((verty[i] > testy) != (verty[j] > testy)) && (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
        //             c = !c;
        //     }
        //     return c;
        // }

        Eigen::Vector3d closest_point_on_line(
            const Eigen::Vector3d point,
            const std::pair<Eigen::Vector3d, Eigen::Vector3d>& line)
        {

            auto unit_line_dir = (line.second - line.first).normalized(); //this needs to be a unit vector
            auto v = point - line.second;
            auto d = v.dot(unit_line_dir);
            return line.second + unit_line_dir * d;
        }

        //the problem here is that if the cop is outside of support polygon you can have non relevant results
        //then you have to create special cases with closest vertices
        //give backs the 2 points from contact_points defining the closest line to point
        //contact_points are expressed in the contact_se3 frame
        std::pair<Eigen::Vector3d, Eigen::Vector3d> closest_segment(
            const Eigen::Vector3d& point,
            const pinocchio::SE3& contact_se3,
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& contact_points)
        {

            auto P0 = point;
            Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
            P0(2) = 1;
            mat.col(0) = P0;

            double min_dist = 1000000;
            std::pair<Eigen::Vector3d, Eigen::Vector3d> closest_line;

            for (uint i = 0; i < contact_points.cols(); i++) {

                Eigen::Vector3d P1 = contact_points.col(i);
                P1 = contact_se3.translation() + contact_se3.rotation() * P1;

                P1(2) = 1;
                mat.col(1) = P1;

                for (uint j = 0; j < contact_points.cols(); j++) {
                    if (i != j) {
                        Eigen::Vector3d P2 = contact_points.col(j);
                        P2 = contact_se3.translation() + contact_se3.rotation() * P2;
                        P2(2) = 1;
                        mat.col(2) = P2;
                        //https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
                        double d = std::abs(mat.determinant()) / (P2 - P1).norm();

                        auto pclose = closest_point_on_line(P0, std::make_pair(P1, P2));
                        // auto d2 = (P0 - pclose).norm(); // same than d
                        // check that we have the closest point and that P0 is on segment P1 - P2
                        if (d < min_dist && ((P2 - P1).norm() == (pclose - P1).norm() + (P2 - pclose).norm())) {
                            min_dist = d;
                            closest_line.first = contact_se3.translation() + contact_se3.rotation() * contact_points.col(i);
                            closest_line.second = contact_se3.translation() + contact_se3.rotation() * contact_points.col(j);
                        }
                    }
                }
            }
            //if the cop projection is not on any
            if (min_dist == 1000000) {
                IWBC_ERROR("COP outside of support polygon");
            }
            return closest_line;
        }

    }; // namespace stabilizer
} // namespace inria_wbc