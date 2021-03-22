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
    } // namespace stabilizer
} // namespace inria_wbc