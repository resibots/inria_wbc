#ifndef IWBC_TRAJS_GENERATOR_HPP
#define IWBC_TRAJS_GENERATOR_HPP

#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Core>
#include <cassert>
#include <vector>
#define assertm(exp, msg) assert(((void)msg, exp))

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>

namespace inria_wbc {
namespace trajs {

    namespace d_order
    {
        constexpr uint16_t ZERO = 0;
        constexpr uint16_t FIRST = 1;
        constexpr uint16_t SECOND = 2;
    }

    template <uint16_t DO>
    Eigen::VectorXd minimum_jerk_polynom(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, double t, double trajectory_duration)
    {
       IWBC_ERROR("minimum_jerk_polynom is not implemented for " + std::to_string(DO) + " derivative order.");
    }

    template <>
    inline Eigen::VectorXd minimum_jerk_polynom<d_order::ZERO>(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, double t, double trajectory_duration)
    {
        assertm(x0.size() == xf.size(), "minimum_jerk_polynom x0 and xf should have the same size");
        double td = t / trajectory_duration;
        double td_pow_3 = std::pow(td, 3);

        Eigen::VectorXd xt = x0 + (xf - x0) * (6 * td * td * td_pow_3 - 15 * td * td_pow_3 + 10 * td_pow_3);
        return xt;
    }

    // first order derivative value of minimum jerk polynom at time t
    template <>
    inline Eigen::VectorXd minimum_jerk_polynom<d_order::FIRST>(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, double t, double trajectory_duration)
    {
        assertm(x0.size() == xf.size(), "minimum_jerk_polynom x0 and xf should have the same size");
        double td = t / trajectory_duration;
        double td_pow_2 = td * td;
        Eigen::VectorXd xt = (xf - x0) * (30 * td * td * td_pow_2 - 60 * td * td_pow_2  + 30 * td_pow_2) / trajectory_duration;
        return xt;
    }

    // second order derivative value of minimum jerk polynom at time t
    template <>
    inline Eigen::VectorXd minimum_jerk_polynom<d_order::SECOND>(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, double t, double trajectory_duration)
    {
        assertm(x0.size() == xf.size(), "minimum_jerk_polynom x0 and xf should have the same size");
        double td = t / trajectory_duration;
        Eigen::VectorXd xt = (xf - x0) * (120 * td * td * td - 180 * td * td  + 60 * td) / std::pow(trajectory_duration, 2.0);
        return xt;
    }

    inline Eigen::VectorXd minimum_jerk_polynom(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, double t, double trajectory_duration)
    {
        return minimum_jerk_polynom<d_order::ZERO>(x0, xf, t, trajectory_duration);
    }


    template <uint16_t ORDER = d_order::ZERO>
    inline std::vector<Eigen::VectorXd> min_jerk_trajectory(const Eigen::VectorXd& start, const Eigen::VectorXd& dest, double dt, double trajectory_duration)
    {
        if(ORDER > d_order::SECOND)
            IWBC_ERROR("min_jerk_trajectory is not implemented for derivative of order " + std::to_string(ORDER) + ".");
        
        uint n_steps = std::floor(trajectory_duration / dt);
        std::vector<Eigen::VectorXd> trajectory(n_steps);
        for (uint i = 0; i < n_steps; i++)
            trajectory[i] = minimum_jerk_polynom<ORDER>(start, dest, dt * i, trajectory_duration);
        return trajectory;
    }

    template<uint16_t ORDER>
    inline std::vector<Eigen::VectorXd> min_jerk_trajectory(const pinocchio::SE3& start, const pinocchio::SE3& dest, double dt, double trajectory_duration)
    {
        if(ORDER > d_order::SECOND || ORDER < d_order::FIRST)
            IWBC_ERROR("min_jerk_trajectory is not implemented for derivative of order " + std::to_string(ORDER) + ".");

        Eigen::Vector3d pos_start, pos_dest, pos_d_xt;
        pos_start = start.translation();
        pos_dest = dest.translation();

        Eigen::AngleAxisd aa_rot(start.rotation().transpose() * dest.rotation());
        const Eigen::Vector3d& rot_axis = aa_rot.axis();
        double rot_angle = aa_rot.angle();

        // Interpolate time as min jerk to use in slerp routine (otherwise slerp gives constant angular velocity result)
        Eigen::VectorXd ang_start(1), ang_dest(1), ang_d_xt(1);
        ang_start << 0.;
        ang_dest << rot_angle;

        uint n_steps = std::floor(trajectory_duration / dt);
        std::vector<Eigen::VectorXd> trajectory;

        for (uint i = 0; i < n_steps; i++) {
            Eigen::Matrix<double,6,1> d_vec;

            // Min jerk trajectory for translation
            pos_d_xt = minimum_jerk_polynom<ORDER>(pos_start, pos_dest, dt * i, trajectory_duration);
            ang_d_xt = minimum_jerk_polynom<ORDER>(ang_start, ang_dest, dt * i, trajectory_duration);

            d_vec.head<3>() = pos_d_xt;
            d_vec.tail<3>() = start.rotation() * (ang_d_xt(0) * rot_axis);

            trajectory.push_back(d_vec);
        }
        return trajectory;
    }

    inline std::vector<pinocchio::SE3> min_jerk_trajectory(const pinocchio::SE3& start, const pinocchio::SE3& dest, double dt, double trajectory_duration)
    {
        Eigen::Vector3d pos_start, pos_dest, pos_xt;
        pos_start = start.translation();
        pos_dest = dest.translation();

        Eigen::AngleAxisd aa_rot(start.rotation().transpose() * dest.rotation());
        const Eigen::Vector3d& rot_axis = aa_rot.axis();
        double rot_angle = aa_rot.angle();

        // Interpolate time as min jerk to use in slerp routine (otherwise slerp gives constant angular velocity result)
        Eigen::VectorXd ang_start(1), ang_dest(1), ang_xt(1);
        ang_start << 0.;
        ang_dest << rot_angle;

        uint n_steps = std::floor(trajectory_duration / dt);
        std::vector<pinocchio::SE3> trajectory;

        for (uint i = 0; i < n_steps; i++) {

            // Min jerk trajectory for translation and rotation
            pos_xt = minimum_jerk_polynom(pos_start, pos_dest, dt * i, trajectory_duration);
            ang_xt = minimum_jerk_polynom(ang_start, ang_dest, dt * i, trajectory_duration);

            Eigen::Matrix3d rot_xt = start.rotation() * Eigen::AngleAxisd(ang_xt(0), rot_axis).toRotationMatrix();

            pinocchio::SE3 xt = pinocchio::SE3(rot_xt, pos_xt);
            trajectory.push_back(xt);
        }
        return trajectory;
    }

    template <typename T>
    inline std::vector<T> constant_traj(const T& p, double dt, double trajectory_duration)
    {
        uint n_steps = std::floor(trajectory_duration / dt);
        std::vector<T> trajectory(n_steps);
        std::fill(trajectory.begin(), trajectory.end(), p);
        return trajectory;
    }

} // namespace trajs
} // namespace inria_wbc

#endif
