#ifndef IWBC_TRAJECTORY_HANDLER_HPP
#define IWBC_TRAJECTORY_HANDLER_HPP

#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Core>
#include <cassert>
#include <vector>
#define assertm(exp, msg) assert(((void)msg, exp))

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>

namespace trajectory_handler {

    namespace order
    {
        constexpr uint16_t ZERO = 0;
        constexpr uint16_t FIRST = 1;
        constexpr uint16_t SECOND = 2;
    }

    template <uint16_t d_order>
    Eigen::VectorXd minimum_jerk_polynom(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, double t, double trajectory_duration)
    {
       IWBC_ERROR("minimum_jerk_polynom is not implemented for " + std::to_string(d_order) + " derivative order.");
    }

    template <>
    inline Eigen::VectorXd minimum_jerk_polynom<order::ZERO>(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, double t, double trajectory_duration)
    {
        assertm(x0.size() == xf.size(), "minimum_jerk_polynom x0 and xf should have the same size");
        double td = t / trajectory_duration;
        double td_pow_3 = std::pow(td, 3);

        Eigen::VectorXd xt = x0 + (xf - x0) * (6 * td * td * td_pow_3 - 15 * td * td_pow_3 + 10 * td_pow_3);
        return xt;
    }

    // first order derivative value of minimum jerk polynom at time t
    template <>
    inline Eigen::VectorXd minimum_jerk_polynom<order::FIRST>(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, double t, double trajectory_duration)
    {
        assertm(x0.size() == xf.size(), "minimum_jerk_polynom x0 and xf should have the same size");
        double td = t / trajectory_duration;
        double td_pow_2 = td * td;
        Eigen::VectorXd xt = (xf - x0) * (30 * td * td * td_pow_2 - 60 * td * td_pow_2  + 30 * td_pow_2);
        return xt;
    }

    // second order derivative value of minimum jerk polynom at time t
    template <>
    inline Eigen::VectorXd minimum_jerk_polynom<order::SECOND>(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, double t, double trajectory_duration)
    {
        assertm(x0.size() == xf.size(), "minimum_jerk_polynom x0 and xf should have the same size");
        double td = t / trajectory_duration;
        Eigen::VectorXd xt = (xf - x0) * (120 * td * td * td - 120 * td * td  + 60 * td);
        return xt;
    }

    inline Eigen::VectorXd minimum_jerk_polynom(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, double t, double trajectory_duration)
    {
        return minimum_jerk_polynom<order::ZERO>(x0, xf, t, trajectory_duration);
    }


    template <uint8_t DO = order::ZERO>
    inline std::vector<Eigen::VectorXd> compute_traj(const Eigen::VectorXd& start, const Eigen::VectorXd& dest, double dt, double trajectory_duration)
    {
        if(DO > order::SECOND)
            IWBC_ERROR("compute_traj is not implemented for " + std::to_string(DO) + " derivative order.");
        
        uint n_steps = std::floor(trajectory_duration / dt);
        std::vector<Eigen::VectorXd> trajectory(n_steps);
        for (uint i = 0; i < n_steps; i++)
            trajectory[i] = minimum_jerk_polynom<DO>(start, dest, dt * i, trajectory_duration);
        return trajectory;
    }

    inline std::vector<pinocchio::SE3> compute_traj(const pinocchio::SE3& start, const pinocchio::SE3& dest, double dt, double trajectory_duration)
    {
        Eigen::Vector3d pos_start, pos_dest, pos_xt;
        pos_start = start.translation();
        pos_dest = dest.translation();
        Eigen::Quaterniond quat_start(start.rotation());
        Eigen::Quaterniond quat_dest(dest.rotation());

        // Interpolate time as min jerk to use in slerp routine (otherwise slerp gives constant angular velocity result)
        Eigen::VectorXd t_start(1), t_dest(1), t_xt(1);
        t_start << 0.;
        t_dest << 1.;

        uint n_steps = std::floor(trajectory_duration / dt);
        std::vector<pinocchio::SE3> trajectory;

        for (uint i = 0; i < n_steps; i++) {
            // Min jerk trajectory for translation
            pos_xt = minimum_jerk_polynom(pos_start, pos_dest, dt * i, trajectory_duration);
            t_xt = minimum_jerk_polynom(t_start, t_dest, dt * i, trajectory_duration);
            // Slerp interpolation for quaternion
            Eigen::Quaterniond quat_xt = quat_start.slerp(t_xt(0), quat_dest);
            pinocchio::SE3 xt = pinocchio::SE3(quat_xt, pos_xt);
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
} // namespace trajectory_handler
#endif
