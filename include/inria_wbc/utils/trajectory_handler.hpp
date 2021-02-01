#ifndef IWBC_TRAJECTORY_HANDLER_HPP
#define IWBC_TRAJECTORY_HANDLER_HPP
#include <cassert>
#define assertm(exp, msg) assert(((void)msg, exp))
namespace trajectory_handler {

    inline Eigen::VectorXd minimum_jerk_polynom(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, double t, double trajectory_duration)
    {
        assertm(x0.size() == xf.size(), "minimum_jerk_polynom x0 and xf should have the same size");
        double td = t / trajectory_duration;
        Eigen::VectorXd xt = x0 + (xf - x0) * (6 * td * td * td * td * td - 15 * td * td * td * td + 10 * td * td * td);
        return xt;
    }

    inline std::vector<Eigen::VectorXd> compute_traj(const Eigen::VectorXd& start, const Eigen::VectorXd& dest, double dt, double trajectory_duration)
    {
        uint n_steps = std::floor(trajectory_duration / dt);
        std::vector<Eigen::VectorXd> trajectory(n_steps);
        for (uint i = 0; i < n_steps; i++)
            trajectory[i] = minimum_jerk_polynom(start, dest, dt * i, trajectory_duration);
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