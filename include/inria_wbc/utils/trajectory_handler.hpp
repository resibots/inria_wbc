#ifndef IWBC_TRAJECTORY_HANDLER_HPP
#define IWBC_TRAJECTORY_HANDLER_HPP
#include <cassert>
#define assertm(exp, msg) assert(((void)msg, exp))
namespace trajectory_handler
{

    inline Eigen::VectorXd minimum_jerk_polynom(Eigen::VectorXd x0, Eigen::VectorXd xf, double t, double trajectory_duration)
    {
        assertm(x0.size() == xf.size(), "minimum_jerk_polynom x0 and xf should have the same size");
        double td = t / trajectory_duration;
        Eigen::VectorXd xt = x0 + (xf - x0) * (6 * td * td * td * td * td - 15 * td * td * td * td + 10 * td * td * td);
        return xt;
    }

    inline std::vector<Eigen::VectorXd> compute_traj(const Eigen::VectorXd &start, const Eigen::VectorXd &dest, double dt, double trajectory_duration)
    {
        std::vector<Eigen::VectorXd> trajectory;
        uint n_steps = std::floor(trajectory_duration / dt);
        for (uint i = 0; i < n_steps; i++)
        {
            trajectory.push_back(minimum_jerk_polynom(start, dest, dt * i, trajectory_duration));
        }
        return trajectory;
    }

    inline std::vector<pinocchio::SE3> compute_traj(const pinocchio::SE3 &start, const pinocchio::SE3 &dest, double dt, double trajectory_duration)
    {
        Eigen::VectorXd eig_start(7), eig_dest(7), eig_xt(7);
        Eigen::Quaterniond start_rotation(start.rotation());
        Eigen::Quaterniond dest_rotation(dest.rotation());
        eig_start << start.translation(), start_rotation.coeffs();
        eig_dest << dest.translation(), dest_rotation.coeffs();
        
        std::vector<pinocchio::SE3> trajectory;
        uint n_steps = std::floor(trajectory_duration / dt);

        for (uint i = 0; i < n_steps; i++)
        {
            eig_xt = minimum_jerk_polynom(eig_start, eig_dest, dt * i, trajectory_duration);
            Eigen::Quaterniond quat(eig_xt(6), eig_xt(3), eig_xt(4), eig_xt(5));
            Eigen::Vector3d pos = eig_xt.head(3);
            pinocchio::SE3 xt = pinocchio::SE3(quat, pos);
            trajectory.push_back(xt);
        }
        return trajectory;
    }

} // namespace trajectory_handler
#endif