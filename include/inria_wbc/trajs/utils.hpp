#ifndef IWBC_TRAJS_UTILS_HPP
#define IWBC_TRAJS_UTILS_HPP

#include <Eigen/Dense>
#include <tsid/trajectories/trajectory-base.hpp>
#include <tsid/math/utils.hpp>


namespace inria_wbc {
    namespace trajs {

        inline pinocchio::SE3 se3_from_sample(const tsid::trajectories::TrajectorySample& sample)
        {
            pinocchio::SE3 se3;
            se3.translation() = sample.getValue().head<3>();
            se3.rotation() = Eigen::Matrix3d::Map(sample.getValue().data()+3);
            return se3;
        }

        inline tsid::trajectories::TrajectorySample to_sample(const Eigen::VectorXd& ref)
        {
            tsid::trajectories::TrajectorySample sample(ref.size(), ref.size());
            sample.setValue(ref);
            return sample;
        }

        inline tsid::trajectories::TrajectorySample to_sample(const pinocchio::SE3& ref)
        {
            tsid::trajectories::TrajectorySample sample(12,6);
            Eigen::VectorXd ref_vec(12);
            tsid::math::SE3ToVector(ref, ref_vec);
            sample.setValue(ref_vec);
            return sample;
        }

        inline tsid::trajectories::TrajectorySample to_sample(const Eigen::VectorXd& ref, 
            const Eigen::VectorXd& ref_vel, const Eigen::VectorXd& ref_acc)
        {
            tsid::trajectories::TrajectorySample sample(ref.size(), ref.size());
            sample.setValue(ref);
            sample.setDerivative(ref_vel);
            sample.setSecondDerivative(ref_acc);
            return sample;
        }

        inline tsid::trajectories::TrajectorySample to_sample(const pinocchio::SE3& ref, 
            const Eigen::VectorXd& ref_vel, const Eigen::VectorXd& ref_acc)
        {
            IWBC_ASSERT(ref_vel.size() == 6, "velocity reference for SE3 must be a vector 6.");
            IWBC_ASSERT(ref_acc.size() == 6, "acceleration reference for SE3 must be a vector 6.");

            Eigen::VectorXd ref_vec(12);

            tsid::trajectories::TrajectorySample sample(12,6);
            tsid::math::SE3ToVector(ref, ref_vec);
            sample.setValue(ref_vec);
            sample.setDerivative(ref_vel);
            sample.setSecondDerivative(ref_acc);
            return sample;
        }

        inline std::vector<tsid::trajectories::TrajectorySample> to_sample_trajectory(
            const std::vector<pinocchio::SE3>& traj)
        {
            std::vector<tsid::trajectories::TrajectorySample> sample_trajectory;
            for(int i=0; i < traj.size(); ++i)
                sample_trajectory.push_back(to_sample(traj[i]));
            return sample_trajectory;
        }

        inline std::vector<tsid::trajectories::TrajectorySample> to_sample_trajectory(
            const std::vector<pinocchio::SE3>& traj, const std::vector<Eigen::VectorXd>& vels, 
            const std::vector<Eigen::VectorXd>& accs)
        {
            std::vector<tsid::trajectories::TrajectorySample> sample_trajectory;
            for(int i=0; i < traj.size(); ++i)
                sample_trajectory.push_back(to_sample(traj[i], vels[i], accs[i]));
            return sample_trajectory;
        }

        inline std::vector<tsid::trajectories::TrajectorySample> to_sample_trajectory(
            const std::vector<Eigen::VectorXd>& traj)
        {
            std::vector<tsid::trajectories::TrajectorySample> sample_trajectory;
            for(int i=0; i < traj.size(); ++i)
                sample_trajectory.push_back(to_sample(traj[i]));
            return sample_trajectory;
        }

        inline std::vector<tsid::trajectories::TrajectorySample> to_sample_trajectory(
            const std::vector<Eigen::VectorXd>& traj, const std::vector<Eigen::VectorXd>& vels, 
            const std::vector<Eigen::VectorXd>& accs)
        {
            std::vector<tsid::trajectories::TrajectorySample> sample_trajectory;
            for(int i=0; i < traj.size(); ++i)
                sample_trajectory.push_back(to_sample(traj[i], vels[i], accs[i]));
            return sample_trajectory;
        }

    } // namespace trajs
} // namespace inria_wbc

#endif // IWBC_TRAJS_UTILS_HPP