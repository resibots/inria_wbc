#ifndef IWBC_BEHAVIOR_GENERIC_CARTESIAN_SEQUENTIAL_HPP
#define IWBC_BEHAVIOR_GENERIC_CARTESIAN_SEQUENTIAL_HPP

#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/trajs/trajectory_generator.hpp>
#include <inria_wbc/trajs/loader.hpp>

namespace inria_wbc{
    namespace behaviors{
        namespace generic{
//defining the cartesian sequential class. Its purpose is to make the robot's hands able to achieve several positions given an order.
            class CartesianSequential : public Behavior{
            public:
                CartesianSequential(const controller_ptr_t& controller, const YAML::Node& config);
                CartesianSequential() = delete;
                CartesianSequential(const CartesianSequential&) = delete;
                virtual ~CartesianSequential();
                void update(const controllers::SensorData& sensor_data = {}) override;
                std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };

            private:
                int time_ = 0;
                int traj_selector_ = 0;
                int task_number = 0;

                //right hand
                std::vector<std::vector<pinocchio::SE3>> trajectories_r_;
                std::vector<std::vector<Eigen::VectorXd>> trajectories_d_r_;
                std::vector<std::vector<Eigen::VectorXd>> trajectories_dd_r_;
                //.......................................

                //left hand
                std::vector<std::vector<pinocchio::SE3>> trajectories_l_;
                std::vector<std::vector<Eigen::VectorXd>> trajectories_d_l_;
                std::vector<std::vector<Eigen::VectorXd>> trajectories_dd_l_;
                //.......................................

                float trajectory_duration_; // from YAML
                std::vector<std::string> task_names_; // from YAML
                bool loop_; // from YAML
                int number_of_targets; //defined exploiting te YAML file
            };


        } //namespace generic
    } //namespace behaviors
} //namespace inria_wbc

#endif