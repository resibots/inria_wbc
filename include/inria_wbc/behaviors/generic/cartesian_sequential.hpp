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
                virtual ~CartesianSequential() {};
                void update(const controllers::SensorData& sensor_data = {}) override;
                std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };
                
                std::vector<std::vector<double>> get_rh_targets() const {return rh_targets;}
                std::vector<std::vector<double>> get_lh_targets() const {return lh_targets;}

                std::vector<std::vector<pinocchio::SE3>> get_trajectories_right() const { return trajectories_r_; }
                std::vector<std::vector<pinocchio::SE3>> get_trajectories_left() const { return trajectories_l_; }

                int get_traj_selector() const { return traj_selector_; }
                bool get_loop() const { return loop_; }

            private:
                int time_ = 0;
                int traj_selector_ = 0;

                //right hand
                std::vector<std::vector<double>> rh_targets;
                std::vector<std::vector<double>> lh_targets;

                std::vector<std::vector<double>> rh_rots;
                std::vector<std::vector<double>> lh_rots;

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
                bool absolute_; //form YAML
                int number_of_targets; //defined exploiting te YAML file
            };


        } //namespace generic
    } //namespace behaviors
} //namespace inria_wbc

#endif