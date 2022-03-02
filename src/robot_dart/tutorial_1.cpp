#include <iostream>
#include <signal.h>

#include "inria_wbc/behaviors/humanoid/squat.hpp"
#include "inria_wbc/controllers/talos_pos_tracker.hpp"
#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/robot_dart/cmd.hpp"
#include "inria_wbc/robot_dart/utils.hpp"

#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robots/talos.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

volatile sig_atomic_t stop;
void user_interrupt(int signum)
{
    stop = 1;
}

int main(int argc, char* argv[])
{
    signal(SIGINT, user_interrupt);

    //Don't forget to set the content of yaml files before running
    //You might need to set base_path and urdf

    ////////// INRIA_WBC ////////////////////////////////////////////////////////////////////
    //First construct the controller from the controller config file
    std::string controller_conf_path = "../etc/talos/talos_pos_tracker.yaml";
    auto controller_yaml = IWBC_CHECK(YAML::LoadFile(controller_conf_path));

    //TalosPosTracker have stabilization but requires sensor_data
    auto controller = std::make_shared<inria_wbc::controllers::TalosPosTracker>(controller_yaml);

    //Now contruct the behavior, the behavior will send reference trajectories to the controller
    std::string behavior_conf_path = "../etc/talos/squat.yaml";
    auto behavior_yaml = IWBC_CHECK(YAML::LoadFile(behavior_conf_path));
    auto behavior = std::make_shared<inria_wbc::behaviors::humanoid::Squat>(controller, behavior_yaml);

    ////////// ROBOT_DART ////////////////////////////////////////////////////////////////////
    //Create talos robot
    auto robot = std::make_shared<robot_dart::robots::Talos>(1000, "talos/talos.urdf");
    robot->set_actuator_types("torque"); //alternatives are velocity, servo

    //Create dart simulation
    double dt = 0.001;
    robot_dart::RobotDARTSimu simu(dt);
    simu.set_collision_detector("fcl"); //alternatives are: dart, ode, bullet

//set the robot_dart gui options
#ifdef GRAPHIC
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>();
    simu.set_graphics(graphics);
    graphics->look_at({3.5, -2, 2.2}, {0., 0., 1.4});
    graphics->record_video("tutorial1.mp4");
#endif

    simu.add_checkerboard_floor();
    simu.add_robot(robot);

    ////////// RUN SIMULATION ////////////////////////////////////////////////////////////////////
    //set the initial robot position in the similation
    robot->set_positions(controller->q0(), controller->all_dofs()); //all_dofs contains the floating_base
    auto controllable_dofs = controller->controllable_dofs(); //get only the controllable joints that are real actuators on the real robot

    inria_wbc::controllers::SensorData sensor_data;
    Eigen::VectorXd tq_sensors = Eigen::VectorXd::Zero(robot->torques().size());

    while (!stop && !simu.graphics()->done()) {

        //Gather sensor_data from simulation
        // left foot
        if (robot->ft_foot_left().active()) {
            sensor_data["lf_torque"] = robot->ft_foot_left().torque();
            sensor_data["lf_force"] = robot->ft_foot_left().force();
        }
        else {
            sensor_data["lf_torque"] = Eigen::VectorXd::Constant(3, 1e-8);
            sensor_data["lf_force"] = Eigen::VectorXd::Constant(3, 1e-8);
        }
        // right foot
        if (robot->ft_foot_right().active()) {
            sensor_data["rf_torque"] = robot->ft_foot_right().torque();
            sensor_data["rf_force"] = robot->ft_foot_right().force();
        }
        else {
            sensor_data["rf_torque"] = Eigen::VectorXd::Constant(3, 1e-8);
            sensor_data["rf_force"] = Eigen::VectorXd::Constant(3, 1e-8);
        }
        // accelerometer
        sensor_data["imu_pos"] = robot->imu().angular_position_vec();
        sensor_data["imu_vel"] = robot->imu().angular_velocity();
        sensor_data["imu_acc"] = robot->imu().linear_acceleration();
        sensor_data["velocity"] = robot->com_velocity().tail<3>();

        sensor_data["positions"] = robot->positions(controller->controllable_dofs(false));
        sensor_data["joint_velocities"] = robot->velocities(controller->controllable_dofs(false));
        int i = 0;
        for (auto& tq_sensor : robot->torques()) {
            if (tq_sensor.second->active())
                tq_sensors(i) = tq_sensor.second->torques()(0, 0);
            else
                tq_sensors(i) = 0;
            i++;
        }
        sensor_data["joints_torque"] = tq_sensors;

        // floating base (perfect: no noise in the estimate)
        sensor_data["floating_base_position"] = inria_wbc::robot_dart::floating_base_pos(robot->positions());
        sensor_data["floating_base_velocity"] = inria_wbc::robot_dart::floating_base_vel(robot->velocities());

        behavior->update(sensor_data);
        auto q = controller->q().tail(controllable_dofs.size());//do not take floating_base

        // convert position to torque commands with SPD (it makes the hypothesis that we almost have perfect actuators)
        auto cmd = inria_wbc::robot_dart::compute_spd(robot, q, dt, controllable_dofs, false);
        robot->set_commands(cmd, controllable_dofs);
        simu.step_world();
    }

    return 0;
}
