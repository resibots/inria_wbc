#include <iostream>
#include <signal.h>

#include "inria_wbc/behaviors/humanoid/squat.hpp"
#include "inria_wbc/controllers/pos_tracker.hpp"
#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/utils/timer.hpp"

volatile sig_atomic_t stop;
void user_interrupt(int signum)
{
    stop = 1;
}
//this test is made to be run on the robot to get benchmark the solver
int main(int argc, char* argv[])
{
    signal(SIGINT, user_interrupt);

    if(argc != 3)
    {
        std::clog << "You need to give in input controller and behavior configuration files' path.\n"
         "qp_timer_test path/to/controller.yaml path/to/behavior.yaml" << std::endl;
        return 1;
    }

    //Don't forget to set the content of yaml files before running
    //You might need to set base_path and urdf


    //First construct the controller from the controller config file
    std::string controller_conf_path( argv[1] ); //"../etc/talos/talos_pos_tracker_real_robot.yaml";
    auto controller_config = IWBC_CHECK(YAML::LoadFile(controller_conf_path));
    auto controller_name = IWBC_CHECK(controller_config["CONTROLLER"]["name"].as<std::string>());

    auto solver_name = IWBC_CHECK(controller_config["CONTROLLER"]["solver"].as<std::string>());

    //PosTracker do not need external sensor_data because it doesn't have stabilization
    auto controller = std::make_shared<inria_wbc::controllers::PosTracker>(controller_config);

    //Now contruct the behavior, the behavior will send reference trajectories to the controller
    std::string behavior_conf_path( argv[2] ); //"../etc/talos/squat.yaml";
    auto behavior_config = IWBC_CHECK(YAML::LoadFile(behavior_conf_path));
    auto behavior_name = IWBC_CHECK(behavior_config["BEHAVIOR"]["name"].as<std::string>());
    auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller, behavior_config);


    std::clog << "controller: " << controller_name << std::endl
        << "behavior: " << behavior_name << std::endl
        << "solver: " << solver_name << std::endl;

    //Create a timer
    inria_wbc::utils::Timer timer;

    int iteration = 0;
    while (!stop) {
        try 
        {
            timer.begin("solver");
            behavior->update();
            timer.end("solver");
            auto q = controller->q();
            timer.report(std::cout, iteration++);
        }
        catch (...)
        {
            std::clog << "Failed to solve qp problem." << std::endl;
            stop = true;
        }
    }
}
