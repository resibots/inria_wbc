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

    //Don't forget to set the content of yaml files before running
    //You might need to set base_path and urdf

    //First construct the controller from the controller config file
    std::string controller_conf_path = "../etc/talos/talos_pos_tracker_real_robot.yaml";
    auto controller_yaml = IWBC_CHECK(YAML::LoadFile(controller_conf_path));

    //PosTracker do not need external sensor_data because it doesn't have stabilization
    auto controller = std::make_shared<inria_wbc::controllers::PosTracker>(controller_yaml);

    //Now contruct the behavior, the behavior will send reference trajectories to the controller
    std::string behavior_conf_path = "../etc/talos/squat.yaml";
    auto behavior_yaml = IWBC_CHECK(YAML::LoadFile(behavior_conf_path));
    auto behavior = std::make_shared<inria_wbc::behaviors::humanoid::Squat>(controller, behavior_yaml);

    //Create a timer
    inria_wbc::utils::Timer timer;

    int iteration = 0;
    while (!stop) {
        timer.begin("solver");
        behavior->update();
        timer.end("solver");
        auto q = controller->q();
        timer.report(std::cout, iteration++);
    }
}
