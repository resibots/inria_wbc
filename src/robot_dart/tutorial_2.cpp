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

// easy search for an URDF
#include <utheque/utheque.hpp>

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
    inria_wbc::utilsRobotrModel::Configuration config;
    config.verbose = false;
    config.is_floating_base = true;

    inria_wbc::utils::RobotModel robot_model(utheque::path("talos/talos.urdf"), config);
    
    Eigen::VectorXd q(robot_model.nq());
    Eigen::VectorXd dq(robot_model.nv());
    
    robot_model.update(q, dq, true, true);
    
    
    return 0;
}
