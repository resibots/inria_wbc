#include <iostream>
#include <signal.h>

#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/utils/robot_model.hpp"

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
    inria_wbc::utils::RobotModel::Configuration config;
    config.verbose = false;
    config.is_floating_base = true;

    std::string urdf_path = utheque::path("talos/talos.urdf");
    std::cout << "Loading model at: " << urdf_path << std::endl;
    
    inria_wbc::utils::RobotModel robot_model(urdf_path, config);
    std::cout << "Robot type: " << (robot_model.is_floating_base() ? "floating" : "fixed") << " base\n";
    
    //robot_model.frame_names();
    
    Eigen::VectorXd q(robot_model.nq());
    Eigen::VectorXd dq(robot_model.nv());

    q.setZero();
    dq.setZero();

    robot_model.update(q, dq, true, true);
    std::cout << "Robot com: " << robot_model.com().transpose() << std::endl;
    std::cout << "Robot bias vector (coriolis, centrifugal and gravity): " << robot_model.bias_vector().transpose() << std::endl;
    
    std::cout << "Exit" << std::endl;
    return 0;
}
