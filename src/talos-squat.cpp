#include "tsid-sot.hpp"
#include "trajectory-handler.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>

#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>

#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

Eigen::VectorXd compute_spd(dart::dynamics::SkeletonPtr robot, Eigen::VectorXd targetpos)
{
    Eigen::VectorXd q = robot->getPositions();
    Eigen::VectorXd dq = robot->getVelocities();

    float stiffness = 10000;
    float damping = 100;
    int ndofs = robot->getNumDofs();
    Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(ndofs, ndofs);
    Eigen::MatrixXd Kd = Eigen::MatrixXd::Identity(ndofs, ndofs);

    for (std::size_t i = 0; i < robot->getNumDofs(); ++i)
    {
        Kp(i, i) = stiffness;
        Kd(i, i) = damping;
    }
    for (std::size_t i = 0; i < 6; ++i)
    {
        Kp(i, i) = 0;
        Kd(i, i) = 0;
    }

    Eigen::MatrixXd invM = (robot->getMassMatrix() + Kd * robot->getTimeStep()).inverse();
    Eigen::VectorXd p = -Kp * (q + dq * robot->getTimeStep() - targetpos);
    Eigen::VectorXd d = -Kd * dq;
    Eigen::VectorXd qddot = invM * (-robot->getCoriolisAndGravityForces() + p + d + robot->getConstraintForces());
    Eigen::VectorXd commands = p + d - Kd * qddot * robot->getTimeStep();
    return commands;
}

int main()
{

    //////////////////// INIT STACK OF TASK //////////////////////////////////////
    float dt = 0.001;
    int duration = 20 / dt;
    float arm_speed = 0.01;
    tsid_sot::talos_sot::Params params = {"../res/models/talos.urdf",
                                          "../res/models/talos_configurations.srdf",
                                          dt};
    auto talos_sot = tsid_sot::talos_sot(params, "../res/yaml/sot-squat.yaml", true);
    auto all_dofs = talos_sot.all_dofs();
    auto controllable_dofs = talos_sot.controllable_dofs();
    uint ncontrollable = controllable_dofs.size();
    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(ncontrollable);

    //////////////////// INIT DART ROBOT //////////////////////////////////////
    std::srand(std::time(NULL));
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "/home/user/rf_ws/src/talos_robot/talos_description"}};
    auto global_robot = std::make_shared<robot_dart::Robot>(params.urdf_path, packages);
    global_robot->set_position_enforced(true);
    // Set actuator types to VELOCITY motors so that they stay in position without any controller
    global_robot->set_actuator_types(dart::dynamics::Joint::FORCE);
    // First 6-DOFs should always be FORCE if robot is floating base
    for (size_t i = 0; i < 6; i++)
        global_robot->set_actuator_type(i, dart::dynamics::Joint::FORCE);
    global_robot->set_positions(talos_sot.q0(), all_dofs);

    //////////////////// INIT DART SIMULATION WORLD //////////////////////////////////////
    robot_dart::RobotDARTSimu simu(dt);
    simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::FCLCollisionDetector::create());
#ifdef GRAPHIC
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(&simu);
    simu.set_graphics(graphics);
    graphics->look_at({0., 3.5, 2.}, {0., 0., 0.25});
#endif
    simu.add_robot(global_robot);
    simu.add_checkerboard_floor();

    //////////////////// DEFINE COM REFERENCES  //////////////////////////////////////
    auto com_init = talos_sot.com_init();
    float trajectory_duration = 5;
    auto trj_generator = std::make_shared<trajectory_utils::trajectory_generator>(dt);
    KDL::Frame com_init_kdl(KDL::Rotation::Identity(), KDL::Vector(com_init(0), com_init(1), com_init(2)));
    KDL::Frame com_ref_kdl(KDL::Rotation::Identity(), KDL::Vector(com_init(0), com_init(1), com_init(2) - 0.2));
    trajectory_handler::computeTrj(trj_generator, com_init_kdl, com_ref_kdl, trajectory_duration);

    KDL::Frame reference_frame;
    Vector3 ref;

    //////////////////// PLAY SIMULATION //////////////////////////////////////
    for (int i = 0; i < duration; i++)
    {
        if (talos_sot.com_task()->position_error().norm() < 0.1)
        {
            reference_frame = trj_generator->Pos();
            ref << reference_frame.p.x(), reference_frame.p.y(), reference_frame.p.z();
            trj_generator->updateTrj();
            talos_sot.set_com_ref(ref);
        }

        talos_sot.solve();
        auto cmd = compute_spd(global_robot->skeleton(), talos_sot.q());
        global_robot->set_commands(cmd);
        simu.step_world();
    }

    global_robot.reset();
    return 0;
}
