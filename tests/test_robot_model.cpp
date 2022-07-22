#include <iostream>
#include <signal.h>

#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/utils/robot_model.hpp"

// easy search for an URDF
#include <utheque/utheque.hpp>

#include "utest.hpp"


void test_kinematics_update(utest::test_t test, inria_wbc::utils::RobotModel robot_model)
{
    Eigen::VectorXd q(robot_model.nq()), dq(robot_model.nv()), ddq(robot_model.nv());
    q.setZero();
    dq.setZero();
    ddq.setZero();

    robot_model.update(q, dq, false, false);
    UTEST_CHECK_EXCEPTION(test, robot_model.inertia_matrix());
    UTEST_CHECK_EXCEPTION(test, robot_model.bias_vector());
    UTEST_CHECK_EXCEPTION(test, robot_model.gravity_vector());
    UTEST_CHECK_EXCEPTION(test, robot_model.jacobian("arm_right_1_joint"));
    UTEST_CHECK_EXCEPTION(test, robot_model.hessian("arm_right_1_joint"));
}

void test_dynamics_update(utest::test_t test, inria_wbc::utils::RobotModel robot_model)
{
    Eigen::VectorXd q(robot_model.nq()), dq(robot_model.nv()), ddq(robot_model.nv());
    q.setZero();
    dq.setZero();
    ddq.setZero();

    robot_model.update(q, dq, ddq, true, false);
    UTEST_CHECK_NO_EXCEPTION(test, robot_model.inertia_matrix());
    UTEST_CHECK_NO_EXCEPTION(test, robot_model.bias_vector());
    UTEST_CHECK_NO_EXCEPTION(test, robot_model.gravity_vector());
    UTEST_CHECK_EXCEPTION(test, robot_model.jacobian("arm_right_1_joint"));
    UTEST_CHECK_EXCEPTION(test, robot_model.hessian("arm_right_1_joint"));

}

void test_jacobians_update(utest::test_t test, inria_wbc::utils::RobotModel robot_model)
{
    Eigen::VectorXd q(robot_model.nq()), dq(robot_model.nv()), ddq(robot_model.nv());
    q.setZero();
    dq.setZero();
    ddq.setZero();

    robot_model.update(q, dq, ddq, false, true);
    UTEST_CHECK_EXCEPTION(test, robot_model.inertia_matrix());
    UTEST_CHECK_EXCEPTION(test, robot_model.bias_vector());
    UTEST_CHECK_EXCEPTION(test, robot_model.gravity_vector());
    UTEST_CHECK_NO_EXCEPTION(test, robot_model.jacobian("arm_right_1_joint"));
    UTEST_CHECK_NO_EXCEPTION(test, robot_model.hessian("arm_right_1_joint"));
}

void test_compute_rnea(utest::test_t test, inria_wbc::utils::RobotModel robot_model)
{
    Eigen::VectorXd q(robot_model.nq()), dq(robot_model.nv()), ddq(robot_model.nv());
    q.setZero();
    dq.setZero();
    ddq.setZero();

    robot_model.update(q, dq, ddq, true, true);

    std::map<std::string, Eigen::MatrixXd> sensor_data;
    Eigen::Vector3d data = Eigen::Vector3d::Zero();
    sensor_data["lf_torque"] = data;
    sensor_data["rf_torque"] = data;
    data(2) = 442.0;
    sensor_data["lf_force"] = data;
    sensor_data["rf_force"] = data;

    UTEST_CHECK_NO_EXCEPTION(test, robot_model.compute_rnea_double_support(sensor_data, q, dq ,ddq, "true","leg_left_6_joint", "leg_right_6_joint","left_sole_link","right_sole_link"));
}


int main(int argc, char** argv)
{
    utest::TestSuite test_suite;
    srand(time(NULL));

    // basic initialization
    inria_wbc::utils::RobotModel::Configuration config;
    config.verbose = false;
    config.is_floating_base = true;

    std::string urdf_path = utheque::path("talos/talos.urdf");
    std::cout << "Loading model at: " << urdf_path << std::endl;
    
    inria_wbc::utils::RobotModel robot_model(urdf_path, config);
    std::cout << "Robot type: " << (robot_model.is_floating_base() ? "floating" : "fixed") << " base\n";
    
    std::cout << "\nRobot has joint: " << std::endl;
    for(const auto& j : robot_model.joint_names())
        std::cout << j << std::endl;
    std::cout << "\nRobot has frames: " << std::endl;
    for(const auto& f : robot_model.frame_names())
        std::cout << f << std::endl;
    
    std::cout << "\nUpdate robot in zero configuration"<< std::endl;
    Eigen::VectorXd q(robot_model.nq());
    Eigen::VectorXd dq(robot_model.nv());
    Eigen::VectorXd ddq(robot_model.nv());
    q.setZero();
    dq.setZero();
    ddq.setZero();
    robot_model.update(q, dq, true, true);

    std::cout << "\nRobot com: " << robot_model.com().transpose() << std::endl;
    std::cout << "Robot bias vector (coriolis, centrifugal and gravity): " << robot_model.bias_vector().transpose() << std::endl;

    auto kin_update_test = utest::make_test("robot_model kinematics update");
    UTEST_REGISTER(test_suite, kin_update_test, test_kinematics_update(kin_update_test, robot_model));

    auto dyn_update_test = utest::make_test("robot_model kinematics update");
    UTEST_REGISTER(test_suite, dyn_update_test, test_dynamics_update(dyn_update_test, robot_model));

    auto jac_update_test = utest::make_test("robot_model kinematics update");
    UTEST_REGISTER(test_suite, jac_update_test, test_jacobians_update(jac_update_test, robot_model));

    auto test_rnea = utest::make_test("test rnea computation");
    UTEST_REGISTER(test_suite, test_rnea, test_compute_rnea(test_rnea, robot_model));

    test_suite.run();
    utest::write_report(test_suite, std::cout, true);
    
    return 0;
}
