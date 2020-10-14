#include "inria_wbc/utils/torque_safety.hpp"

#include <iostream>



TorqueCollisionDetection::TorqueCollisionDetection(std::vector<std::string> joints, double threshold, int buffer_len)
:   _threshold(threshold),
    _buffer_len(buffer_len),
    _counter(0),
    _buffer(_buffer_len)
{ }


bool TorqueCollisionDetection::safety_check(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors)
{
    _buffer[_counter++ % _buffer_len] = sensors;

    _discrepancy = (target - sensors).cwiseAbs();

    return _discrepancy.maxCoeff() < _threshold;
}


//
// RobotTorqueCollisionDetection::RobotTorqueCollisionDetection()
// :
// {}
//
// RobotTorqueCollisionDetection::safety
