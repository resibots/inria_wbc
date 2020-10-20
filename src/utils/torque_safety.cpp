#include "inria_wbc/utils/torque_safety.hpp"

#include <iostream>


TorqueCollisionDetection::TorqueCollisionDetection(std::vector<std::string> joints, double threshold, int buffer_len)
:   _nvar(joints.size()),
    _buffer_len(buffer_len),
    _buffer_count(0),
    _buffer(_nvar, _buffer_len),
    _threshold(Eigen::VectorXd::Constant(_nvar, threshold)),
    _discrepancy(_nvar),
    _filtered_sensors(_nvar)
{ }


TorqueCollisionDetection::TorqueCollisionDetection(std::vector<std::string> joints, Eigen::VectorXd threshold, int buffer_len)
:   _nvar(joints.size()),
    _buffer_len(buffer_len),
    _buffer_count(0),
    _buffer(_nvar, _buffer_len),
    _threshold(threshold),
    _discrepancy(_nvar),
    _filtered_sensors(_nvar)
{ }


bool TorqueCollisionDetection::check(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors)
{
    _discrepancy = (target - sensors).cwiseAbs();

    _validity = _discrepancy.array() < _threshold.array();

    return _validity.all();
}


void TorqueCollisionDetection::set_threshold(double threshold)
{
    _threshold = Eigen::VectorXd::Constant(_nvar, threshold);
}


void TorqueCollisionDetection::set_threshold(const Eigen::VectorXd& threshold)
{
    assert("Wrong size for the threshold vector." && _nvar == threshold.size());
    _threshold = threshold;
}


Eigen::VectorXd TorqueCollisionDetection::get_discrepancy() const
{
    return _discrepancy;
}


Eigen::VectorXd TorqueCollisionDetection::get_filtered_sensors() const
{
    return _filtered_sensors;
}


std::vector<int> TorqueCollisionDetection::get_invalid_ids() const
{
    std::vector<int> ids;
    for(int i=0; i<_validity.size(); ++i)
        if(!_validity(i))
            ids.push_back(i);
    return ids;
}
