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
{ 
    this->set_ignore_count(1);
}


TorqueCollisionDetection::TorqueCollisionDetection(std::vector<std::string> joints, Eigen::VectorXd threshold, int buffer_len)
:   _nvar(joints.size()),
    _buffer_len(buffer_len),
    _buffer_count(0),
    _buffer(_nvar, _buffer_len),
    _threshold(threshold),
    _discrepancy(_nvar),
    _filtered_sensors(_nvar)
{ 
    this->set_ignore_count(1);
}


bool TorqueCollisionDetection::check(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors)
{
    _compute_validity(target, sensors);
    _compute_validity_over_steps();

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


void TorqueCollisionDetection::set_ignore_count(unsigned int counter)
{
    _ignore_steps = counter;

    _previous_signs.resize(_nvar, _ignore_steps);
    _previous_signs.setZero();

    _invalid_steps_threshold.resize(_nvar);
    _invalid_steps_threshold.setConstant(_ignore_steps);
}



void TorqueCollisionDetection::_compute_validity(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors)
{
    _discrepancy = target - sensors;
    _validity = _discrepancy.cwiseAbs().array() < _threshold.array();
}


void TorqueCollisionDetection::compute_validity_over_steps()
{
    Eigen::VectorXi sign = _discrepancy.array().sign().matrix().cast<int>();
    _previous_signs.col(_buffer_count++ % _ignore_steps) = _validity.matrix().cast<int>().eval().cwiseProduct(sign);

    Eigen::VectorXi cumulated_signs = _previous_signs.colwise().sum().cwiseAbs();

    return cumulated_signs.array() < _invalid_steps_threshold.array();
}