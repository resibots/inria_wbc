#include "inria_wbc/safety/torque_collision_detection.hpp"
#include "inria_wbc/exceptions.hpp"

#include <iostream>

namespace inria_wbc {
namespace safety {


TorqueCollisionDetection::TorqueCollisionDetection(int nvar, double threshold)
:   _nvar(nvar),
    _step_count(0),
    _threshold(Eigen::VectorXd::Constant(_nvar, threshold)),
    _discrepancy(_nvar),
    _targets(_nvar),
    _sensors(_nvar),
    _filtered_sensors(_nvar)
{ 
    this->set_max_consecutive_invalid(0);
}


TorqueCollisionDetection::TorqueCollisionDetection(Eigen::VectorXd threshold)
:   _nvar(threshold.size()),
    _step_count(0),
    _threshold(threshold),
    _discrepancy(_nvar),
    _targets(_nvar),
    _sensors(_nvar),
    _filtered_sensors(_nvar)
{ 
    this->set_max_consecutive_invalid(0);
}


void TorqueCollisionDetection::reset()
{
    _step_count = 0;
    _previous_signs.setZero();

    if(_filter_ptr)
        _filter_ptr->reset();
}


bool TorqueCollisionDetection::check(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors)
{
    ++_step_count;

    _targets = target;
    _sensors = sensors;

    _filtered_sensors = _filter_ptr ? _filter_ptr->filter(sensors) : sensors;

    if(_add_offset)
        _filtered_sensors = _filtered_sensors + _offset;

    _compute_validity(target, _filtered_sensors);

    if(_invalid_threshold > 0)
        _compute_validity_over_steps();

    return _validity.all();
}



void TorqueCollisionDetection::set_threshold(double threshold)
{
    _threshold = Eigen::VectorXd::Constant(_nvar, threshold);
}


void TorqueCollisionDetection::set_threshold(const Eigen::VectorXd& threshold)
{
    IWBC_ASSERT("Wrong size for the threshold vector.", _nvar == threshold.size());
    _threshold = threshold;
}

void TorqueCollisionDetection::set_offset(const Eigen::VectorXd& offset)
{
    IWBC_ASSERT(offset.size() == _nvar, "Offset dimensionality error");
    _offset = offset;
    _add_offset = true;
}


Eigen::VectorXd TorqueCollisionDetection::get_offset() const
{
    return _offset;
}

void TorqueCollisionDetection::remove_offset()
{
    _offset.setZero();
    _add_offset = false;
}

void TorqueCollisionDetection::set_filter(estimators::Filter::Ptr filter_ptr)
{
    _filter_ptr = filter_ptr;
}

estimators::Filter::Ptr TorqueCollisionDetection::get_filter()
{
    return _filter_ptr;
}

void TorqueCollisionDetection::remove_filter()
{
    _filter_ptr = nullptr;
}


Eigen::VectorXd TorqueCollisionDetection::get_discrepancy() const
{
    return _discrepancy;
}

Eigen::VectorXi TorqueCollisionDetection::get_validity() const
{
    return _validity.matrix().cast<int>();;
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


void TorqueCollisionDetection::set_max_consecutive_invalid(unsigned int counter)
{
    _invalid_threshold = counter + 1;

    _previous_signs.resize(_nvar, _invalid_threshold);
    _previous_signs.setZero();

    _invalid_steps_threshold.resize(_nvar);
    _invalid_steps_threshold.setConstant(_invalid_threshold);
}



void TorqueCollisionDetection::_compute_validity(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors)
{
    _discrepancy = target - sensors;
    _validity = _discrepancy.cwiseAbs().array() < _threshold.array();
}


void TorqueCollisionDetection::_compute_validity_over_steps()
{
    Eigen::VectorXi sign = _discrepancy.array().sign().matrix().cast<int>();
    Eigen::VectorXi nval = Eigen::VectorXi::Ones(_nvar) - _validity.matrix().cast<int>();
    _previous_signs.col(_step_count % _invalid_threshold) = nval.cwiseProduct(sign);

    Eigen::VectorXi cumulated_signs = _previous_signs.rowwise().sum().cwiseAbs();

    _validity = cumulated_signs.array() < _invalid_steps_threshold.array();
}


} // namespace safety
} // namespace inria_wbc