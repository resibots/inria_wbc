#ifndef TORQUE_SAFETY_HPP
#define TORQUE_SAFETY_HPP


#include <vector>
#include <Eigen/Dense>

class TorqueCollisionDetection
{
public:
    TorqueCollisionDetection(std::vector<std::string> joints, double threshold=1.0, int buffer_len=1);

    ~TorqueCollisionDetection() = default;

    bool safety_check(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors);

    void set_threshold(double threshold);

    Eigen::VectorXd get_discrepancy();

private:
    double _threshold;
    int _buffer_len;
    int _counter;

    Eigen::VectorXd _discrepancy;

    std::vector<Eigen::VectorXd> _buffer;

};


inline void TorqueCollisionDetection::set_threshold(double threshold)
{
    _threshold = threshold;
}


inline Eigen::VectorXd TorqueCollisionDetection::get_discrepancy()
{
    return _discrepancy;
}


#endif
