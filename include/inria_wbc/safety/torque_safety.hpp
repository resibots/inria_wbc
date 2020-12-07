#ifndef TORQUE_SAFETY_HPP
#define TORQUE_SAFETY_HPP

#include <vector>
#include <Eigen/Dense>


namespace inria_wbc {
namespace safety {


class TorqueCollisionDetection
{
public:

    typedef Eigen::Array<bool,-1,-1> ArrayXb;

    typedef struct { 
        Eigen::VectorXd operator()(const Eigen::MatrixXd& x) { return x.rowwise().mean(); } 
    } MeanFilter; 

    typedef struct { 
        Eigen::VectorXd operator()(const Eigen::MatrixXd& x) 
        { 
            int c = x.cols(), r = x.rows();

            Eigen::VectorXd res(r);
            for(int i=0; i<r; ++i)
            {
                Eigen::VectorXd v = x.row(i).eval();
                double* v_ptr = v.data();
                std::sort(v_ptr, v_ptr+c);
                
                if(c % 2 == 0)
                    res(i) = (v(c/2) + v(c/2-1)) / 2;
                else
                    res(i) = v(c/2);
            }
            return res;
        } 
    } MedianFilter;

    typedef struct { 
        Eigen::VectorXd operator()(const Eigen::MatrixXd& x) 
        { 
            Eigen::VectorXd average = x.rowwise().mean();
            return average;
        } 
    } MovingAverageFilter; 


    TorqueCollisionDetection() = default;
    TorqueCollisionDetection(int nvar, double threshold=1.0, int buffer_len=1);
    TorqueCollisionDetection(Eigen::VectorXd threshold, int buffer_len=1);
    ~TorqueCollisionDetection() = default;

    template <typename FilterFunctor = MeanFilter>
    bool check(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors, FilterFunctor filter);
    bool check(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors);

    void set_max_consecutive_invalid(unsigned int counter);

    void set_offset(const Eigen::VectorXd& offset);
    Eigen::VectorXd get_offset() const;
    void reset_offset();

    void set_threshold(double threshold);
    void set_threshold(const Eigen::VectorXd& threshold);
    Eigen::VectorXd get_threshold() const;

    Eigen::VectorXd get_discrepancy() const;

    Eigen::VectorXd get_filtered_sensors() const;

    std::vector<int> get_invalid_ids() const;


protected:

    void _compute_validity(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors);
    void _compute_validity_over_steps();


private:

    int _nvar;
    int _step_count;
    int _buffer_len;

    Eigen::MatrixXd _buffer;
    Eigen::VectorXd _offset;
    Eigen::VectorXd _threshold;
    Eigen::VectorXd _discrepancy;
    Eigen::VectorXd _filtered_sensors;
    ArrayXb _validity;

    bool _add_offset = false;

    u_int32_t _invalid_threshold;
    Eigen::MatrixXi _previous_signs;
    Eigen::VectorXi _invalid_steps_threshold;

};

template <typename FilterFunctor>
bool TorqueCollisionDetection::check(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors, FilterFunctor filter)
{
    _step_count++;

    // keep an ordered buffer for denoising and get denoised sensor value
    if(_step_count < _buffer_len)
    {
        _buffer.col(_step_count++) = sensors;
        _filtered_sensors = filter(_buffer.leftCols(_step_count));
    }
    else
    {
        _buffer.leftCols(_buffer_len-1) = _buffer.rightCols(_buffer_len-1).eval();
        _buffer.rightCols<1>() = sensors;
        _filtered_sensors = filter(_buffer);
    }

    if(_add_offset)
        _filtered_sensors = _filtered_sensors + _offset;

    _compute_validity(target, _filtered_sensors);

    if(_invalid_threshold > 0)
        _compute_validity_over_steps();

    return _validity.all();
}


} // namespace safety
} // namespace inria_wbc

#endif
