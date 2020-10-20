#ifndef TORQUE_SAFETY_HPP
#define TORQUE_SAFETY_HPP


#include <vector>
#include <Eigen/Dense>

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


    TorqueCollisionDetection(std::vector<std::string> joints, double threshold=1.0, int buffer_len=1);
    TorqueCollisionDetection(std::vector<std::string> joints, Eigen::VectorXd threshold, int buffer_len=1);
    ~TorqueCollisionDetection() = default;

    template <typename FilterFunctor = MeanFilter>
    bool check(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors, FilterFunctor filter);
    bool check(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors);

    void set_threshold(double threshold);
    void set_threshold(const Eigen::VectorXd& threshold);

    Eigen::VectorXd get_threshold() const;

    Eigen::VectorXd get_discrepancy() const;

    Eigen::VectorXd get_filtered_sensors() const;

    std::vector<int> get_invalid_ids() const;


private:

    int _nvar;
    int _buffer_len;
    int _buffer_count;

    Eigen::MatrixXd _buffer;
    Eigen::VectorXd _threshold;
    Eigen::VectorXd _discrepancy;
    Eigen::VectorXd _filtered_sensors;
    ArrayXb _validity;

};



template <typename FilterFunctor>
bool TorqueCollisionDetection::check(const Eigen::VectorXd& target, const Eigen::VectorXd& sensors, FilterFunctor filter)
{
    // keep an ordered buffer for denoising and get denoised sensor value
    if(_buffer_count < _buffer_len)
    {
        _buffer.col(_buffer_count++) = sensors;
        _filtered_sensors = filter(_buffer.leftCols(_buffer_count));
    }
    else
    {
        _buffer.leftCols(_buffer_len-1) = _buffer.rightCols(_buffer_len-1).eval();
        _buffer.rightCols<1>() = sensors;
        _filtered_sensors = filter(_buffer);
    }

    // compare with the target
    _discrepancy = (target - _filtered_sensors).cwiseAbs();

    _validity = _discrepancy.array() < _threshold.array();

    return _validity.all();
}


#endif
