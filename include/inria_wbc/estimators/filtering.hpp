#ifndef IWBC_ESTIMATOR_FILTERING_HPP
#define IWBC_ESTIMATOR_FILTERING_HPP

#include <vector>
#include <memory>
#include <Eigen/Dense>

#include "inria_wbc/exceptions.hpp"


namespace inria_wbc {
namespace estimators {


class Filter
{
public:
    typedef std::shared_ptr<Filter> Ptr;

    Filter() = default;
    
    Filter(int nvar, int wsize)
    : _cnt(0), _nvar(nvar), _wsize(wsize)   
    {
        reset();
    }
    virtual ~Filter(){}

    // virtual void init(int nvar, int wsize) {  _nvar = nvar; _wsize = wsize; reset(); };
    // virtual void set_num_var(int nvar) { _nvar = nvar; reset(); }
    // virtual void set_window_size(int wsize) { _wsize = wsize; reset();  }

    int get_num_var() const { return _nvar; }
    int get_window_size() const { return _wsize; }
    bool data_ready() const { return (_cnt >=  _wsize);}

    Eigen::VectorXd filter(const Eigen::VectorXd& sample)
    {
        IWBC_ASSERT(sample.size() == _nvar, "Size of sample differs from filter get_num_var()!");

        if(_cnt < _wsize)
        {
            _buffer.col(_cnt++) = sample;
            _filter_impl(_buffer.leftCols(_cnt));
        }
        else
        {
            if(_wsize > 1)
                _buffer.leftCols(_wsize-1) = _buffer.rightCols(_wsize-1).eval();
            
            _buffer.rightCols<1>() = sample;
            _filter_impl(_buffer);
        }

        return _filtered;
    };

    
    virtual void reset()
    {
        _buffer.resize(_nvar, _wsize);
        _buffer.setZero();

        _filtered.resize(_nvar);
        _filtered.setZero();

        _cnt = 0;
    }

protected:

    // use buffer to compute the _filtered value
    virtual void _filter_impl(const Eigen::MatrixXd& window) = 0;

    int _cnt;
    int _nvar;
    int _wsize;
    Eigen::MatrixXd _buffer;
    Eigen::VectorXd _filtered;
};

class MedianFilter : public Filter
{
public:

    typedef std::shared_ptr<MedianFilter> Ptr;

    MedianFilter() = default;
    MedianFilter(int nvar, int wsize) : Filter(nvar, wsize) { }

    void _filter_impl(const Eigen::MatrixXd& window) override
    {
        int c = window.cols(), r = window.rows();

        Eigen::VectorXd res(r);
        for(int i=0; i<r; ++i)
        {
            Eigen::VectorXd v = window.row(i).eval();
            double* v_ptr = v.data();
            std::sort(v_ptr, v_ptr+c);
            
            if(c % 2 == 0)
                res(i) = (v(c/2) + v(c/2-1)) / 2;
            else
                res(i) = v(c/2);
        }
        _filtered = res;
    }
};


class MovingAverageFilter : public Filter
{
public:

    typedef std::shared_ptr<MovingAverageFilter> Ptr;

    MovingAverageFilter() = default;
    MovingAverageFilter(int nvar, int wsize) : Filter(nvar, wsize) { }

    void _filter_impl(const Eigen::MatrixXd& window) override
    {
        _filtered = window.rowwise().mean();
    }
};


class CompositeFilter : public Filter
{

public:
    CompositeFilter() : Filter(1, 1) { }

    void add_filter(const Filter::Ptr filter)
    {
        if(_filter_list.empty())
        {        
            _nvar = filter->get_num_var();
            reset();
        }
        
        IWBC_ASSERT(this->get_num_var() == filter->get_num_var(), "Added filter which operates on a different number of vars!");
        _filter_list.push_back(filter);

    }

    //void init(int nvar, int wsize) override { IWBC_ASSERT("method not implemented for composite filter."); }
    //void set_num_var(int nvar) override { IWBC_ASSERT("method implemented for composite filter."); }
    //void set_window_size(int wsize) override { IWBC_ASSERT("method implemented for composite filter.");  }

protected:

    void _filter_impl(const Eigen::MatrixXd& window) override
    {
        _filtered = window.rightCols(1);

        for(auto flt_ptr : _filter_list)
            _filtered = flt_ptr->filter(_filtered);
    }

    std::vector<Filter::Ptr> _filter_list;
};


} // enf namespace estimators
} // end namespace inria_wbc

#endif // IWBC_ESTIMATOR_FILTERING_HPP