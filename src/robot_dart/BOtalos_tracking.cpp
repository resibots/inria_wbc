#include "inria_wbc/utils/talos_scaled_tracking.hpp"

#include <limbo/limbo.hpp>

#include "../utils/talos_scaled_tracking.cpp"

//defining needs to use limbo library
using namespace limbo;

struct Params {
    struct bayes_opt_boptimizer : public defaults::bayes_opt_boptimizer {
    };

    struct opt_gridsearch : public defaults::opt_gridsearch {
        BO_PARAM(int, bins, 200);
    };

    // enable / disable the writing of the result files
    struct bayes_opt_bobase : public defaults::bayes_opt_bobase {
        BO_PARAM(int, stats_enabled, true);
    };

    // no noise
    struct kernel : public defaults::kernel {
        BO_PARAM(double, noise, 1e-5);
    };

    struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {
    };

    struct kernel_exp : public defaults::kernel_exp {
        BO_PARAM(double, sigma_sq, 2e-1);
        BO_PARAM(double, l, 5e-2);
    };

    struct mean_constant : public defaults::mean_constant {
        BO_PARAM(double,constant,-0.2);
    };

    // we use 10 random samples to initialize the algorithm
    struct init_randomsampling {
        BO_PARAM(int, samples, 3);
    };

    // we stop after 10 iterations
    struct stop_maxiterations {
        BO_PARAM(int, iterations, 5);
    };

    // struct stop_mintolerance {
    //     BO_PARAM(double, tolerance, -0.00001);
    // };

    struct opt_rprop : public defaults::opt_rprop {
    };

    struct opt_parallelrepeater : public defaults::opt_parallelrepeater {
    };

    // we use the default parameters for acqui_ucb
    struct acqui_ucb : public defaults::acqui_ucb {
        BO_PARAM(double, alpha, 1);
    };

    struct acqui_ei {
        BO_PARAM(double, jitter, 0.0);
    };

    struct stat_gp {
        BO_PARAM(int, bins, 1000);
    };
};

//defining min tolerance and distance to target to create a stopping criterion
template <typename Params>
struct MinTolerance {
    MinTolerance() {}

    template <typename BO, typename AggregatorFunction>
    bool operator()(const BO& bo, const AggregatorFunction& afun)
    {
        return afun(bo.best_observation(afun)) > Params::stop_mintolerance::tolerance();
    }
};

template <typename Params>
struct DistanceToTarget {
    using result_type = double;
    DistanceToTarget(const Eigen::Vector3d& target) : _target(target) {}

    double operator()(const Eigen::VectorXd& x) const
    {
        return -(x - _target).norm();
    }

protected:
    Eigen::Vector3d _target;
};

//eval function
template <typename Params>
struct eval_func {
    // number of input dimension (x.size())
    BO_PARAM(size_t, dim_in, 1);
    // number of dimensions of the result (res.size())
    BO_PARAM(size_t, dim_out, 1);

    //to get arguments when calling the program
    int argcc;
    std::vector<char**> argvv;
    eval_func(int argc,char* argv[]) {
        argcc = argc;
        argvv.push_back(argv);
    }

    // the function to be optimized
    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
    {
        Eigen::Matrix3d K = Eigen::DiagonalMatrix<double,Eigen::Dynamic>(Eigen::Vector3d(1,1,2*x(0)));
        // we return a 1-dimensional vector
        double y = -talos_scaled_tracking(argcc,argvv[0],K);
        //double y = -x(0)*x(0) + 3*x(0) + 1;
        return tools::make_vector(y);
    }
};

int main(int argc,char* argv[])
{
    //initializing the GP
    using kernel_t = kernel::Exp<Params>;
    using mean_t = mean::Constant<Params>;
    using gp_t = model::GP<Params,kernel_t,mean_t>;

    //Acquisition aliases
    using acqui_t = acqui::UCB<Params,gp_t>;
    using acqui_opt_t = opt::GridSearch<Params>;

    //Initialization alias
    using init_t = init::RandomSampling<Params>;

    //stopping criteria alias
    using stop_t = boost::fusion::vector<stop::MaxIterations<Params>>;

    // Statistics alias
    using stat_t = boost::fusion::vector<limbo::stat::ConsoleSummary<Params>,limbo::stat::GP<Params>,
                                         limbo::stat::Samples<Params>, limbo::stat::Observations<Params>,
                                         limbo::stat::GPAcquisitions<Params>, limbo::stat::GPKernelHParams<Params>>;
                  
    
    //generate optimizer
    bayes_opt::BOptimizer<Params, modelfun<gp_t>, acquifun<acqui_t>, acquiopt<acqui_opt_t>, initfun<init_t>, statsfun<stat_t>, stopcrit<stop_t>> boptimizer;

    //solve the optimization problem
    boptimizer.optimize(eval_func<Params>(argc,argv));

    return 0;
}
