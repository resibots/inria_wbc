#include "inria_wbc/utils/talos_scaled_tracking.hpp"

#include <limbo/limbo.hpp>

#include "../utils/talos_scaled_tracking.cpp"

//defining needs to use limbo library
using namespace limbo;

struct Params {
    struct bayes_opt_boptimizer : public defaults::bayes_opt_boptimizer {
    };

    struct opt_gridsearch : public defaults::opt_gridsearch {
        BO_PARAM(int, bins, 20);
    };

    // enable / disable the writing of the result files
    struct bayes_opt_bobase : public defaults::bayes_opt_bobase {
        BO_PARAM(int, stats_enabled, true);
    };

    // no noise
    struct kernel : public defaults::kernel {
        BO_PARAM(double, noise, 1e-10);
    };

    struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {
    };

    // we use 10 random samples to initialize the algorithm
    struct init_randomsampling {
        BO_PARAM(int, samples, 2);
    };

    // we stop after 10 iterations
    struct stop_maxiterations {
        BO_PARAM(int, iterations, 10);
    };

    struct stop_mintolerance {
        BO_PARAM(double, tolerance, -0.1);
    };

    struct opt_rprop : public defaults::opt_rprop {
    };

    struct opt_parallelrepeater : public defaults::opt_parallelrepeater {
    };

    // we use the default parameters for acqui_ucb
    struct acqui_ucb : public defaults::acqui_ucb {
    };

    struct acqui_ei {
        BO_PARAM(double, jitter, 0.0);
    };
};

template <typename Params>
struct MeanFWModel : mean::BaseMean<Params> {
    MeanFWModel(size_t dim_out = 1) {}

    template <typename GP>
    Eigen::VectorXd operator()(const Eigen::VectorXd& x, const GP&) const
    {
        return tools::make_vector(-40);
    }
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
    BO_PARAM(size_t, dim_in, 3);
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
        Eigen::Matrix3d K = Eigen::DiagonalMatrix<double,Eigen::Dynamic>(Eigen::Vector3d(x(0),x(1),x(2)));
        // we return a 1-dimensional vector
        double y = -talos_scaled_tracking(argcc,argvv[0],K);
        return tools::make_vector(y);
    }
};

int main(int argc,char* argv[])
{
    //initializing the GP
    using kernel_t = kernel::SquaredExpARD<Params>;
    using mean_t = MeanFWModel<Params>;
    using gp_opt_t = model::gp::KernelLFOpt<Params>;
    using gp_t = model::GP<Params,kernel_t,mean_t,gp_opt_t>;

    //Acquisition aliases
    using acqui_t = acqui::EI<Params,gp_t>;
    using acqui_opt_t = opt::GridSearch<Params>;

    //Initialization alias
    using init_t = init::RandomSampling<Params>;

    //stopping criteria alias
    using stop_t = boost::fusion::vector<stop::MaxIterations<Params>,MinTolerance<Params>>;

    // Statistics alias
    using stat_t = boost::fusion::vector<limbo::stat::ConsoleSummary<Params>,
                                         limbo::stat::Samples<Params>, limbo::stat::Observations<Params>,
                                         limbo::stat::AggregatedObservations<Params>, limbo::stat::GPAcquisitions<Params>,
                                         limbo::stat::BestAggregatedObservations<Params>, limbo::stat::GPKernelHParams<Params>>;
                  
    
    //generate optimizer
    bayes_opt::BOptimizer<Params, modelfun<gp_t>, acquifun<acqui_t>, acquiopt<acqui_opt_t>, initfun<init_t>, statsfun<stat_t>, stopcrit<stop_t>> boptimizer;
    // Instantiate aggregator
    DistanceToTarget<Params> aggregator({1, 1,1});

    //solve the optimization problem
    boptimizer.optimize(eval_func<Params>(argc,argv), aggregator);
    std::cout << "New target!" << std::endl;
    aggregator = DistanceToTarget<Params>({1.5, 1,1});
    // Do not forget to pass `false` as the last parameter in `optimize`,
    // so you do not reset the data in boptimizer
    // i.e. keep all the previous data points in the Gaussian Process
    boptimizer.optimize(eval_func<Params>(argc,argv), aggregator, false);

    return 0;
}
