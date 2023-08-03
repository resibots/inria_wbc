#include "inria_wbc/utils/talos_scaled_tracking.hpp"

#include "../utils/talos_scaled_tracking.cpp"

int main(int argc, char* argv[])
{
    double note = talos_scaled_tracking(argc,argv,Eigen::Matrix3d::Identity());
    std::cout << "note obtained: " << note << std::endl;
    return 0;
}
