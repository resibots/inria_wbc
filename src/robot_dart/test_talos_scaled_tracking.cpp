#include "inria_wbc/utils/talos_scaled_tracking.hpp"

#include "../utils/talos_scaled_tracking.cpp"

int main(int argc, char* argv[])
{
    const double s = 1;
    const double v_sat = 2;
    const double thr = 2;
    double note = talos_scaled_tracking(argc,argv,Eigen::Matrix3d::Identity(),s,v_sat,thr);
    std::cout << "note obtained: " << note << std::endl;
    return 0;
}
