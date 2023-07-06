//file to test if vive tracking is working correctly
#include "inria_wbc/utils/ViveTracking.hpp"
#include <iostream>

void print_matrix(Eigen::Matrix3d mat){
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            std::cout << mat.coeff(i,j) << "       ";
        }
        std::cout << std::endl;
    }
    
}

int main(int argc, char const *argv[])
{
    std::cout << "Hello World! " << std::endl;

    //tracker initialization
    inria::ViveTracking vive;
    vive.init("127.0.0.1","127.0.0.1");
    int cont = 1;
    int k = 0;

    for (auto const& it : vive.get())
        std::cout << it.first << std::endl;

    //printing coordinates and rotations received from the vive tracking system while the user wants to continue
    while (cont == 1)
    {
        k = (k+1)%1000;

        if (k == 999){
            vive.update();
            //for each tracker perceived
            for (const auto& it : vive.get())
            {
                Eigen::Vector3d positions = it.second.posHand;
                Eigen::Matrix3d rotations = it.second.matHand;
                std::cout << it.first << std::endl;
                // //print the positions
                // std::cout << "position x: "
                //     << positions[0] << std::endl
                //     << "position y: "
                //     << positions[1] << std::endl
                //     << "position z: "
                //     << positions[2] << std::endl << std::endl;

                // //print the rotation matrix
                // print_matrix(rotations);
                // std::cout << "is valid? " << it.second.isValid << std::endl;
                
            }
        }

    }
    

    return 0;
}