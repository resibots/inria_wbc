#ifndef COLLISION_CHECK_HPP
#define COLLISION_CHECK_HPP
/* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include <yaml-cpp/yaml.h>
#include "inria_wbc/exceptions.hpp"

namespace inria_wbc::utils {

    class CollisionCheck {

    public:
        CollisionCheck(){};

        //Load the collision spheres yaml.
        //A sphere is represented by a 4d float array [x_relative, y_relative, z_relative, diameter].
        //The position of the sphere is relative to the position of the frame name in the yaml file.
        void load_collision_file(const std::string& collision_file_path, bool verbose = false);
       
        //returns true if a collision between members is detected
        bool is_colliding(const pinocchio::Model& model, const pinocchio::Data& data);

        std::map<std::string, std::vector<std::pair<Eigen::Vector3d, float>>> spherical_members() { return spherical_members_; };
        std::pair<std::pair<std::string, int>, std::pair<std::string, int>> collision_index() { return collision_index_; };

    protected:
        //It computes the absolute positions of the collisions spheres
        //The model need to already be in its posture q
        void _create_collision_spheres(const pinocchio::Model& model, const pinocchio::Data& data);

        std::map<std::string, std::vector<std::pair<std::string, std::vector<std::vector<float>>>>> collision_data_; //collision data: member name, vector of (link names - sphere data) pairs
        std::map<std::string, std::vector<std::pair<Eigen::Vector3d, float>>> spherical_members_; //position and diameter of the spheres
        std::pair<std::pair<std::string, int>, std::pair<std::string, int>> collision_index_ = {{"", -1}, {"", -1}};
        bool verbose_ = false;
    };
} // namespace inria_wbc::utils
#endif
