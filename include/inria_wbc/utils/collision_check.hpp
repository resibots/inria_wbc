#ifndef COLLISION_CHECK_HPP
#define COLLISION_CHECK_HPP
/* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

namespace inria_wbc::utils {

    class CollisionCheck {

    public:
        CollisionCheck(){};

        //Load the collision spheres yaml.
        //A sphere is represented by a 4d float array [x_relative, y_relative, z_relative, diameter].
        //The position of the sphere is relative to the position of the frame name in the yaml file.
        void load_collision_file(const std::string& collision_file_path, bool verbose = false)
        {
            if (verbose)
                std::cout << "Parsing virtual frame file:" << collision_file_path << std::endl;
            YAML::Node node = IWBC_CHECK(YAML::LoadFile(collision_file_path));

            collision_data_.clear(); //data is joint_names and number of sphere per links
            for (auto it = node["members"].begin(); it != node["members"].end(); ++it) {
                auto member_name = IWBC_CHECK(it->first.as<std::string>());
                std::vector<std::pair<std::string, std::vector<std::vector<float>>>> vec;
                for (auto it2 = node["members"][member_name].begin(); it2 != node["members"][member_name].end(); ++it2) {
                    auto link_name = IWBC_CHECK(it2->first.as<std::string>());
                    auto spheres = IWBC_CHECK(it2->second.as<std::vector<std::vector<float>>>());
                    vec.push_back(std::make_pair(link_name, spheres));
                }
                collision_data_[member_name] = vec;
            }
        }

        //returns true if a collision between members is detected
        bool is_colliding(const pinocchio::Model& model, const pinocchio::Data& data)
        {
            _create_collision_spheres(model, data);

            if (spherical_members_.empty())
                IWBC_ERROR("CollisionCheck::_is_colliding : the map is empty, call _create_collision_spheres first");
            for (auto& it : spherical_members_) {
                for (auto& it2 : spherical_members_) {
                    if (it.first != it2.first) {
                        for (auto& sphere : it.second) {
                            for (auto& sphere2 : it2.second) {
                                if ((sphere2.first - sphere.first).norm() < (sphere2.second / 2 + sphere.second / 2)) //check that the distance btwn two spheres are more than the sum of the spheres radius
                                    return true;
                            }
                        }
                    }
                }
            }
            return false;
        }

        inline std::map<std::string, std::vector<std::pair<Eigen::Vector3d, float>>> spherical_members() { return spherical_members_; };

    protected:
        //It computes the absolute positions of the collisions spheres
        //The model need to already be in its posture q
        void _create_collision_spheres(const pinocchio::Model& model, const pinocchio::Data& data)
        {
            if (collision_data_.empty())
                IWBC_ERROR("CollisionCheck::_create_collision_spheres : the map is empty, call load_collision_file first");

            spherical_members_.clear();
            float epsilon = 0.05;
            for (auto& it : collision_data_) {
                std::vector<std::pair<Eigen::Vector3d, float>> spherical_member;
                for (auto& member : it.second) {
                    for (auto& frame : model.frames) {
                        if (frame.name == member.first) {
                            for (auto& sphere : member.second) {
                                if (sphere.size() != 4)
                                    IWBC_ERROR("collisions yaml : sphere data should be an float array of dim 4");
                                if (!model.existFrame(frame.name))
                                    IWBC_ERROR("collisions yaml : frame name", frame.name, "doesn't exists");
                                auto index = model.getFrameId(frame.name);
                                Eigen::Vector3d sphere_position_relative = {sphere[0], sphere[1], sphere[2]};
                                pinocchio::SE3 sphere_rel = pinocchio::SE3::Identity();
                                sphere_rel.translation() = sphere_position_relative;
                                auto sphere_position = data.oMi[model.frames[index].parent].act(sphere_rel);
                                spherical_member.push_back(std::make_pair(sphere_position.translation(), sphere[3]));
                            }
                        }
                    }
                }
                spherical_members_[it.first] = spherical_member;
            }
        }

        std::map<std::string, std::vector<std::pair<std::string, std::vector<std::vector<float>>>>> collision_data_; //collision data: member name, vector of (link names - sphere data) pairs
        std::map<std::string, std::vector<std::pair<Eigen::Vector3d, float>>> spherical_members_; //position and diameter of the spheres
    };
} // namespace inria_wbc::utils
#endif
