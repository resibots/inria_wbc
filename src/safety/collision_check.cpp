
#include "inria_wbc/safety/collision_check.hpp"

namespace inria_wbc::utils {

    void CollisionCheck::load_collision_file(const std::string& collision_file_path, bool verbose)
    {
        verbose_ = verbose;

        if (verbose_)
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

    bool CollisionCheck::is_colliding(const pinocchio::Model& model, const pinocchio::Data& data)
    {
        _create_collision_spheres(model, data);

        if (spherical_members_.empty())
            IWBC_ERROR("CollisionCheck::_is_colliding : the map is empty, call _create_collision_spheres first");
        for (auto& it : spherical_members_) {
            for (auto& it2 : spherical_members_) {
                if (it.first != it2.first) {
                    for (int i = 0; i < it.second.size(); i++) {
                        auto sphere = it.second[i];
                        for (int j = 0; j < it2.second.size(); j++) {
                            auto sphere2 = it2.second[j];
                            if ((sphere2.first - sphere.first).norm() < (sphere2.second / 2 + sphere.second / 2)) { //check that the distance btwn two spheres are more than the sum of the spheres radius
                                collision_index_.first = std::make_pair(it.first, i);
                                collision_index_.second = std::make_pair(it2.first, j);
                                if (verbose_) {
                                    std::cout << "Collision detected" << std::endl;
                                    std::cout << it.first << " " << i << std::endl;
                                    std::cout << it2.first << " " << j << std::endl;
                                }
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    void CollisionCheck::_create_collision_spheres(const pinocchio::Model& model, const pinocchio::Data& data)
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

} // namespace inria_wbc::utils

