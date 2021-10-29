#ifndef IWBC_TRAJ_LOADER__
#define IWBC_TRAJ_LOADER__

#include <fstream>
#include <iostream>
#include <pinocchio/spatial/se3.hpp>
#include <string>
#include <tsid/math/fwd.hpp>
#include <unordered_map>
#include <vector>

namespace inria_wbc {
    namespace trajs {
        class Loader {
        public:
            Loader(const std::string& yaml_path);
            const std::vector<std::string>& ref_names() const { return _ref_names; }
            const std::vector<std::string>& ref_names_vec() const { return _ref_names_vec; }
            // TODO: convert to SE3 (will not compile...)
            const pinocchio::SE3& task_ref(const std::string& name, int k) const { return _task_refs.at(name)[k]; }
            const Eigen::VectorXd& task_ref_vec(const std::string& name, int k) const { return _task_refs_vec.at(name)[k]; }
            
            tsid::math::Vector3 com_ref(int k) const { return _com_refs[k].head(3); }
            bool has_com_refs() const { return !_com_refs.empty(); }
            // number of points in the trajectories
            size_t size() const { return _task_refs.begin()->second.size(); }
            size_t size_vec() const { return _task_refs_vec.begin()->second.size(); }

        protected:
            std::vector<std::string> _ref_names;
            std::vector<std::string> _ref_names_vec;

            std::vector<Eigen::VectorXd> _com_refs;
            std::unordered_map<std::string, std::vector<pinocchio::SE3>> _task_refs;
            std::unordered_map<std::string, std::vector<Eigen::VectorXd>> _task_refs_vec;
        };
    } // namespace trajs
} // namespace inria_wbc

#endif