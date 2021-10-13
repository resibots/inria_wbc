#ifndef IWBC_TRAJ_SAVER__
#define IWBC_TRAJ_SAVER__
#include <boost/filesystem.hpp> // in c++11 we will use std::filesystem
#include <fstream>

#include <inria_wbc/controllers/pos_tracker.hpp>

namespace inria_wbc {
    namespace trajs {
        // this saves in a directory (name):
        // - a yaml with meta-data (associates a task to a file)
        // - a file for each task with se3 references (3D ref for CoM)
        class Saver {
        public:
            // you should use se3 task names, CoM is a special case (3D)
            Saver(const std::shared_ptr<controllers::PosTracker>& pos_tracker, const std::string& traj_name, const std::vector<std::string>& ref_names);
            void update();
        protected:
            std::shared_ptr<controllers::PosTracker> _pos_tracker;
            std::string _traj_name;
            std::vector<std::string> _ref_names;
            std::vector<std::shared_ptr<std::ofstream>> _ref_ofs;
            YAML::Emitter _yaml;
        };
    } // namespace trajs
} // namespace inria_wbc

#endif