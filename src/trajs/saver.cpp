#include <inria_wbc/trajs/saver.hpp>

namespace inria_wbc {
    namespace trajs {
        // you should use se3 task names, CoM is a special case (3D)
        Saver::Saver(const std::shared_ptr<controllers::PosTracker>& pos_tracker, const std::string& traj_name, const std::vector<std::string>& ref_names)
            : _pos_tracker(pos_tracker),
              _traj_name(traj_name),
              _ref_names(ref_names)
        {
            // create a directory
            boost::filesystem::path traj_dir(traj_name);
            boost::filesystem::create_directory(traj_dir);

            std::string yaml_name = (traj_dir.filename_is_dot() || traj_dir.filename_is_dot_dot()) ? 
                "trajectory" : traj_dir.stem().string();

            // create a file in the directory for each se3 reference
            for (auto& x : ref_names)
                _ref_ofs.push_back(std::make_shared<std::ofstream>((traj_dir / (x + ".csv")).c_str()));
            // create the yaml file for meta data
            namespace y = YAML;
            // we make a "ref maps" because we might want to add more meta-data in the future (e.g., contacts)
            _yaml << y::BeginMap << y::Key << "refs" << y::Value;
            _yaml << y::BeginMap;
            for (auto& x : ref_names)
                _yaml << y::Key << x << y::Value << x + ".csv";
            _yaml << y::EndMap; // end map of refs
            _yaml << y::EndMap; // end map of meta_data

            std::ofstream ofs((traj_dir / (yaml_name + ".yaml")).c_str());
            ofs << _yaml.c_str();
        }
        
        void Saver::update()
        {
            for (size_t i = 0; i < _ref_names.size(); ++i) {
                if (_ref_names[i] == "com") {
                    auto com = _pos_tracker->get_com_ref();
                    (*_ref_ofs[i]) << com.transpose() << std::endl;
                }
                else {
                    auto se3 = _pos_tracker->get_se3_ref(_ref_names[i]);
                    // we flatten the rotation
                    Eigen::VectorXd rot = Eigen::Map<Eigen::VectorXd>(se3.rotation().data(), se3.rotation().size());
                    (*_ref_ofs[i]) << se3.translation().transpose() << " " << rot.transpose() << std::endl;
                }
            }
        }

    } // namespace controllers
} // namespace inria_wbc