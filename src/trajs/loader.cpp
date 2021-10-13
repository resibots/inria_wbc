#include <iostream>
#include <boost/filesystem.hpp>
#include <inria_wbc/trajs/loader.hpp>
#include <inria_wbc/exceptions.hpp>
#include <iterator> // std::istream_iterator
#include <yaml-cpp/yaml.h>

namespace inria_wbc {
    namespace trajs {

        static std::vector<Eigen::Vector3d> load_vector3d(const std::string& path)
        {
            static constexpr int cols = 3;
            std::ifstream ifs(path.c_str());
            IWBC_ASSERT(ifs.good(), std::string("Error when loading trajectory:") + path);
            // load all the values (treat \n as space here, so no info on column)
            std::vector<double> data = std::vector<double>{
                std::istream_iterator<double>(ifs),
                std::istream_iterator<double>()};
            // ensure that all the lines are full
            assert(data.size() % cols == 0);
            // copy to the matrix
            int rows = data.size() / cols;
            std::vector<Eigen::Vector3d> res(rows);
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++)
                    res[i](j) = data[i * cols + j];
            }
            return res;
        }

        static std::vector<pinocchio::SE3> load_se3(const std::string& path)
        {
            static constexpr int cols = 3 + 9;
            std::ifstream ifs(path.c_str());
            IWBC_ASSERT(ifs.good(), std::string("Error when loading trajectory:") + path);
            // load all the values (treat \n as space here, so no info on column)
            std::vector<double> data = std::vector<double>{
                std::istream_iterator<double>(ifs),
                std::istream_iterator<double>()};
            // ensure that all the lines are full
            assert(data.size() % cols == 0);
            // copy
            int rows = data.size() / cols;
            std::vector<pinocchio::SE3> res(rows);
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < 3; j++)
                    res[i].translation()(j) = data[i * cols + j];
                for (int j = 3; j < cols; j++)
                    res[i].rotation().data()[j - 3] = data[i * cols + j];
            }
            return res;
        }

        // you should use se3 task names, CoM is a special case (3D)
        Loader::Loader(const std::string& yaml_file)
        {
            auto yaml = IWBC_CHECK(YAML::LoadFile(yaml_file));
            auto path = boost::filesystem::path(yaml_file).remove_filename();
            auto refs = IWBC_CHECK(yaml["refs"]);
            for (const auto& x : refs) {
                auto task = x.first.as<std::string>();
                auto name = x.second.as<std::string>();
                auto filename = (path / boost::filesystem::path(name)).string();
                if (task != "com") {
                    _ref_names.push_back(task);
                    _task_refs[task] = load_se3(filename);
                    IWBC_ASSERT(_task_refs[task].size() == _task_refs.begin()->second.size(), std::string("wrong number of rows in") + filename);
                }
                else {
                    _com_refs = load_vector3d(filename);
                }              
            }
        }

    } // namespace controllers
} // namespace inria_wbc