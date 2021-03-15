#ifndef IWBC_UTILS_HPP
#define IWBC_UTILS_HPP

/*!
 * \file utils.hpp
 * \brief parsing utils
 * \author Eloïse Dalin
 * \version 0.1
 */

#include <cstdlib>
#include <unordered_set>
#include <yaml-cpp/yaml.h>

#include <inria_wbc/exceptions.hpp>

namespace inria_wbc {
    namespace utils {
        template <typename type>
        inline bool parse(type& parameter, std::string parameterName, YAML::Node& config, std::string prevName, bool verbose = true)
        {
            if (!config[prevName][parameterName]) {
                throw IWBC_EXCEPTION("Parameter :", prevName, "/", parameterName, " not found in YAML file");
            }
            else {
                parameter = config[prevName][parameterName].as<type>();
                return true;
            }
        }

        // special case for strings
        inline std::string parse(std::string parameterName, YAML::Node& config, std::string prevName, bool verbose = true)
        {
            std::string str;
            parse(str, parameterName, config, prevName, verbose);
            return str;
        }

        template <typename T>
        inline std::vector<T> remove_intersection(const std::vector<T>& vec, const std::vector<T>& b)
        {
            std::vector<T> a = vec;
            std::unordered_multiset<T> st;
            st.insert(a.begin(), a.end());
            st.insert(b.begin(), b.end());
            auto predicate = [&st](const T& k) { return st.count(k) > 1; };
            a.erase(std::remove_if(a.begin(), a.end(), predicate), a.end());
            return a;
        }

        inline Eigen::VectorXd slice_vec(const Eigen::VectorXd& vec, const std::vector<int>& indexes)
        {
            Eigen::VectorXd filtered(indexes.size());
            uint k = 0;
            for (uint i = 0; i < vec.size(); i++) {
                auto it = std::find(indexes.begin(), indexes.end(), i);
                if (it != indexes.end()) {
                    filtered(k) = vec(i);
                    k++;
                }
            }
            return filtered;
        }

        // search and replace a value in a yaml file.
        // you need to provide the precise section (and subsection - if there is one) 
        // ex inria_wbc::utils::search_and_replace(config_path, "false", "stabilizer:", "activated:");
        // returns the initial value that was contained in the yaml
        inline std::string search_and_replace(
            const std::string& infile,
            const std::string& value,
            const std::string& section,
            const std::string& sub_section = "",
            std::string outfile = "")
        {
            if (outfile.size() == 0)
                outfile = infile;

            std::string initial_value;
            std::ifstream input_file(infile);
            std::vector<std::string> lines;
            std::string input;
            while (std::getline(input_file, input))
                lines.push_back(input);

            bool found_section = false;
            bool replace = false;

            for (auto& line : lines) {
                if (!found_section) {
                    if (line.find(section) != std::string::npos) {
                        found_section = true;
                        if (sub_section.size() == 0) {
                            line.erase(0, line.find(section) + section.size() + 1);
                            initial_value = line;
                            line = "  " + section + " " + value;
                        }
                        else {
                            replace = true;
                        }
                    }
                }
                if (replace) {
                    if (line.find(sub_section) != std::string::npos) {
                        line.erase(0, line.find(sub_section) + sub_section.size() + 1);
                        initial_value = line;
                        line = "    " + sub_section + " " + value;
                        replace = false;
                    }
                }
            }
            input_file.close();

            std::ofstream output_file(outfile);
            for (auto const& line : lines)
                output_file << line << '\n';
            output_file.close();

            return initial_value;
        }

    } // namespace utils
} // namespace inria_wbc
#endif
