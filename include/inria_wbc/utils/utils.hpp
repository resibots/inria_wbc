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

    } // namespace utils
} // namespace inria_wbc
#endif
