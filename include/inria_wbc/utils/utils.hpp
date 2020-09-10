#ifndef IWBC_UTILS_HPP
#define IWBC_UTILS_HPP

/*!
 * \file utils.hpp
 * \brief quaternion + parsing utils
 * \author Eloïse Dalin
 * \version 0.1
 */

#include <cstdlib>
#include <yaml-cpp/yaml.h>
#include <unordered_set>
// To convert quaternions to euler angles
// source https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
namespace inria_wbc
{
  namespace utils
  {
    struct Quaternion
    {
      double w, x, y, z;
    };

    struct EulerAngles
    {
      double roll, pitch, yaw;
    };

    inline EulerAngles to_euler(Quaternion q)
    {
      EulerAngles angles;

      // roll (x-axis rotation)
      double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
      double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
      angles.roll = std::atan2(sinr_cosp, cosr_cosp);

      // pitch (y-axis rotation)
      double sinp = 2 * (q.w * q.y - q.z * q.x);
      if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
      else
        angles.pitch = std::asin(sinp);

      // yaw (z-axis rotation)
      double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      angles.yaw = std::atan2(siny_cosp, cosy_cosp);

      return angles;
    }

    template <typename type>
    inline void parse(type &parameter, std::string parameterName, YAML::Node &config, bool verbose = true, std::string prevName = "SOT")
    {
      if (!config[prevName][parameterName])
      {
        if (verbose)
          std::cout << "No parameter " << parameterName << " found taking the default one" << std::endl;
      }
      else
      {
        parameter = config[prevName][parameterName].as<type>();
        if (verbose)
          std::cout << "Parameter " << parameterName << " found" << std::endl;
      }
    }

    // special case for strings
    inline std::string parse(std::string parameterName, YAML::Node &config, bool verbose = true, std::string prevName = "SOT")
    {
      std::string str;
      parse(str, parameterName, config, verbose, prevName);
      return str;
    }

    template <typename T>
    inline std::vector<T> remove_intersection(const std::vector<T> &vec, const std::vector<T> &b)
    {
      std::vector<T> a = vec;
      std::unordered_multiset<T> st;
      st.insert(a.begin(), a.end());
      st.insert(b.begin(), b.end());
      auto predicate = [&st](const T &k) { return st.count(k) > 1; };
      a.erase(std::remove_if(a.begin(), a.end(), predicate), a.end());
      return a;
    }

    inline Eigen::VectorXd slice_vec(const Eigen::VectorXd &vec, const std::vector<int> &indexes)
    {
      Eigen::VectorXd filtered(indexes.size());
      uint k = 0;
      for (uint i = 0; i < vec.size(); i++)
      {
        auto it = std::find(indexes.begin(), indexes.end(), i);
        if (it != indexes.end())
        {
          filtered(k) = vec(i);
          k++;
        }
      }
      return filtered;
    }
  } // namespace utils
} // namespace inria_wbc
#endif
