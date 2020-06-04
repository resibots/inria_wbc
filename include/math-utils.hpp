#ifndef UTILS_HPP
#define UTILS_HPP

/*!
 * \file utils.hpp
 * \brief quaternion utils
 * \author Eloïse Dalin
 * \version 0.1
 */

#include <cstdlib>

// To convert quaternions to euler angles
// source https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
struct Quaternion
{
  double w, x, y, z;
};

struct EulerAngles
{
  double roll, pitch, yaw;
};

EulerAngles to_euler(Quaternion q)
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

#endif
