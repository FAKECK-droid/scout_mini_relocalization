#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <Eigen/Geometry>

namespace scout_mini_relocalization
{
inline bool valid_transform(const Eigen::Isometry3d & transform)
{
  const auto & r = transform.linear();
  return transform.matrix().allFinite() &&
         (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
         std::abs(r.determinant() - 1.0) < 1e-6;
}

inline Eigen::Isometry3d make_transform(
  const std::vector<double> & translation, const std::vector<double> & rotation)
{
  if (translation.size() != 3 || rotation.size() != 9) {
    throw std::invalid_argument("Extrinsics require 3 translation and 9 row-major rotation values");
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 3; ++i) {
    transform.translation()[i] = translation[i];
    for (int j = 0; j < 3; ++j) {
      transform.linear()(i, j) = rotation[3 * i + j];
    }
  }
  if (!valid_transform(transform)) {
    throw std::invalid_argument("Extrinsics must be finite with an orthonormal right-handed rotation");
  }
  return transform;
}

inline Eigen::Isometry3d base_from_body(
  const std::vector<double> & t_imu_lidar, const std::vector<double> & r_imu_lidar,
  const std::vector<double> & t_base_lidar, const std::vector<double> & rpy_base_lidar)
{
  if (rpy_base_lidar.size() != 3 ||
    !std::all_of(rpy_base_lidar.begin(), rpy_base_lidar.end(),
      [](double value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument("rpy_base_lidar requires three finite radians");
  }
  const Eigen::Matrix3d r =
    (Eigen::AngleAxisd(rpy_base_lidar[2], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy_base_lidar[1], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy_base_lidar[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
  std::vector<double> flat(9);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      flat[3 * i + j] = r(i, j);
    }
  }
  // T_B_I = T_B_L * inverse(T_I_L); body is the FAST-LIO IMU frame.
  return make_transform(t_base_lidar, flat) * make_transform(t_imu_lidar, r_imu_lidar).inverse();
}

inline Eigen::Isometry3d map_from_odom(
  const Eigen::Isometry3d & map_base, const Eigen::Isometry3d & odom_base)
{
  return map_base * odom_base.inverse();
}

inline double rotation_distance(
  const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  return std::abs(Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle());
}

inline bool nearby(
  const Eigen::Isometry3d & a, const Eigen::Isometry3d & b,
  double translation_limit, double rotation_limit)
{
  return (a.translation() - b.translation()).norm() <= translation_limit &&
         rotation_distance(a, b) <= rotation_limit;
}
}  // namespace scout_mini_relocalization
