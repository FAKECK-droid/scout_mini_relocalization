#include <iostream>
#include <stdexcept>
#include <vector>
#include "scout_mini_relocalization/registration.hpp"

using namespace scout_mini_relocalization;

void require(bool value, const char * message)
{
  if (!value) {throw std::runtime_error(message);}
}

Eigen::Isometry3d pose(double x, double y, double z, double yaw)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(x, y, z);
  result.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitX())).toRotationMatrix();
  return result;
}

int main()
{
  try {
    const auto imu_lidar = make_transform({-0.011, -0.02329, 0.04412},
      {1, 0, 0, 0, 1, 0, 0, 0, 1});
    const auto base_body = base_from_body(
      {-0.011, -0.02329, 0.04412}, {1, 0, 0, 0, 1, 0, 0, 0, 1},
      {0.07, -0.09, 0.145}, {0.11, -0.12, -1.1});
    const auto base_lidar = base_body * imu_lidar;
    require((base_body * imu_lidar.matrix().col(3) - base_lidar.matrix().col(3)).norm() < 1e-12,
      "IMU-lidar inverse direction");
    const auto map_odom = pose(4.0, -3.0, 0.5, 0.35);
    const auto odom_base1 = pose(1.4, -0.8, 0.2, -0.25);
    const auto odom_base2 = pose(1.8, -0.7, 0.3, -0.1);
    const auto seed = map_from_odom(map_odom * odom_base1, odom_base1);
    require((seed.matrix() - map_odom.matrix()).norm() < 1e-12, "nonidentity EKF compensation");
    require((seed * odom_base2).isApprox(map_odom * odom_base2), "seed motion propagation");
    require(!nearby(map_odom * odom_base1, map_odom * odom_base2, 0.1, 0.05),
      "absolute poses change with motion");
    require(nearby(map_from_odom(map_odom * odom_base1, odom_base1),
      map_from_odom(map_odom * odom_base2, odom_base2), 1e-10, 1e-10),
      "confirmation must compare map-odom");
    bool invalid = false;
    try {make_transform({0, 0, 0}, {1, 0, 0, 0, 1, 0, 0, 0, -1});}
    catch (const std::invalid_argument &) {invalid = true;}
    require(invalid, "reflection must be rejected");

    std::vector<Eigen::Vector3d> target_points, source_points;
    const auto known = pose(0.22, -0.18, 0.13, 0.06);
    for (int i = -25; i <= 25; ++i) {
      for (int j = -20; j <= 20; ++j) {
        const double x = i * 0.12, y = j * 0.12;
        target_points.emplace_back(x, y, 0.12 * std::sin(x) + 0.08 * std::cos(2 * y));
        target_points.emplace_back(x, 3.0 + 0.10 * std::sin(y), y + 2.5);
        target_points.emplace_back(-3.4 + 0.07 * std::cos(x), x, y + 2.5);
      }
    }
    for (const auto & p : target_points) {source_points.push_back(known.inverse() * p);}
    auto target = small_gicp::preprocess_points(target_points, 0.15, 20, 2);
    auto source = small_gicp::preprocess_points(source_points, 0.15, 20, 2);
    small_gicp::RegistrationSetting setting;
    setting.num_threads = 2;
    setting.max_iterations = 40;
    auto result = small_gicp::align(*target.first, *source.first, *target.second,
      Eigen::Isometry3d::Identity(), setting);
    auto quality = measure_quality(*source.first, *target.second, result, 1.0);
    require(rejection_reason(result, quality, Eigen::Isometry3d::Identity(), 200, 100,
      0.5, 0.30, 1.0, 0.3).empty(), "GICP quality");
    require(nearby(result.T_target_source, known, 0.02, 0.005), "source-to-target recovery");
    require(rejection_reason(result, quality, pose(10, 0, 0, 1), 200, 100,
      0.5, 0.30, 1.0, 0.3) == "correction_jump", "jump gate");
    result.converged = false;
    require(rejection_reason(result, quality, known, 200, 100, 0.5, 0.30, 1.0, 0.3) ==
      "not_converged", "convergence gate");
    std::cout << "geometry and GICP passed; RMSE=" << quality.rmse
              << " points=" << quality.source_points << std::endl;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
  return 0;
}
