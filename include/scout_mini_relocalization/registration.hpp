#pragma once

#include <limits>
#include <string>
#include <Eigen/Eigenvalues>
#include <small_gicp/registration/registration_helper.hpp>
#include "scout_mini_relocalization/geometry.hpp"

namespace scout_mini_relocalization
{
struct Quality
{
  size_t source_points = 0;
  size_t gicp_inliers = 0;
  size_t euclidean_inliers = 0;
  size_t iterations = 0;
  double rmse = std::numeric_limits<double>::infinity();
  double gicp_error = 0.0;
  double hessian_min_eigenvalue = 0.0;
  double hessian_condition = std::numeric_limits<double>::infinity();
  double processing_ms = 0.0;
  double correction_translation = std::numeric_limits<double>::infinity();
  double correction_rotation = std::numeric_limits<double>::infinity();
};

inline Quality measure_quality(
  const small_gicp::PointCloud & source,
  const small_gicp::KdTree<small_gicp::PointCloud> & tree,
  const small_gicp::RegistrationResult & result, double max_distance)
{
  Quality quality;
  quality.source_points = source.size();
  quality.gicp_inliers = result.num_inliers;
  quality.iterations = result.iterations;
  quality.gicp_error = result.error;
  double squared_sum = 0.0;
  if (valid_transform(result.T_target_source)) {
    for (size_t i = 0; i < source.size(); ++i) {
      const Eigen::Vector4d query = result.T_target_source.matrix() * source.point(i);
      size_t index = 0;
      double squared_distance = 0.0;
      if (tree.nearest_neighbor_search(query, &index, &squared_distance) == 1 &&
        std::isfinite(squared_distance) && squared_distance <= max_distance * max_distance)
      {
        ++quality.euclidean_inliers;
        squared_sum += squared_distance;
      }
    }
  }
  if (quality.euclidean_inliers != 0) {
    quality.rmse = std::sqrt(squared_sum / quality.euclidean_inliers);
  }
  if (result.H.allFinite()) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(
      0.5 * (result.H + result.H.transpose()));
    if (eigen.info() == Eigen::Success) {
      quality.hessian_min_eigenvalue = eigen.eigenvalues().minCoeff();
      if (quality.hessian_min_eigenvalue > 1e-12) {
        quality.hessian_condition =
          eigen.eigenvalues().maxCoeff() / quality.hessian_min_eigenvalue;
      }
    }
  }
  return quality;
}

inline std::string rejection_reason(
  const small_gicp::RegistrationResult & result, const Quality & quality,
  const Eigen::Isometry3d & guess, size_t min_source, size_t min_inliers,
  double min_ratio, double max_rmse, double max_translation, double max_rotation,
  bool check_correction = true)
{
  if (!result.converged) {return "not_converged";}
  if (!valid_transform(result.T_target_source) || !result.H.allFinite() ||
    !result.b.allFinite() || !std::isfinite(result.error))
  {
    return "nonfinite_registration";
  }
  if (quality.source_points < min_source) {return "insufficient_source_points";}
  if (quality.gicp_inliers < min_inliers || quality.euclidean_inliers < min_inliers) {
    return "insufficient_inliers";
  }
  if (static_cast<double>(quality.gicp_inliers) / quality.source_points < min_ratio ||
    static_cast<double>(quality.euclidean_inliers) / quality.source_points < min_ratio)
  {
    return "insufficient_inlier_ratio";
  }
  if (!std::isfinite(quality.rmse) || quality.rmse > max_rmse) {return "euclidean_rmse";}
  if (check_correction &&
    !nearby(guess, result.T_target_source, max_translation, max_rotation))
  {
    return "correction_jump";
  }
  return {};
}
}  // namespace scout_mini_relocalization
