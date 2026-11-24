// Directional odometry for the sequential Karto scan matcher (map coordinates).
#pragma once

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <optional>
#include "karto_sdk/Mapper.h"

namespace mgts {
inline std::optional<Eigen::Vector2d> weakAxis(karto::LocalizedRangeScan *scan) {
  const auto *laser = scan->GetLaserRangeFinder();
  const auto &points = scan->GetPointReadings(false);  // one world-frame point per beam
  const auto *ranges = scan->GetRangeReadings();
  const int half = std::max(1, static_cast<int>(std::lround(0.07 / laser->GetAngularResolution())));
  Eigen::Matrix2d information = Eigen::Matrix2d::Zero();
  int count = 0;
  for (int i = half; i + half < static_cast<int>(points.size()); ++i) {
    const auto valid = [&](int j) {
      return ranges[j] > laser->GetMinimumRange() && ranges[j] < laser->GetRangeThreshold();
    };
    if (!valid(i - half) || !valid(i) || !valid(i + half)) continue;
    const auto &a = points[i - half], &b = points[i], &c = points[i + half];
    const Eigen::Vector2d chord(c.GetX() - a.GetX(), c.GetY() - a.GetY());
    const double length = chord.norm();
    if (length < 0.15 || length > 1.5) continue;
    const Eigen::Vector2d normal(-chord.y() / length, chord.x() / length);
    // A large point-to-line residual indicates a corner or a break in the surface.
    if (std::abs(normal.dot(Eigen::Vector2d(b.GetX() - a.GetX(), b.GetY() - a.GetY()))) > 0.045) continue;
    information += normal * normal.transpose();
    ++count;
  }
  if (count < 20) return std::nullopt;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(information);
  if (eigen.eigenvalues()[0] >= 0.08 * eigen.eigenvalues()[1]) return std::nullopt;
  return eigen.eigenvectors().col(0);
}

inline void keepOdometry(karto::LocalizedRangeScan *scan, const Eigen::Vector2d &axis,
                         karto::Pose2 &matched, karto::Matrix3 &covariance) {
  const auto &prior = scan->GetCorrectedPose();  // last map pose composed with local odometry
  const auto base = scan->GetCorrectedAt(matched);
  const double along = axis.dot(Eigen::Vector2d(base.GetX() - prior.GetX(), base.GetY() - prior.GetY()));
  matched.SetX(matched.GetX() - along * axis.x());
  matched.SetY(matched.GetY() - along * axis.y());
  // Keep Karto's lateral variance. The longitudinal edge now uses a 5 cm odometry noise floor.
  const Eigen::Vector2d across(-axis.y(), axis.x());
  Eigen::Matrix2d xy;
  xy << covariance(0,0), covariance(0,1), covariance(1,0), covariance(1,1);
  xy = (across.dot(xy * across) * across * across.transpose()
        + std::max(0.0025, axis.dot(xy * axis)) * axis * axis.transpose()).eval();
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j) covariance(i,j) = xy(i,j);
}
}  // namespace mgts
