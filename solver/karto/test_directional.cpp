#include "directional.hpp"
#include <limits>
#include <memory>
#include <stdexcept>

int main() {
  std::unique_ptr<karto::LaserRangeFinder> laser(karto::LaserRangeFinder::CreateLaserRangeFinder(
      karto::LaserRangeFinder_Custom, karto::Name("test_laser")));
  const double step = M_PI / 180, yaw = 0.71;
  laser->SetMinimumAngle(-M_PI);
  laser->SetMaximumAngle(M_PI - step);
  laser->SetAngularResolution(step);
  laser->SetMinimumRange(0.1);
  laser->SetMaximumRange(16.);
  laser->SetRangeThreshold(16.);
  laser->SetOffsetPose({0.3, -0.1, 0.});
  karto::SensorManager::GetInstance()->RegisterSensor(laser.get());

  enum class Scene { corridor, room, empty };
  for (const auto scene : {Scene::corridor, Scene::room, Scene::empty}) {
    karto::RangeReadingsVector ranges;
    for (int i = 0; i < 360; ++i) {
      const double angle = -M_PI + i * step;
      double range = 2. / std::abs(std::sin(angle));
      if (scene == Scene::room) range = std::min(range, 2. / std::abs(std::cos(angle)));
      if (scene == Scene::empty || range >= 16.) range = std::numeric_limits<double>::infinity();
      ranges.push_back(range);
    }
    karto::LocalizedRangeScan scan(laser->GetName(), ranges);
    scan.SetCorrectedPose({3., 4., yaw});
    const auto weak = mgts::weakAxis(&scan);
    if (scene != Scene::corridor) {
      if (weak) throw std::runtime_error("unexpected weak axis in room or empty scan");
      continue;
    }
    const Eigen::Vector2d along(std::cos(yaw), std::sin(yaw)), across(-along.y(), along.x());
    if (!weak || std::abs(weak->dot(along)) < 0.99999) throw std::runtime_error("wrong corridor axis");
    const auto delta = (1.4 * along - 0.28 * across).eval();
    auto matched = scan.GetSensorAt({3. + delta.x(), 4. + delta.y(), yaw + 0.12});
    karto::Matrix3 cov;
    cov.SetToIdentity();
    mgts::keepOdometry(&scan, *weak, matched, cov);
    const auto base = scan.GetCorrectedAt(matched);
    Eigen::Vector2d correction(base.GetX() - 3., base.GetY() - 4.);
    if ((correction + 0.28 * across).norm() > 1e-6 || std::abs(base.GetHeading() - yaw - 0.12) > 1e-6)
      throw std::runtime_error("wrong translation or rotation with offset sensor");
  }
  karto::SensorManager::GetInstance()->UnregisterSensor(laser.get());
}
