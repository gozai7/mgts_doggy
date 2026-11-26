#include <glim/odometry/callbacks.hpp>
#include <glim/util/extension_module.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Callbacks = glim::OdometryEstimationCallbacks;
using gtsam::symbol_shorthand::X;

const char* required(const char* name) {
  const char* value = std::getenv(name);
  if (!value || !*value) throw std::runtime_error(std::string("Missing ") + name);
  return value;
}

double sigma(const char* name) {
  if (const char* value = std::getenv(name)) {
    std::size_t consumed = 0;
    double s = std::stod(value, &consumed);
    if (value[consumed] || !std::isfinite(s) || s <= 0) throw std::runtime_error(name);
    return s;
  }
  return 0.003;
}

gtsam::Pose3 pose(std::istream& stream) {
  double x, y, z, qx, qy, qz, qw;
  if (!(stream >> x >> y >> z >> qx >> qy >> qz >> qw) || !(stream >> std::ws).eof()) {
    throw std::runtime_error("Expected x y z qx qy qz qw");
  }
  Eigen::Quaterniond q(qw, qx, qy, qz);
  gtsam::Point3 p(x, y, z);
  if (!p.allFinite() || !q.coeffs().allFinite() || std::abs(q.norm()-1.0)>1e-3) {
    throw std::runtime_error("Invalid odometry pose");
  }
  return gtsam::Pose3(gtsam::Rot3(q.normalized()), p);
}

struct Sample {
  double stamp;
  gtsam::Pose3 body;
};

class Go2Odometry final : public glim::ExtensionModule {
 public:
  Go2Odometry() {
    std::istringstream calibration(required("MGTS_GO2_BASE_IMU"));
    mount_ = pose(calibration);
    std::ifstream input(required("MGTS_GO2_ODOM_TUM"));
    std::string line;
    while (std::getline(input, line)) {
      if (line.empty() || line[0]=='#') continue;
      std::istringstream row(line);
      double t;
      if (!(row >> t) || !std::isfinite(t) || (!samples_.empty() && t <= samples_.back().stamp)) {
        throw std::runtime_error("Non-monotonic Go2 odometry timestamps");
      }
      samples_.push_back({t, pose(row)});
    }
    if (input.bad() || samples_.size()<2) throw std::runtime_error("Cannot load Go2 odometry TUM");
    gtsam::Vector6 sigmas;
    sigmas.head<3>().setConstant(sigma("MGTS_GO2_ODOM_SIGMA_RADIANS"));
    sigmas.tail<3>().setConstant(sigma("MGTS_GO2_ODOM_SIGMA_METRES"));
    noise_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    frame_slot_ = Callbacks::on_new_frame.add([this](const auto& frame) {
      previous_ = current_;
      current_ = at(frame->stamp);
      previous_id_ = id_;
      id_ = frame->id;
      stamp_ = frame->stamp;
    });
    update_slot_ = Callbacks::on_smoother_update.add([this](auto&, auto& graph, auto& values, auto& stamps) {
      if (!previous_ || !current_ || previous_id_ < 0 || id_ != previous_id_+1 || last_added_ == id_) return;
      const auto key = X(id_);
      const auto found = stamps.find(key);
      if (found == stamps.end() || found->second != stamp_ || !values.exists(key)) return;
      graph.template emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(previous_id_), key, previous_->between(*current_), noise_);
      last_added_ = id_;
      pending_ = true;
    });
    finish_slot_ = Callbacks::on_smoother_update_finish.add([this](auto&) {
      if (pending_) ++count_;
      pending_ = false;
    });
  }

  ~Go2Odometry() override {
    Callbacks::on_new_frame.remove(frame_slot_);
    Callbacks::on_smoother_update.remove(update_slot_);
    Callbacks::on_smoother_update_finish.remove(finish_slot_);
  }

  void at_exit(const std::string& dump) override {
    std::ofstream out(dump + "/go2_odom_factors.txt");
    out << count_ << '\n';
    out.close();
    if (!out) throw std::runtime_error("Cannot save Go2 odometry factor count");
    std::cerr << "go2_glim_odom: added=" << count_ << std::endl;
  }

 private:
  std::optional<gtsam::Pose3> at(double t) const {
    auto hi = std::lower_bound(samples_.begin(), samples_.end(), t,
                               [](const Sample& s, double key) { return s.stamp < key; });
    if (hi == samples_.end()) return {};
    if (hi->stamp == t) return hi->body.compose(mount_);
    if (hi == samples_.begin()) return {};
    const auto& lo = *std::prev(hi);
    double dt = hi->stamp-lo.stamp;
    if (dt > 0.1) return {};  // Do not bridge missing high-rate odometry.
    double f = (t-lo.stamp)/dt;
    auto q = lo.body.rotation().toQuaternion().slerp(f, hi->body.rotation().toQuaternion());
    const gtsam::Point3 p = (1-f)*lo.body.translation()+f*hi->body.translation();
    return gtsam::Pose3(gtsam::Rot3(q), p).compose(mount_);
  }

  std::vector<Sample> samples_;
  gtsam::Pose3 mount_;
  gtsam::SharedNoiseModel noise_;
  std::optional<gtsam::Pose3> previous_, current_;
  long previous_id_ = -1, id_ = -1, last_added_ = -1;
  double stamp_ = 0.0;
  bool pending_ = false;
  std::size_t count_ = 0;
  int frame_slot_, update_slot_, finish_slot_;
};
}  // namespace

extern "C" glim::ExtensionModule* create_extension_module() { return new Go2Odometry(); }
