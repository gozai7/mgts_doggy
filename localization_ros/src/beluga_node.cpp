#include <beluga/algorithm/amcl_core.hpp>
#include <beluga/motion/omnidirectional_drive_model.hpp>
#include <beluga_vdb/sensor/vdb_likelihood_field_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <fstream>
#include <map>
#include <random>
#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <stdexcept>
#include <vector>
#include <iomanip>
#include <sstream>

struct Cloud {
  std::vector<Eigen::Vector3d> data;
  Sophus::SE3d mount;
  const auto& points() const { return data; }
  const auto& origin() const { return mount; }
};
using Sensor = beluga_vdb::VDBLikelihoodFieldModel2<openvdb::FloatGrid, Cloud>;
using Filter = beluga::Amcl<beluga::OmnidirectionalDriveModel, Sensor, std::function<Sophus::SE2d()>>;

static Sophus::SE3d pose3(const geometry_msgs::msg::Pose& p) {
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (!q.coeffs().allFinite() || std::abs(q.norm()-1.) > .01) throw std::runtime_error("Invalid odometry quaternion");
  Eigen::Vector3d t(p.position.x, p.position.y, p.position.z);
  if (!t.allFinite()) throw std::runtime_error("Invalid odometry position");
  return {q.normalized(), t};
}
static geometry_msgs::msg::Pose message(const Sophus::SE3d& t) {
  geometry_msgs::msg::Pose p;
  p.position.x=t.translation().x(); p.position.y=t.translation().y(); p.position.z=t.translation().z();
  auto q=t.unit_quaternion();
  p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z(); p.orientation.w=q.w();
  return p;
}
static int64_t stamp(const std_msgs::msg::Header& h) { return rclcpp::Time(h.stamp).nanoseconds(); }

class BelugaNode : public rclcpp::Node {
 public:
  BelugaNode() : Node("mgts_beluga") {
    const auto path=declare_parameter<std::string>("map_path");
    openvdb::initialize(); openvdb::io::File file(path); file.open();
    grid_=openvdb::gridPtrCast<openvdb::FloatGrid>(file.readGrid("map")); file.close();
    if (!grid_ || grid_->empty() || grid_->getGridClass()!=openvdb::GRID_LEVEL_SET)
      throw std::runtime_error("Expected nonempty VDB level-set map");
    std::ifstream free(declare_parameter<std::string>("free_cells_path"));
    double x,y; while (free >> x >> y) free_.emplace_back(x,y);
    if (!free.eof() || free_.empty()) throw std::runtime_error("Invalid/empty free-space mask");
    const auto extr=declare_parameter<std::vector<double>>("T_base_lidar");
    if (extr.size()!=7) throw std::runtime_error("Missing LiDAR extrinsics");
    geometry_msgs::msg::Pose p; p.position.x=extr[0]; p.position.y=extr[1]; p.position.z=extr[2];
    p.orientation.x=extr[3]; p.orientation.y=extr[4]; p.orientation.z=extr[5]; p.orientation.w=extr[6];
    mount_=pose3(p);
    frame_=declare_parameter<std::string>("global_frame", "map");
    initial_height_=declare_parameter<double>("initial_base_height_m");
    cell_resolution_=declare_parameter("free_resolution_m", .05);
    min_particles_=declare_parameter("min_particles", 2000);
    max_particles_=declare_parameter("max_particles", 12000);
    max_points_=declare_parameter("max_points", 256);
    sigma_=declare_parameter("sigma_hit_m", .15);
    max_xy_var_=declare_parameter("max_xy_variance_m2", .16);
    max_yaw_var_=declare_parameter("max_yaw_variance_rad2", .03);
    min_overlap_=declare_parameter("min_overlap", .65);
    seed_=declare_parameter("seed", 42);
    alpha_=declare_parameter("motion_noise", .15);
    known_=declare_parameter<std::vector<double>>("initial_pose_se2", std::vector<double>{});
    if (min_particles_<2 || max_particles_<min_particles_ || max_points_<5 || alpha_<=0 || sigma_<=0)
      throw std::runtime_error("Invalid particle/sensor/motion configuration");
    rng_.seed(seed_); ranges::detail::get_random_engine().seed(seed_);
    reset();
    const auto latched=rclcpp::QoS(1000).reliable().transient_local();
    pose_pub_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/beluga/pose",latched);
    status_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/beluga/alignment_status",latched);
    particles_pub_=create_publisher<geometry_msgs::msg::PoseArray>("/beluga/particles",rclcpp::QoS(1).best_effort());
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/competition/odometry",1000,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        odom_[stamp(m->header)]=m;
        if (pending_ && odom_.count(stamp(pending_->header))) { auto cloud=pending_; pending_.reset(); process(cloud); }
        while (odom_.size()>1000) odom_.erase(odom_.begin());
      });
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/competition/lidar",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
        if (odom_.count(stamp(m->header))) process(m); else pending_=m;
      });
    reset_srv_=create_service<std_srvs::srv::Trigger>("/beluga/global_localization",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r) {
        known_.clear(); reset(); ++resets_; r->success=true; r->message="Uniform free-space initialization";
      });
  }
 private:
  void reset() {
    beluga::AmclParams params; params.min_particles=min_particles_; params.max_particles=max_particles_;
    params.update_min_a=0.; params.update_min_d=0.; params.kld_epsilon=.03;
    params.alpha_slow=.001; params.alpha_fast=.1;
    Sensor::param_type sensor; sensor.sigma_hit=sigma_; sensor.z_hit=.95; sensor.z_random=.05; sensor.max_laser_distance=100.;
    auto sample=[this]() {
      const auto& xy=free_[std::uniform_int_distribution<std::size_t>(0,free_.size()-1)(rng_)];
      std::uniform_real_distribution<double> jitter(-cell_resolution_/2,cell_resolution_/2), yaw(-M_PI,M_PI);
      return Sophus::SE2d{Sophus::SO2d(yaw(rng_)), xy+Eigen::Vector2d(jitter(rng_),jitter(rng_))};
    };
    filter_=std::make_unique<Filter>(beluga::OmnidirectionalDriveModel({alpha_,alpha_,alpha_,alpha_,alpha_}),
      Sensor(sensor,*grid_),sample,beluga::spatial_hash<Sophus::SE2d>(.3,.15),params);
    if (known_.empty()) {
      filter_->initialize([sample](auto&) mutable { return sample(); });
    } else {
      if (known_.size()!=3) throw std::runtime_error("initial_pose_se2 must be empty or [x,y,yaw]");
      filter_->initialize(Sophus::SE2d(Sophus::SO2d(known_[2]),Eigen::Vector2d(known_[0],known_[1])),
                          Eigen::Vector3d(.04,.04,.02).asDiagonal().toDenseMatrix());
    }
  }
  void process(sensor_msgs::msg::PointCloud2::ConstSharedPtr scan) {
    const auto started=std::chrono::steady_clock::now();
    const auto odom=odom_.at(stamp(scan->header));
    const auto base=pose3(odom->pose.pose);
    if (!z_offset_) z_offset_=initial_height_-base.translation().z();
    const auto planar=beluga::To2d(base);
    auto residual=beluga::To3d(planar).inverse()*base;
    residual.translation().z()+=*z_offset_;
    Cloud cloud; cloud.mount=residual*mount_;
    sensor_msgs::PointCloud2ConstIterator<float> ix(*scan,"x"), iy(*scan,"y"), iz(*scan,"z");
    const std::size_t stride=std::max<std::size_t>(1,(scan->width*scan->height+max_points_-1)/max_points_);
    for (std::size_t i=0;ix!=ix.end();++ix,++iy,++iz,++i)
      if (i%stride==0 && std::isfinite(*ix) && std::isfinite(*iy) && std::isfinite(*iz)) cloud.data.emplace_back(*ix,*iy,*iz);
    if (cloud.data.size()<5) throw std::runtime_error("Too few finite LiDAR observations");
    filter_->force_update(); const auto estimate=filter_->update(planar,cloud).value();
    const auto& mean=estimate.first; const auto& cov=estimate.second;
    const auto full=beluga::To3d(mean)*residual;
    auto access=grid_->getConstAccessor(); std::size_t hits=0;
    for (const auto& point : cloud.data) {
      const auto world=beluga::To3d(mean)*(cloud.mount*point);
      const auto ijk=grid_->transform().worldToIndexCellCentered(openvdb::Vec3d(world.x(),world.y(),world.z()));
      if (std::abs(access.getValue(ijk))<2*sigma_) ++hits;
    }
    const double overlap=static_cast<double>(hits)/static_cast<double>(cloud.data.size());
    const bool accepted=std::max(cov(0,0),cov(1,1))<max_xy_var_ && cov(2,2)<max_yaw_var_ && overlap>=min_overlap_;
    geometry_msgs::msg::PoseWithCovarianceStamped out; out.header=scan->header; out.header.frame_id=frame_;
    out.pose.pose=message(full);
    // Conditional blocks: no claim of a jointly estimated 6DoF covariance or known cross-correlations.
    for (int axis : {2,3,4}) out.pose.covariance[axis*6+axis]=odom->pose.covariance[axis*6+axis];
    const int axes[3]={0,1,5};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) out.pose.covariance[axes[i]*6+axes[j]]=cov(i,j);
    pose_pub_->publish(out);
    geometry_msgs::msg::PoseArray particles; particles.header=out.header;
    for (const auto& state : filter_->particles() | beluga::views::states) particles.poses.push_back(message(beluga::To3d(state)*residual));
    particles_pub_->publish(particles);
    diagnostic_msgs::msg::DiagnosticArray diagnostics; diagnostics.header=out.header;
    diagnostic_msgs::msg::DiagnosticStatus status; status.name="mgts_beluga/alignment";
    status.level=accepted?0:1; status.message=accepted?"ok":"global_search";
    auto value=[&](const std::string& key, double n) {
      diagnostic_msgs::msg::KeyValue v; v.key=key;
      std::ostringstream stream; stream<<std::setprecision(17)<<n; v.value=stream.str(); status.values.push_back(v);
    };
    value("particles",static_cast<double>(filter_->particles().size())); value("overlap",overlap);
    value("variance_x",cov(0,0)); value("variance_y",cov(1,1)); value("variance_yaw",cov(2,2));
    value("reinitializations",resets_);
    value("update_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count());
    diagnostics.status.push_back(status); status_pub_->publish(diagnostics);
  }
  openvdb::FloatGrid::Ptr grid_; std::vector<Eigen::Vector2d> free_;
  std::unique_ptr<Filter> filter_; std::mt19937 rng_; Sophus::SE3d mount_;
  double initial_height_,cell_resolution_,sigma_,max_xy_var_,max_yaw_var_,min_overlap_,alpha_;
  int64_t min_particles_,max_particles_,max_points_,seed_,resets_=0;
  std::optional<double> z_offset_; std::vector<double> known_; std::string frame_;
  std::map<int64_t,nav_msgs::msg::Odometry::ConstSharedPtr> odom_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<BelugaNode>()); }
  catch (const std::exception& e) { RCLCPP_ERROR(rclcpp::get_logger("mgts_beluga"),"%s",e.what()); rclcpp::shutdown(); return 2; }
  rclcpp::shutdown();
}
