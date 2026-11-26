#pragma once

#include <array>
#include <cstdint>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rosbag2_cpp { class Writer; }
namespace competition {

using Cloud = pcl::PointCloud<pcl::PointXYZINormal>;
constexpr char kLidarTopic[] = "/competition/lidar";
constexpr char kImuTopic[] = "/competition/imu";

struct Scan {
    Cloud::Ptr points;
    double begin;
    double end;
};

Scan unpack_scan(const sensor_msgs::msg::PointCloud2 &msg, bool timed);
int64_t replay_bag(const std::string &path,
                   const std::function<void(const sensor_msgs::msg::PointCloud2 &)> &lidar,
                   const std::function<void(sensor_msgs::msg::Imu::SharedPtr)> &imu,
                   const std::function<void(int64_t, const std::string &)> &initialize);

class Output {
public:
    Output(const std::string &dir, std::string lidar_frame, double voxel, const std::string &type_hash);
    ~Output();
    void append(int64_t stamp, const Eigen::Isometry3d &map_imu,
                const Eigen::Isometry3d &map_lidar, const Cloud &scan);
    void finish();

private:
    std::string dir_;
    std::string lidar_frame_;
    double voxel_;
    int64_t last_stamp_ = -1;
    size_t poses_ = 0;
    std::ofstream trajectory_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    std::map<std::array<int64_t, 3>, pcl::PointXYZI> map_;
};

}  // namespace competition
