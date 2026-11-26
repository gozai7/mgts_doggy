#include "competition_io.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_filter.hpp>

namespace competition {

Scan unpack_scan(const sensor_msgs::msg::PointCloud2 &msg, bool timed)
{
    // normalize_bag writes packed little-endian XYZ intensity and optional double time.
    const auto step = timed ? 24u : 16u;
    if (msg.is_bigendian || msg.height != 1 || msg.width < 5 || msg.point_step != step ||
        msg.row_step != msg.width * size_t(step) || msg.data.size() != msg.row_step) {
        throw std::runtime_error("Unexpected normalized PointCloud2 layout");
    }
    Scan scan{std::make_shared<Cloud>(), rclcpp::Time(msg.header.stamp).seconds(), 0.0};
    scan.points->resize(msg.width);
    double first = 0.0;
    double previous = 0.0;
    for (size_t i = 0; i < msg.width; ++i) {
        const auto *data = msg.data.data() + i * step;
        float xyzi[4];
        std::memcpy(xyzi, data, sizeof(xyzi));
        double offset = 0.0;
        if (timed) {
            std::memcpy(&offset, data + sizeof(xyzi), sizeof(offset));
        }
        if (!std::isfinite(offset) || !std::isfinite(xyzi[0]) ||
            !std::isfinite(xyzi[1]) || !std::isfinite(xyzi[2])) {
            throw std::runtime_error("Nonfinite value in a normalized scan");
        }
        if (i == 0) {
            first = offset;
        } else if (offset < previous) {
            throw std::runtime_error("Normalized point times are not sorted");
        }
        previous = offset;
        auto &point = (*scan.points)[i];
        point.x = xyzi[0];
        point.y = xyzi[1];
        point.z = xyzi[2];
        point.intensity = std::isfinite(xyzi[3]) ? xyzi[3] : 0.0f;
        point.curvature = static_cast<float>((offset - first) * 1000.0);
    }
    scan.end = scan.begin + previous;
    scan.begin += first;
    return scan;
}

template<class Message>
void deserialize(const rosbag2_storage::SerializedBagMessage &record, Message &msg)
{
    rclcpp::SerializedMessage serialized(*record.serialized_data);
    rclcpp::Serialization<Message>().deserialize_message(&serialized, &msg);
}

int64_t replay_bag(const std::string &path,
                   const std::function<void(const sensor_msgs::msg::PointCloud2 &)> &lidar,
                   const std::function<void(sensor_msgs::msg::Imu::SharedPtr)> &imu,
                   const std::function<void(int64_t, const std::string &)> &initialize)
{
    rosbag2_cpp::Reader reader;
    reader.open(path);
    const auto topics = reader.get_all_topics_and_types();
    const auto cloud_topic = std::find_if(topics.begin(), topics.end(), [](const auto &topic) {
        return topic.name == kLidarTopic && topic.type == "sensor_msgs/msg/PointCloud2";
    });
    if (cloud_topic == topics.end() || cloud_topic->type_description_hash.empty()) {
        throw std::runtime_error("Normalized bag is missing PointCloud2 type metadata");
    }
    rosbag2_storage::StorageFilter filter;
    filter.topics = {kLidarTopic, kImuTopic};
    reader.set_filter(filter);
    int64_t epoch = -1;
    int64_t scans = 0;
    while (rclcpp::ok() && reader.has_next()) {
        const auto record = reader.read_next();
        if (epoch < 0) {
            // Leave room for scans whose point offsets precede their header stamp.
            epoch = std::max<int64_t>(0, record->recv_timestamp - 30'000'000'000LL);
            initialize(epoch, cloud_topic->type_description_hash);
        }
        if (record->topic_name == kLidarTopic) {
            sensor_msgs::msg::PointCloud2 msg;
            deserialize(*record, msg);
            msg.header.stamp = rclcpp::Time(rclcpp::Time(msg.header.stamp).nanoseconds() - epoch);
            lidar(msg);
            ++scans;
        } else {
            auto msg = std::make_shared<sensor_msgs::msg::Imu>();
            deserialize(*record, *msg);
            msg->header.stamp = rclcpp::Time(rclcpp::Time(msg->header.stamp).nanoseconds() - epoch);
            imu(msg);
        }
    }
    if (!rclcpp::ok()) {
        throw std::runtime_error("Fast-LIO replay interrupted");
    }
    return scans;
}

Output::Output(const std::string &dir, std::string lidar_frame, double voxel, const std::string &type_hash)
    : dir_(dir), lidar_frame_(std::move(lidar_frame)), voxel_(voxel)
{
    if (!std::filesystem::create_directory(dir_)) {
        throw std::runtime_error("Cannot create Fast-LIO output directory " + dir_);
    }
    trajectory_.exceptions(std::ios::badbit | std::ios::failbit);
    trajectory_.open(dir_ + "/trajectory_imu.tum");
    rosbag2_storage::StorageOptions options;
    options.uri = dir_ + "/scans";
    options.storage_id = "sqlite3";
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(options);
    // The typed Writer overload omits the hash which rosbags validates on read.
    rosbag2_storage::TopicMetadata topic;
    topic.name = kLidarTopic;
    topic.type = "sensor_msgs/msg/PointCloud2";
    topic.serialization_format = "cdr";
    topic.type_description_hash = type_hash;
    writer_->create_topic(topic);
}

Output::~Output() = default;

void Output::append(int64_t stamp, const Eigen::Isometry3d &map_imu,
                    const Eigen::Isometry3d &map_lidar, const Cloud &scan)
{
    if (stamp <= last_stamp_ || !map_imu.matrix().allFinite() ||
        map_imu.translation().cwiseAbs().maxCoeff() > 1e8) {
        throw std::runtime_error("Invalid Fast-LIO pose or scan-end timestamp");
    }
    const Eigen::Quaterniond orientation(map_imu.rotation());
    trajectory_ << stamp / 1'000'000'000 << '.' << std::setw(9) << std::setfill('0')
                << stamp % 1'000'000'000 << std::setfill(' ') << ' '
                << std::fixed << std::setprecision(9) << map_imu.translation().transpose() << ' '
                << orientation.coeffs().transpose() << '\n';

    pcl::PointCloud<pcl::PointXYZI> points;
    points.resize(scan.size());
    for (size_t i = 0; i < scan.size(); ++i) {
        const auto &source = scan[i];
        points[i] = pcl::PointXYZI(source.x, source.y, source.z, source.intensity);
        const Eigen::Vector3d world = map_lidar * Eigen::Vector3d(source.x, source.y, source.z);
        if (!world.allFinite()) {
            throw std::runtime_error("Nonfinite deskewed Fast-LIO point");
        }
        std::array<int64_t, 3> key;
        for (int axis = 0; axis < 3; ++axis) {
            key[axis] = static_cast<int64_t>(std::floor(world[axis] / voxel_));
        }
        map_.try_emplace(key, pcl::PointXYZI(world.x(), world.y(), world.z(), source.intensity));
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(points, msg);
    msg.header.frame_id = lidar_frame_;
    msg.header.stamp = rclcpp::Time(stamp);
    writer_->write(msg, kLidarTopic, rclcpp::Time(stamp));
    last_stamp_ = stamp;
    ++poses_;
}

void Output::finish()
{
    if (poses_ < 2 || map_.empty()) {
        throw std::runtime_error("Fast-LIO could not build a map and trajectory from the input");
    }
    writer_->close();
    trajectory_.close();
    pcl::PointCloud<pcl::PointXYZI> map;
    map.reserve(map_.size());
    for (const auto &voxel : map_) {
        map.push_back(voxel.second);
    }
    if (pcl::io::savePCDFileBinary(dir_ + "/map.pcd", map) != 0) {
        throw std::runtime_error("Could not save the Fast-LIO map");
    }
}

}  // namespace competition
