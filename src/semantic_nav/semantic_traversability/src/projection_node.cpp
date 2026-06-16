// Spatial-projection node (paper 2310.08873, Sec. IV-A/B adapted to RGB-D).
//
// Subscribes:
//   ~/detection   btcpp_ros2_interfaces/SemanticDetection2D  (from the VLM node)
//   <depth_topic>       sensor_msgs/Image       (aligned depth, 32FC1 m or 16UC1 mm)
//   <camera_info_topic> sensor_msgs/CameraInfo  (intrinsics)
// Publishes:
//   <regions_topic> btcpp_ros2_interfaces/SemanticRegionArray (ground polygons)
//
// For each detection it deprojects the pixels inside the bbox (optionally the
// mask) to 3D using the depth + intrinsics, transforms them into a stable
// target frame, drops Z, and emits the convex-hull footprint on the ground.

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "btcpp_ros2_interfaces/msg/semantic_detection2_d.hpp"
#include "btcpp_ros2_interfaces/msg/semantic_region.hpp"
#include "btcpp_ros2_interfaces/msg/semantic_region_array.hpp"

namespace semantic_traversability {

class ProjectionNode : public rclcpp::Node {
public:
  ProjectionNode() : rclcpp::Node("semantic_projection_node") {
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/depth");
    camera_info_topic_ =
        declare_parameter<std::string>("camera_info_topic", "/camera_info");
    regions_topic_ =
        declare_parameter<std::string>("regions_topic", "/semantic_regions");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    // If empty, use the depth image's own header.frame_id as the optical frame.
    camera_optical_frame_ =
        declare_parameter<std::string>("camera_optical_frame", "");
    pixel_step_ = declare_parameter<int>("pixel_step", 4);
    min_depth_ = declare_parameter<double>("min_depth", 0.15);
    max_depth_ = declare_parameter<double>("max_depth", 6.0);
    transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.2);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::SensorDataQoS sensor_qos;
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, sensor_qos,
        [this](sensor_msgs::msg::Image::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_depth_ = msg;
        });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, sensor_qos,
        [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_info_ = msg;
        });
    detection_sub_ =
        create_subscription<btcpp_ros2_interfaces::msg::SemanticDetection2D>(
            "~/detection", 10,
            std::bind(&ProjectionNode::onDetection, this, std::placeholders::_1));

    rclcpp::QoS pub_qos(rclcpp::KeepLast(1));
    pub_qos.transient_local();  // latch so late costmaps still see the region
    regions_pub_ =
        create_publisher<btcpp_ros2_interfaces::msg::SemanticRegionArray>(
            regions_topic_, pub_qos);

    RCLCPP_INFO(get_logger(),
                "semantic_projection_node up: depth=%s info=%s -> regions=%s "
                "(target_frame=%s)",
                depth_topic_.c_str(), camera_info_topic_.c_str(),
                regions_topic_.c_str(), target_frame_.c_str());
  }

private:
  static double readDepthMeters(const sensor_msgs::msg::Image& img, int u, int v) {
    const int idx = v * static_cast<int>(img.step) +
                    u * (img.encoding == "16UC1" ? 2 : 4);
    if (img.encoding == "16UC1") {
      const uint16_t mm = *reinterpret_cast<const uint16_t*>(&img.data[idx]);
      return mm == 0 ? std::nan("") : mm / 1000.0;
    }
    // default: 32FC1 in meters
    const float m = *reinterpret_cast<const float*>(&img.data[idx]);
    return m;
  }

  // Andrew's monotone-chain convex hull on XY points. Returns >=3 points or {}.
  static std::vector<std::array<double, 2>> convexHull(
      std::vector<std::array<double, 2>> pts) {
    const size_t n = pts.size();
    if (n < 3) {
      return {};
    }
    std::sort(pts.begin(), pts.end());
    auto cross = [](const std::array<double, 2>& o,
                    const std::array<double, 2>& a,
                    const std::array<double, 2>& b) {
      return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    };
    std::vector<std::array<double, 2>> hull(2 * n);
    size_t k = 0;
    for (size_t i = 0; i < n; ++i) {
      while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0) k--;
      hull[k++] = pts[i];
    }
    for (size_t i = n - 1, t = k + 1; i > 0; --i) {
      while (k >= t && cross(hull[k - 2], hull[k - 1], pts[i - 1]) <= 0) k--;
      hull[k++] = pts[i - 1];
    }
    hull.resize(k > 0 ? k - 1 : 0);
    return hull;
  }

  void onDetection(
      const btcpp_ros2_interfaces::msg::SemanticDetection2D::SharedPtr det) {
    sensor_msgs::msg::Image::SharedPtr depth;
    sensor_msgs::msg::CameraInfo::SharedPtr info;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      depth = latest_depth_;
      info = latest_info_;
    }
    if (!depth || !info) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "no depth/camera_info yet; dropping detection");
      return;
    }
    if (depth->encoding != "32FC1" && depth->encoding != "16UC1") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "unsupported depth encoding '%s'",
                           depth->encoding.c_str());
      return;
    }

    const double fx = info->k[0], fy = info->k[4];
    const double cx = info->k[2], cy = info->k[5];
    if (fx == 0.0 || fy == 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "camera_info has zero focal length");
      return;
    }

    const std::string optical_frame = camera_optical_frame_.empty()
                                          ? depth->header.frame_id
                                          : camera_optical_frame_;
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
          target_frame_, optical_frame, tf2::TimePointZero,
          tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF %s->%s failed: %s", optical_frame.c_str(),
                           target_frame_.c_str(), ex.what());
      return;
    }

    const int u0 = std::max(0, det->x);
    const int v0 = std::max(0, det->y);
    const int u1 = std::min(static_cast<int>(depth->width), det->x + det->width);
    const int v1 = std::min(static_cast<int>(depth->height), det->y + det->height);
    const int step = std::max(1, pixel_step_);

    std::vector<std::array<double, 2>> ground_pts;
    for (int v = v0; v < v1; v += step) {
      for (int u = u0; u < u1; u += step) {
        const double z = readDepthMeters(*depth, u, v);
        if (!std::isfinite(z) || z < min_depth_ || z > max_depth_) {
          continue;
        }
        // Pinhole deprojection in the camera optical frame.
        geometry_msgs::msg::PointStamped pin, pout;
        pin.header.frame_id = optical_frame;
        pin.point.x = (u - cx) * z / fx;
        pin.point.y = (v - cy) * z / fy;
        pin.point.z = z;
        tf2::doTransform(pin, pout, tf);
        ground_pts.push_back({pout.point.x, pout.point.y});
      }
    }

    const auto hull = convexHull(std::move(ground_pts));
    if (hull.size() < 3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "detection '%s' produced no valid ground polygon",
                           det->label.c_str());
      return;
    }

    btcpp_ros2_interfaces::msg::SemanticRegion region;
    region.label = det->label;
    region.traversable = det->traversable;
    region.cost = 0.0f;  // authoritative cost is set by the layer's param
    region.polygon.header.stamp = det->header.stamp;
    region.polygon.header.frame_id = target_frame_;
    for (const auto& p : hull) {
      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(p[0]);
      pt.y = static_cast<float>(p[1]);
      pt.z = 0.0f;
      region.polygon.polygon.points.push_back(pt);
    }

    btcpp_ros2_interfaces::msg::SemanticRegionArray out;
    out.header.stamp = now();
    out.header.frame_id = target_frame_;
    out.regions.push_back(region);
    regions_pub_->publish(out);
  }

  std::string depth_topic_, camera_info_topic_, regions_topic_;
  std::string target_frame_, camera_optical_frame_;
  int pixel_step_;
  double min_depth_, max_depth_, transform_tolerance_;

  std::mutex mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_info_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<btcpp_ros2_interfaces::msg::SemanticDetection2D>::SharedPtr
      detection_sub_;
  rclcpp::Publisher<btcpp_ros2_interfaces::msg::SemanticRegionArray>::SharedPtr
      regions_pub_;
};

}  // namespace semantic_traversability

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<semantic_traversability::ProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
