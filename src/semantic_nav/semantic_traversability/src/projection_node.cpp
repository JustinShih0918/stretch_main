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
#include <chrono>
#include <cmath>
#include <map>
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
    // Keep a detected region alive (republished) after the last time it was
    // seen, so a detector drop-out (e.g. when the robot is too close and the
    // object fills the frame) does not wipe the costmap override.
    //   <0  -> hold forever: once detected, the region is never expired.
    //    0  -> no hold: publish only on detection.
    //   >0  -> hold for this many seconds (sim time under use_sim_time).
    region_hold_sec_ = declare_parameter<double>("region_hold_sec", -1.0);
    // Avoid memorizing one-frame false positives. A region is promoted to the
    // held cache only after this many spatially consistent detections.
    region_confirmation_hits_ =
        declare_parameter<int>("region_confirmation_hits", 2);
    region_match_distance_m_ =
        declare_parameter<double>("region_match_distance_m", 0.60);
    pending_region_ttl_sec_ =
        declare_parameter<double>("pending_region_ttl_sec", 5.0);
    // Below this footprint area (m^2) the detection is treated as a thin/vertical
    // surface and widened to min_polygon_thickness (m) so it covers the cells
    // the LiDAR marked LETHAL (a curtain/wall otherwise projects to a 1D line).
    min_polygon_area_ = declare_parameter<double>("min_polygon_area", 0.02);
    min_polygon_thickness_ =
        declare_parameter<double>("min_polygon_thickness", 0.30);

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

    if (region_hold_sec_ != 0.0) {
      // Republish held regions at 5 Hz and expire them once age > hold. Use the
      // node clock (sim time when use_sim_time=true) so the republish cadence
      // and the age test live in the SAME time domain as the cached stamps:
      // consistent under sim time-scaling and safe if the sim is paused.
      publish_timer_ = rclcpp::create_timer(
          this, get_clock(), rclcpp::Duration::from_seconds(0.2),
          std::bind(&ProjectionNode::publishRegions, this));
    }

    RCLCPP_INFO(get_logger(),
                "semantic_projection_node up: depth=%s info=%s -> regions=%s "
                "(target_frame=%s region_hold_sec=%.1f confirmation_hits=%d "
                "match_distance=%.2f)",
                depth_topic_.c_str(), camera_info_topic_.c_str(),
                regions_topic_.c_str(), target_frame_.c_str(),
                region_hold_sec_, region_confirmation_hits_,
                region_match_distance_m_);
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

  // Shoelace area of a polygon (absolute value).
  static double polygonArea(const std::vector<std::array<double, 2>>& poly) {
    if (poly.size() < 3) {
      return 0.0;
    }
    double a = 0.0;
    for (size_t i = 0, n = poly.size(); i < n; ++i) {
      const auto& p = poly[i];
      const auto& q = poly[(i + 1) % n];
      a += p[0] * q[1] - q[0] * p[1];
    }
    return std::abs(a) * 0.5;
  }

  // Oriented bounding box of the points with a guaranteed minimum thickness on
  // each axis. A thin vertical surface (curtain/wall) deprojects to a near-1D
  // line whose convex hull is degenerate; this turns that line into a 2D
  // rectangle wide enough to cover the LETHAL cells the LiDAR marked.
  static std::vector<std::array<double, 2>> minThicknessBox(
      const std::vector<std::array<double, 2>>& pts, double min_thickness) {
    double cx = 0.0, cy = 0.0;
    for (const auto& p : pts) {
      cx += p[0];
      cy += p[1];
    }
    cx /= pts.size();
    cy /= pts.size();

    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (const auto& p : pts) {
      const double dx = p[0] - cx, dy = p[1] - cy;
      sxx += dx * dx;
      sxy += dx * dy;
      syy += dy * dy;
    }
    // Principal axis (u) from the covariance; minor axis (v) is perpendicular.
    const double theta = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
    const double ux = std::cos(theta), uy = std::sin(theta);
    const double vx = -uy, vy = ux;

    double umin = 1e9, umax = -1e9, vmin = 1e9, vmax = -1e9;
    for (const auto& p : pts) {
      const double du = (p[0] - cx) * ux + (p[1] - cy) * uy;
      const double dv = (p[0] - cx) * vx + (p[1] - cy) * vy;
      umin = std::min(umin, du);
      umax = std::max(umax, du);
      vmin = std::min(vmin, dv);
      vmax = std::max(vmax, dv);
    }
    const double half = min_thickness * 0.5;
    if (umax - umin < min_thickness) {
      const double m = (umin + umax) * 0.5;
      umin = m - half;
      umax = m + half;
    }
    if (vmax - vmin < min_thickness) {
      const double m = (vmin + vmax) * 0.5;
      vmin = m - half;
      vmax = m + half;
    }
    auto corner = [&](double du, double dv) -> std::array<double, 2> {
      return {cx + du * ux + dv * vx, cy + du * uy + dv * vy};
    };
    return {corner(umin, vmin), corner(umax, vmin),
            corner(umax, vmax), corner(umin, vmax)};
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

    if (ground_pts.size() < 3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "detection '%s' has too few valid depth points (%zu)",
                           det->label.c_str(), ground_pts.size());
      return;
    }
    // Convex hull for genuine 2D footprints (grass, rugs, ...). For a thin
    // vertical surface (curtain/wall) the points collapse to a line and the
    // hull is degenerate (~zero area); fall back to a minimum-thickness box so
    // the region actually covers the LETHAL cells the LiDAR marked.
    auto hull = convexHull(ground_pts);
    if (polygonArea(hull) < min_polygon_area_) {
      hull = minThicknessBox(ground_pts, min_polygon_thickness_);
    }
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

    // No hold: publish the single fresh region immediately (legacy behavior).
    if (region_hold_sec_ == 0.0) {
      btcpp_ros2_interfaces::msg::SemanticRegionArray out;
      out.header.stamp = now();
      out.header.frame_id = target_frame_;
      out.regions.push_back(region);
      regions_pub_->publish(out);
      return;
    }

    // Hold enabled: promote only repeated, spatially consistent detections to
    // persistent memory. This prevents one-frame false positives from clearing
    // cost forever, while confirmed objects survive close-range detector dropouts.
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      updateRegionMemory(region, now());
    }
    publishRegions();
  }

  static std::array<double, 2> polygonCentroid(
      const btcpp_ros2_interfaces::msg::SemanticRegion& region) {
    double x = 0.0;
    double y = 0.0;
    const auto& points = region.polygon.polygon.points;
    for (const auto& p : points) {
      x += p.x;
      y += p.y;
    }
    const double n = static_cast<double>(points.size());
    return {x / n, y / n};
  }

  bool sameObject(const std::array<double, 2>& a,
                  const std::array<double, 2>& b) const {
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    return dx * dx + dy * dy <=
           region_match_distance_m_ * region_match_distance_m_;
  }

  void updateRegionMemory(
      const btcpp_ros2_interfaces::msg::SemanticRegion& region,
      const rclcpp::Time& stamp) {
    const auto centroid = polygonCentroid(region);

    auto confirmed = region_cache_.find(region.label);
    if (confirmed != region_cache_.end() &&
        sameObject(centroid, confirmed->second.centroid)) {
      confirmed->second.region = region;
      confirmed->second.stamp = stamp;
      confirmed->second.centroid = centroid;
      confirmed->second.hits++;
      return;
    }

    const int required_hits = std::max(1, region_confirmation_hits_);
    auto pending = pending_regions_.find(region.label);
    if (pending == pending_regions_.end() ||
        !sameObject(centroid, pending->second.centroid) ||
        (pending_region_ttl_sec_ >= 0.0 &&
         (stamp - pending->second.stamp).seconds() > pending_region_ttl_sec_)) {
      pending_regions_[region.label] =
          CachedRegion{region, stamp, centroid, 1};
      pending = pending_regions_.find(region.label);
    } else {
      pending->second.region = region;
      pending->second.stamp = stamp;
      pending->second.centroid = centroid;
      pending->second.hits++;
    }

    if (pending->second.hits >= required_hits) {
      region_cache_[region.label] = pending->second;
      pending_regions_.erase(pending);
      RCLCPP_INFO(get_logger(),
                  "confirmed semantic region '%s' after %d detections",
                  region.label.c_str(), required_hits);
    }
  }

  // Republish every cached region younger than region_hold_sec_, dropping the
  // expired ones. Held regions are re-stamped to now(): they live in the stable
  // target frame, so this keeps them inside the costmap's transform tolerance.
  void publishRegions() {
    const rclcpp::Time tnow = now();
    btcpp_ros2_interfaces::msg::SemanticRegionArray out;
    out.header.stamp = tnow;
    out.header.frame_id = target_frame_;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      for (auto it = pending_regions_.begin(); it != pending_regions_.end();) {
        const bool expired =
            pending_region_ttl_sec_ >= 0.0 &&
            (tnow - it->second.stamp).seconds() > pending_region_ttl_sec_;
        if (expired) {
          it = pending_regions_.erase(it);
        } else {
          ++it;
        }
      }
      for (auto it = region_cache_.begin(); it != region_cache_.end();) {
        const bool expired = region_hold_sec_ >= 0.0 &&
                             (tnow - it->second.stamp).seconds() > region_hold_sec_;
        if (expired) {
          it = region_cache_.erase(it);
        } else {
          auto region = it->second.region;
          region.polygon.header.stamp = tnow;
          out.regions.push_back(std::move(region));
          ++it;
        }
      }
    }
    regions_pub_->publish(out);
  }

  struct CachedRegion {
    btcpp_ros2_interfaces::msg::SemanticRegion region;
    rclcpp::Time stamp;
    std::array<double, 2> centroid;
    int hits{0};
  };

  std::string depth_topic_, camera_info_topic_, regions_topic_;
  std::string target_frame_, camera_optical_frame_;
  int pixel_step_, region_confirmation_hits_;
  double min_depth_, max_depth_, transform_tolerance_, region_hold_sec_;
  double min_polygon_area_, min_polygon_thickness_;
  double region_match_distance_m_, pending_region_ttl_sec_;

  std::mutex mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_info_;

  std::mutex cache_mutex_;
  std::map<std::string, CachedRegion> region_cache_;
  std::map<std::string, CachedRegion> pending_regions_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

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
