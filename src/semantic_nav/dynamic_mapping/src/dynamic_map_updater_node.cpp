#include "dynamic_mapping/dynamic_map_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <btcpp_ros2_interfaces/msg/semantic_region_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace dynamic_mapping {
namespace {

GridGeometry geometryFromMap(const nav_msgs::msg::OccupancyGrid& map) {
  GridGeometry geometry;
  geometry.width = map.info.width;
  geometry.height = map.info.height;
  geometry.resolution = map.info.resolution;
  geometry.origin_x = map.info.origin.position.x;
  geometry.origin_y = map.info.origin.position.y;
  return geometry;
}

bool isOccupied(int8_t value, int occupied_threshold) {
  return value >= occupied_threshold;
}

double squaredDistance(double ax, double ay, double bx, double by) {
  const double dx = ax - bx;
  const double dy = ay - by;
  return dx * dx + dy * dy;
}

}  // namespace

class DynamicMapUpdaterNode : public rclcpp::Node {
public:
  DynamicMapUpdaterNode() : rclcpp::Node("dynamic_map_updater_node") {
    raw_map_topic_ =
        declare_parameter<std::string>("raw_map_topic", "/rtabmap/map");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/laser_scan");
    semantic_regions_topic_ =
        declare_parameter<std::string>("semantic_regions_topic",
                                       "/semantic_regions");
    output_map_topic_ =
        declare_parameter<std::string>("output_map_topic", "/map");
    cleared_cells_topic_ = declare_parameter<std::string>(
        "cleared_cells_topic", "/dynamic_map/cleared_cells");
    change_events_topic_ = declare_parameter<std::string>(
        "change_events_topic", "/dynamic_map/change_events");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    transform_tolerance_ =
        declare_parameter<double>("transform_tolerance", 0.2);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 65);
    ray_pass_through_margin_m_ =
        declare_parameter<double>("ray_pass_through_margin_m", 0.15);
    publish_change_events_ =
        declare_parameter<bool>("publish_change_events", true);

    auto labels = declare_parameter<std::vector<std::string>>(
        "dynamic_labels", {"person", "chair", "cart", "box"});
    dynamic_labels_ = normalizedLabelSet(labels);

    enable_traversable_persistence_ =
        declare_parameter<bool>("enable_traversable_persistence", true);
    auto traversable_labels = declare_parameter<std::vector<std::string>>(
        "traversable_labels", {"curtain", "grass"});
    traversable_labels_ = normalizedLabelSet(traversable_labels);
    traversable_confidence_radius_m_ =
        declare_parameter<double>("traversable_confidence_radius_m", 0.10);

    EvidenceThresholds thresholds;
    thresholds.min_absence_hits =
        declare_parameter<int>("min_absence_hits", 5);
    thresholds.min_distinct_poses =
        declare_parameter<int>("min_distinct_poses", 3);
    thresholds.min_pose_separation_m =
        declare_parameter<double>("min_pose_separation_m", 0.25);
    evidence_.setThresholds(thresholds);

    EvidenceThresholds traversable_thresholds;
    traversable_thresholds.min_absence_hits =
        declare_parameter<int>("min_traversable_observations", 4);
    traversable_thresholds.min_distinct_poses =
        declare_parameter<int>("min_traversable_distinct_poses", 3);
    traversable_thresholds.min_pose_separation_m =
        declare_parameter<double>("traversable_min_pose_separation_m", 0.25);
    traversable_evidence_.setThresholds(traversable_thresholds);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();
    raw_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        raw_map_topic_, map_qos,
        std::bind(&DynamicMapUpdaterNode::onRawMap, this,
                  std::placeholders::_1));

    rclcpp::SensorDataQoS scan_qos;
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, scan_qos,
        std::bind(&DynamicMapUpdaterNode::onScan, this, std::placeholders::_1));

    rclcpp::QoS regions_qos(rclcpp::KeepLast(1));
    regions_qos.transient_local();
    regions_sub_ = create_subscription<
        btcpp_ros2_interfaces::msg::SemanticRegionArray>(
        semantic_regions_topic_, regions_qos,
        std::bind(&DynamicMapUpdaterNode::onSemanticRegions, this,
                  std::placeholders::_1));

    cleaned_map_pub_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>(output_map_topic_,
                                                       map_qos);
    cleared_cells_pub_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>(cleared_cells_topic_,
                                                       map_qos);
    change_events_pub_ =
        create_publisher<std_msgs::msg::String>(change_events_topic_, 10);

    RCLCPP_INFO(get_logger(),
                "dynamic_map_updater_node up: raw_map=%s scan=%s regions=%s "
                "-> map=%s (dynamic_labels=%zu traversable_labels=%zu)",
                raw_map_topic_.c_str(), scan_topic_.c_str(),
                semantic_regions_topic_.c_str(), output_map_topic_.c_str(),
                dynamic_labels_.size(), traversable_labels_.size());
  }

private:
  void onRawMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const GridGeometry new_geometry = geometryFromMap(*msg);
    const bool geometry_changed = !raw_map_ || !sameGeometry(geometry_, new_geometry);
    raw_map_ = *msg;
    geometry_ = new_geometry;
    map_frame_ =
        raw_map_->header.frame_id.empty() ? map_frame_ : raw_map_->header.frame_id;

    if (geometry_changed) {
      evidence_.reset(raw_map_->data.size());
      traversable_evidence_.reset(raw_map_->data.size());
      candidate_cells_.assign(raw_map_->data.size(), false);
      traversable_candidate_cells_.assign(raw_map_->data.size(), false);
      rebuildCandidateMaskLocked();
      RCLCPP_INFO(get_logger(),
                  "raw map geometry updated: %ux%u res=%.3f frame=%s",
                  geometry_.width, geometry_.height, geometry_.resolution,
                  map_frame_.c_str());
    } else {
      resetEvidenceForNonOccupiedLocked();
    }
    publishMapsLocked();
  }

  void onSemanticRegions(
      const btcpp_ros2_interfaces::msg::SemanticRegionArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_regions_ = msg;
    rebuildCandidateMaskLocked();
    accumulateTraversableEvidenceLocked();
    publishMapsLocked();
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!raw_map_) {
      return;
    }
    if (geometry_.width == 0 || geometry_.height == 0 ||
        geometry_.resolution <= 0.0) {
      return;
    }

    geometry_msgs::msg::TransformStamped map_from_base;
    geometry_msgs::msg::TransformStamped map_from_scan;
    try {
      const auto tolerance = tf2::durationFromSec(transform_tolerance_);
      map_from_base = tf_buffer_->lookupTransform(
          map_frame_, base_frame_, tf2::TimePointZero, tolerance);
      map_from_scan = tf_buffer_->lookupTransform(
          map_frame_, scan->header.frame_id, tf2::TimePointZero, tolerance);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF lookup failed for scan update: %s", ex.what());
      return;
    }

    const Pose2D robot_pose{map_from_base.transform.translation.x,
                            map_from_base.transform.translation.y};
    const double scan_origin_x = map_from_scan.transform.translation.x;
    const double scan_origin_y = map_from_scan.transform.translation.y;

    Cell start_cell;
    if (!worldToGrid(geometry_, scan_origin_x, scan_origin_y, start_cell)) {
      return;
    }

    std::size_t newly_cleared = 0;
    double angle = scan->angle_min;
    for (const float range : scan->ranges) {
      processScanRayLocked(*scan, range, angle, scan_origin_x, scan_origin_y,
                           start_cell, map_from_scan, robot_pose,
                           &newly_cleared);
      angle += scan->angle_increment;
    }

    if (newly_cleared > 0 && publish_change_events_) {
      std_msgs::msg::String event;
      event.data = "cleared_cells=" + std::to_string(newly_cleared) +
                   " total_cleared=" + std::to_string(evidence_.clearedCount());
      change_events_pub_->publish(event);
    }
    publishMapsLocked();
  }

  void processScanRayLocked(
      const sensor_msgs::msg::LaserScan& scan, float range, double angle,
      double scan_origin_x, double scan_origin_y, const Cell& start_cell,
      const geometry_msgs::msg::TransformStamped& map_from_scan,
      const Pose2D& robot_pose, std::size_t* newly_cleared) {
    if (!std::isfinite(range) || range < scan.range_min ||
        range > scan.range_max) {
      return;
    }

    geometry_msgs::msg::PointStamped endpoint_scan;
    endpoint_scan.header = scan.header;
    endpoint_scan.point.x = static_cast<double>(range) * std::cos(angle);
    endpoint_scan.point.y = static_cast<double>(range) * std::sin(angle);
    endpoint_scan.point.z = 0.0;

    geometry_msgs::msg::PointStamped endpoint_map;
    tf2::doTransform(endpoint_scan, endpoint_map, map_from_scan);

    Cell end_cell;
    if (!worldToGrid(geometry_, endpoint_map.point.x, endpoint_map.point.y,
                     end_cell)) {
      return;
    }

    const auto endpoint_index = cellIndex(geometry_, end_cell);
    if (endpoint_index && isCandidateOccupied(*endpoint_index)) {
      evidence_.resetEvidence(*endpoint_index);
    }

    const double max_free_distance =
        std::max(0.0, static_cast<double>(range) - ray_pass_through_margin_m_);
    const double max_free_distance_sq = max_free_distance * max_free_distance;
    for (const auto& cell : raytraceCells(start_cell, end_cell)) {
      const auto index = cellIndex(geometry_, cell);
      if (!index || !isCandidateOccupied(*index)) {
        continue;
      }
      const auto center = gridToWorldCenter(geometry_, cell);
      if (squaredDistance(scan_origin_x, scan_origin_y, center.first,
                          center.second) > max_free_distance_sq) {
        continue;
      }

      const bool was_cleared = evidence_.cleared(*index);
      evidence_.addFreeSpaceEvidence(*index, robot_pose);
      if (!was_cleared && evidence_.cleared(*index)) {
        (*newly_cleared)++;
      }
    }
  }

  bool isCandidateOccupied(std::size_t index) const {
    return raw_map_ && index < raw_map_->data.size() &&
           index < candidate_cells_.size() &&
           candidate_cells_[index] &&
           isOccupied(raw_map_->data[index], occupied_threshold_);
  }

  void rebuildCandidateMaskLocked() {
    if (!raw_map_) {
      return;
    }
    if (geometry_.width == 0 || geometry_.height == 0 ||
        geometry_.resolution <= 0.0) {
      return;
    }

    candidate_cells_.assign(raw_map_->data.size(), false);
    traversable_candidate_cells_.assign(raw_map_->data.size(), false);
    if (!latest_regions_) {
      return;
    }

    for (const auto& region : latest_regions_->regions) {
      const auto label = normalizeLabel(region.label);
      if (dynamic_labels_.find(label) != dynamic_labels_.end()) {
        fillRegionCellsLocked(region, &candidate_cells_);
      }
      if (shouldUseTraversableRegion(region)) {
        fillRegionCellsLocked(region, &traversable_candidate_cells_);
      }
    }

    resetEvidenceOutsideCandidatesLocked();
    resetTraversableEvidenceOutsideCandidatesLocked();
  }

  void clampWorldToCellLocked(double wx, double wy, Cell& cell) const {
    const int mx = static_cast<int>(
        std::floor((wx - geometry_.origin_x) / geometry_.resolution));
    const int my = static_cast<int>(
        std::floor((wy - geometry_.origin_y) / geometry_.resolution));
    cell.x = std::clamp(mx, 0, static_cast<int>(geometry_.width) - 1);
    cell.y = std::clamp(my, 0, static_cast<int>(geometry_.height) - 1);
  }

  void resetEvidenceForNonOccupiedLocked() {
    for (std::size_t i = 0; i < raw_map_->data.size(); ++i) {
      if (!isOccupied(raw_map_->data[i], occupied_threshold_)) {
        evidence_.resetEvidence(i);
        traversable_evidence_.resetEvidence(i);
      }
    }
  }

  void resetEvidenceOutsideCandidatesLocked() {
    for (std::size_t i = 0; i < candidate_cells_.size(); ++i) {
      if (!candidate_cells_[i]) {
        evidence_.resetEvidence(i);
      }
    }
  }

  void resetTraversableEvidenceOutsideCandidatesLocked() {
    for (std::size_t i = 0; i < traversable_candidate_cells_.size(); ++i) {
      if (!traversable_candidate_cells_[i] && !traversable_evidence_.cleared(i)) {
        traversable_evidence_.resetEvidence(i);
      }
    }
  }

  bool shouldUseTraversableRegion(
      const btcpp_ros2_interfaces::msg::SemanticRegion& region) const {
    if (!enable_traversable_persistence_ || !region.traversable) {
      return false;
    }
    if (traversable_labels_.empty()) {
      return true;
    }
    return traversable_labels_.find(normalizeLabel(region.label)) !=
           traversable_labels_.end();
  }

  bool fillRegionCellsLocked(
      const btcpp_ros2_interfaces::msg::SemanticRegion& region,
      std::vector<bool>* cells) {
    if (region.polygon.polygon.points.size() < 3 || !raw_map_ ||
        geometry_.width == 0 || geometry_.height == 0 ||
        geometry_.resolution <= 0.0) {
      return false;
    }

    const auto tolerance = tf2::durationFromSec(transform_tolerance_);
    geometry_msgs::msg::TransformStamped map_from_region;
    try {
      map_from_region = tf_buffer_->lookupTransform(
          map_frame_, region.polygon.header.frame_id, tf2::TimePointZero,
          tolerance);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF %s->%s failed for semantic region '%s': %s",
                           region.polygon.header.frame_id.c_str(),
                           map_frame_.c_str(), region.label.c_str(),
                           ex.what());
      return false;
    }

    std::vector<std::pair<double, double>> polygon;
    polygon.reserve(region.polygon.polygon.points.size());
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    for (const auto& point : region.polygon.polygon.points) {
      geometry_msgs::msg::PointStamped in;
      geometry_msgs::msg::PointStamped out;
      in.header = region.polygon.header;
      in.point.x = point.x;
      in.point.y = point.y;
      in.point.z = point.z;
      tf2::doTransform(in, out, map_from_region);
      polygon.push_back({out.point.x, out.point.y});
      min_x = std::min(min_x, out.point.x);
      min_y = std::min(min_y, out.point.y);
      max_x = std::max(max_x, out.point.x);
      max_y = std::max(max_y, out.point.y);
    }

    min_x -= traversable_confidence_radius_m_;
    min_y -= traversable_confidence_radius_m_;
    max_x += traversable_confidence_radius_m_;
    max_y += traversable_confidence_radius_m_;

    Cell min_cell;
    Cell max_cell;
    if (!worldToGrid(geometry_, min_x, min_y, min_cell)) {
      clampWorldToCellLocked(min_x, min_y, min_cell);
    }
    if (!worldToGrid(geometry_, max_x, max_y, max_cell)) {
      clampWorldToCellLocked(max_x, max_y, max_cell);
    }

    const int x0 = std::clamp(std::min(min_cell.x, max_cell.x), 0,
                              static_cast<int>(geometry_.width) - 1);
    const int x1 = std::clamp(std::max(min_cell.x, max_cell.x), 0,
                              static_cast<int>(geometry_.width) - 1);
    const int y0 = std::clamp(std::min(min_cell.y, max_cell.y), 0,
                              static_cast<int>(geometry_.height) - 1);
    const int y1 = std::clamp(std::max(min_cell.y, max_cell.y), 0,
                              static_cast<int>(geometry_.height) - 1);

    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const Cell cell{x, y};
        const auto center = gridToWorldCenter(geometry_, cell);
        bool inside = pointInPolygon(center.first, center.second, polygon);
        if (!inside && traversable_confidence_radius_m_ > 0.0) {
          for (const auto& p : polygon) {
            if (squaredDistance(center.first, center.second, p.first,
                                p.second) <=
                traversable_confidence_radius_m_ *
                    traversable_confidence_radius_m_) {
              inside = true;
              break;
            }
          }
        }
        if (!inside) {
          continue;
        }
        if (const auto index = cellIndex(geometry_, cell)) {
          (*cells)[*index] = true;
        }
      }
    }
    return true;
  }

  std::optional<Pose2D> lookupRobotPoseLocked() {
    try {
      const auto tf = tf_buffer_->lookupTransform(
          map_frame_, base_frame_, tf2::TimePointZero,
          tf2::durationFromSec(transform_tolerance_));
      return Pose2D{tf.transform.translation.x, tf.transform.translation.y};
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF lookup failed for semantic evidence: %s",
                           ex.what());
      return std::nullopt;
    }
  }

  void accumulateTraversableEvidenceLocked() {
    if (!enable_traversable_persistence_ || !raw_map_ || !latest_regions_) {
      return;
    }
    const auto robot_pose = lookupRobotPoseLocked();
    if (!robot_pose) {
      return;
    }

    std::size_t newly_confirmed = 0;
    for (std::size_t i = 0; i < traversable_candidate_cells_.size(); ++i) {
      if (!traversable_candidate_cells_[i] ||
          !isOccupied(raw_map_->data[i], occupied_threshold_)) {
        continue;
      }
      const bool was_cleared = traversable_evidence_.cleared(i);
      traversable_evidence_.addFreeSpaceEvidence(i, *robot_pose);
      if (!was_cleared && traversable_evidence_.cleared(i)) {
        newly_confirmed++;
      }
    }

    if (newly_confirmed > 0 && publish_change_events_) {
      std_msgs::msg::String event;
      event.data = "confirmed_traversable_cells=" +
                   std::to_string(newly_confirmed) +
                   " total_confirmed_traversable=" +
                   std::to_string(traversable_evidence_.clearedCount());
      change_events_pub_->publish(event);
    }
  }

  void publishMapsLocked() {
    if (!raw_map_) {
      return;
    }

    auto cleaned = *raw_map_;
    auto debug = *raw_map_;
    std::fill(debug.data.begin(), debug.data.end(), static_cast<int8_t>(-1));
    cleaned.header.stamp = now();
    debug.header.stamp = cleaned.header.stamp;

    for (std::size_t i = 0; i < cleaned.data.size(); ++i) {
      if ((isCandidateOccupied(i) && evidence_.cleared(i)) ||
          (isOccupied(raw_map_->data[i], occupied_threshold_) &&
           traversable_evidence_.cleared(i))) {
        cleaned.data[i] = 0;
        debug.data[i] = 0;
      }
    }

    cleaned_map_pub_->publish(cleaned);
    cleared_cells_pub_->publish(debug);
  }

  std::mutex mutex_;
  std::optional<nav_msgs::msg::OccupancyGrid> raw_map_;
  GridGeometry geometry_;
  EvidenceGrid evidence_;
  EvidenceGrid traversable_evidence_;
  std::vector<bool> candidate_cells_;
  std::vector<bool> traversable_candidate_cells_;
  btcpp_ros2_interfaces::msg::SemanticRegionArray::SharedPtr latest_regions_;
  std::unordered_set<std::string> dynamic_labels_;
  std::unordered_set<std::string> traversable_labels_;

  std::string raw_map_topic_;
  std::string scan_topic_;
  std::string semantic_regions_topic_;
  std::string output_map_topic_;
  std::string cleared_cells_topic_;
  std::string change_events_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double transform_tolerance_{0.2};
  int occupied_threshold_{65};
  double ray_pass_through_margin_m_{0.15};
  double traversable_confidence_radius_m_{0.10};
  bool publish_change_events_{true};
  bool enable_traversable_persistence_{true};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<
      btcpp_ros2_interfaces::msg::SemanticRegionArray>::SharedPtr regions_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cleaned_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cleared_cells_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr change_events_pub_;
};

}  // namespace dynamic_mapping

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dynamic_mapping::DynamicMapUpdaterNode>());
  rclcpp::shutdown();
  return 0;
}
