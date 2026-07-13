#include "semantic_traversability/semantic_traversability_layer.hpp"

#include <algorithm>
#include <limits>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace semantic_traversability {

void SemanticTraversabilityLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"SemanticTraversabilityLayer: node is null"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("polygon_topic", rclcpp::ParameterValue("/semantic_regions"));
  declareParameter("traversable_cost", rclcpp::ParameterValue(-1.0));
  declareParameter("override_threshold", rclcpp::ParameterValue(254));
  declareParameter("clear_unknown", rclcpp::ParameterValue(false));
  declareParameter("clear_margin_m", rclcpp::ParameterValue(0.0));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.2));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".polygon_topic", polygon_topic_);
  node->get_parameter(name_ + ".traversable_cost", traversable_cost_);
  int override_threshold = 254;
  node->get_parameter(name_ + ".override_threshold", override_threshold);
  override_threshold_ =
      static_cast<unsigned char>(std::clamp(override_threshold, 0, 255));
  node->get_parameter(name_ + ".clear_unknown", clear_unknown_);
  node->get_parameter(name_ + ".clear_margin_m", clear_margin_m_);
  clear_margin_m_ = std::max(0.0, clear_margin_m_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local();  // latch the last regions for late-joining costmaps
  regions_sub_ = node->create_subscription<
      btcpp_ros2_interfaces::msg::SemanticRegionArray>(
      polygon_topic_, qos,
      std::bind(&SemanticTraversabilityLayer::regionsCallback, this,
                std::placeholders::_1));

  // Watch runtime parameter changes so ros2 param set takes effect immediately.
  param_cb_handle_ = node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params)
          -> rcl_interfaces::msg::SetParametersResult {
        const std::string prefix = name_ + ".";
        for (const auto& p : params) {
          if (p.get_name().rfind(prefix, 0) != 0) continue;
          const std::string local = p.get_name().substr(prefix.size());
          if (local == "enabled") {
            enabled_ = p.as_bool();
          } else if (local == "traversable_cost") {
            traversable_cost_ = p.as_double();
          } else if (local == "override_threshold") {
            override_threshold_ = static_cast<unsigned char>(
                std::clamp(static_cast<int>(p.as_int()), 0, 255));
          } else if (local == "clear_unknown") {
            clear_unknown_ = p.as_bool();
          } else if (local == "clear_margin_m") {
            clear_margin_m_ = std::max(0.0, p.as_double());
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

  current_ = true;
  RCLCPP_INFO(node->get_logger(),
              "[%s] SemanticTraversabilityLayer up: topic=%s traversable_cost=%.0f "
              "override_threshold=%d clear_unknown=%s clear_margin_m=%.2f",
              name_.c_str(), polygon_topic_.c_str(), traversable_cost_,
              override_threshold_, clear_unknown_ ? "true" : "false",
              clear_margin_m_);
}

void SemanticTraversabilityLayer::regionsCallback(
    const btcpp_ros2_interfaces::msg::SemanticRegionArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_regions_ = msg;
}

bool SemanticTraversabilityLayer::pointInPolygon(
    double x, double y,
    const std::vector<geometry_msgs::msg::Point32>& poly) {
  // Standard ray-casting point-in-polygon test.
  bool inside = false;
  const size_t n = poly.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = poly[i].x, yi = poly[i].y;
    const double xj = poly[j].x, yj = poly[j].y;
    const bool intersect = ((yi > y) != (yj > y)) &&
                           (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

double SemanticTraversabilityLayer::distanceToSegmentSquared(
    double x, double y, const geometry_msgs::msg::Point32& a,
    const geometry_msgs::msg::Point32& b) {
  const double ax = a.x;
  const double ay = a.y;
  const double bx = b.x;
  const double by = b.y;
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len_sq = dx * dx + dy * dy;
  if (len_sq <= std::numeric_limits<double>::epsilon()) {
    const double px = x - ax;
    const double py = y - ay;
    return px * px + py * py;
  }
  const double t =
      std::clamp(((x - ax) * dx + (y - ay) * dy) / len_sq, 0.0, 1.0);
  const double sx = ax + t * dx;
  const double sy = ay + t * dy;
  const double px = x - sx;
  const double py = y - sy;
  return px * px + py * py;
}

bool SemanticTraversabilityLayer::pointNearPolygon(
    double x, double y,
    const std::vector<geometry_msgs::msg::Point32>& poly,
    double margin_m) {
  if (margin_m <= 0.0 || poly.size() < 2) {
    return false;
  }
  const double margin_sq = margin_m * margin_m;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    if (distanceToSegmentSquared(x, y, poly[j], poly[i]) <= margin_sq) {
      return true;
    }
  }
  return false;
}

std::vector<SemanticTraversabilityLayer::GroundRegion>
SemanticTraversabilityLayer::transformRegions(double* min_x, double* min_y,
                                              double* max_x, double* max_y) {
  std::vector<GroundRegion> out;
  btcpp_ros2_interfaces::msg::SemanticRegionArray::SharedPtr regions;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    regions = latest_regions_;
  }
  if (!regions) {
    return out;
  }

  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  const auto tolerance =
      tf2::durationFromSec(transform_tolerance_);

  for (const auto& region : regions->regions) {
    if (!region.traversable || region.polygon.polygon.points.size() < 3) {
      continue;
    }

    const std::string& src_frame = region.polygon.header.frame_id;
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_->lookupTransform(global_frame, src_frame,
                                tf2::TimePointZero, tolerance);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(node_.lock()->get_logger(), *node_.lock()->get_clock(),
                           2000, "[%s] TF %s->%s failed: %s", name_.c_str(),
                           src_frame.c_str(), global_frame.c_str(), ex.what());
      continue;
    }

    GroundRegion gr;
    const double cost = traversable_cost_ >= 0.0 ? traversable_cost_
                                                 : static_cast<double>(region.cost);
    gr.cost = static_cast<unsigned char>(
        std::clamp(static_cast<int>(cost), 0, 254));
    gr.points.reserve(region.polygon.polygon.points.size());
    double region_min_x = std::numeric_limits<double>::max();
    double region_min_y = std::numeric_limits<double>::max();
    double region_max_x = std::numeric_limits<double>::lowest();
    double region_max_y = std::numeric_limits<double>::lowest();
    for (const auto& p : region.polygon.polygon.points) {
      geometry_msgs::msg::PointStamped in, out_pt;
      in.header = region.polygon.header;
      in.point.x = p.x;
      in.point.y = p.y;
      in.point.z = p.z;
      tf2::doTransform(in, out_pt, tf);

      geometry_msgs::msg::Point32 gp;
      gp.x = static_cast<float>(out_pt.point.x);
      gp.y = static_cast<float>(out_pt.point.y);
      gp.z = 0.0f;
      gr.points.push_back(gp);

      region_min_x = std::min(region_min_x, out_pt.point.x);
      region_min_y = std::min(region_min_y, out_pt.point.y);
      region_max_x = std::max(region_max_x, out_pt.point.x);
      region_max_y = std::max(region_max_y, out_pt.point.y);
    }
    *min_x = std::min(*min_x, region_min_x - clear_margin_m_);
    *min_y = std::min(*min_y, region_min_y - clear_margin_m_);
    *max_x = std::max(*max_x, region_max_x + clear_margin_m_);
    *max_y = std::max(*max_y, region_max_y + clear_margin_m_);
    out.push_back(std::move(gr));
  }
  return out;
}

void SemanticTraversabilityLayer::updateBounds(double /*robot_x*/,
                                               double /*robot_y*/,
                                               double /*robot_yaw*/,
                                               double* min_x, double* min_y,
                                               double* max_x, double* max_y) {
  if (!enabled_) {
    return;
  }
  // transformRegions() is also called in updateCosts(); here we only need it to
  // expand the update window so the master grid lets us write those cells.
  transformRegions(min_x, min_y, max_x, max_y);
  current_ = true;
}

void SemanticTraversabilityLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
    int max_j) {
  if (!enabled_) {
    return;
  }

  double dummy_min = std::numeric_limits<double>::max();
  double dummy_max = std::numeric_limits<double>::lowest();
  const auto regions =
      transformRegions(&dummy_min, &dummy_min, &dummy_max, &dummy_max);
  if (regions.empty()) {
    return;
  }

  unsigned char* costmap = master_grid.getCharMap();

  for (const auto& region : regions) {
    // Compute the cell-space bounding box of this polygon, clamped to the
    // window nav2 handed us.
    double wmin_x = std::numeric_limits<double>::max();
    double wmin_y = std::numeric_limits<double>::max();
    double wmax_x = std::numeric_limits<double>::lowest();
    double wmax_y = std::numeric_limits<double>::lowest();
    for (const auto& p : region.points) {
      wmin_x = std::min(wmin_x, static_cast<double>(p.x));
      wmin_y = std::min(wmin_y, static_cast<double>(p.y));
      wmax_x = std::max(wmax_x, static_cast<double>(p.x));
      wmax_y = std::max(wmax_y, static_cast<double>(p.y));
    }
    wmin_x -= clear_margin_m_;
    wmin_y -= clear_margin_m_;
    wmax_x += clear_margin_m_;
    wmax_y += clear_margin_m_;

    // Clamp the polygon's world bbox to map cells (handles partial overlap).
    int mx0, my0, mx1, my1;
    master_grid.worldToMapEnforceBounds(wmin_x, wmin_y, mx0, my0);
    master_grid.worldToMapEnforceBounds(wmax_x, wmax_y, mx1, my1);

    const int i0 = std::max(mx0, min_i);
    const int j0 = std::max(my0, min_j);
    const int i1 = std::min(mx1 + 1, max_i);
    const int j1 = std::min(my1 + 1, max_j);

    for (int j = j0; j < j1; ++j) {
      for (int i = i0; i < i1; ++i) {
        const unsigned int index = master_grid.getIndex(i, j);
        if (!clear_unknown_ &&
            costmap[index] == nav2_costmap_2d::NO_INFORMATION) {
          continue;
        }
        if (costmap[index] < override_threshold_) {
          continue;  // only clear cells at/above configured cost threshold
        }
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);
        if (pointInPolygon(wx, wy, region.points) ||
            pointNearPolygon(wx, wy, region.points, clear_margin_m_)) {
          costmap[index] = region.cost;
        }
      }
    }
  }
}

void SemanticTraversabilityLayer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_regions_.reset();
}

}  // namespace semantic_traversability

PLUGINLIB_EXPORT_CLASS(semantic_traversability::SemanticTraversabilityLayer,
                       nav2_costmap_2d::Layer)
