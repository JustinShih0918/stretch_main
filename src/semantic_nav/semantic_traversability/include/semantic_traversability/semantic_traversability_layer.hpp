#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>

#include "btcpp_ros2_interfaces/msg/semantic_region_array.hpp"

namespace semantic_traversability {

// Action-aware costmap layer (paper 2310.08873). Runs AFTER the LiDAR
// ObstacleLayer and BEFORE the InflationLayer: it overwrites LETHAL cells that
// fall inside a "traversable" semantic polygon with a lower, configurable cost,
// so the planner can route through traversable obstacles (curtains, grass).
class SemanticTraversabilityLayer : public nav2_costmap_2d::Layer {
public:
  SemanticTraversabilityLayer() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i,
                   int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return false; }

private:
  // A semantic polygon already transformed into this layer's global frame (XY).
  struct GroundRegion {
    std::vector<geometry_msgs::msg::Point32> points;  // ground-plane polygon
    unsigned char cost;                               // value to write
  };

  void regionsCallback(
      const btcpp_ros2_interfaces::msg::SemanticRegionArray::SharedPtr msg);

  // Transform every traversable region into the costmap global frame. Returns
  // the regions that transformed successfully and the union of their XY bounds.
  std::vector<GroundRegion> transformRegions(double* min_x, double* min_y,
                                             double* max_x, double* max_y);

  static bool pointInPolygon(double x, double y,
                             const std::vector<geometry_msgs::msg::Point32>& poly);

  rclcpp::Subscription<btcpp_ros2_interfaces::msg::SemanticRegionArray>::SharedPtr
      regions_sub_;

  std::mutex mutex_;
  btcpp_ros2_interfaces::msg::SemanticRegionArray::SharedPtr latest_regions_;

  // Parameters
  std::string polygon_topic_;
  double traversable_cost_{-1.0};      // <0 = honor each region's cost field;
                                       // >=0 = force this cost (0 = FREESPACE, paper)
  unsigned char override_threshold_{254};  // only overwrite cells >= this cost
  double transform_tolerance_{0.2};
};

}  // namespace semantic_traversability
