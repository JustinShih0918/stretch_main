#include "bt_nav/navigate_to_pose_action.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace bt_nav {

NavigateToPoseAction::NavigateToPoseAction(const std::string& name,
                                           const BT::NodeConfig& conf,
                                           const BT::RosNodeParams& params)
    : BT::RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params) {}

BT::PortsList NavigateToPoseAction::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<double>("x", "Target x in frame_id"),
      BT::InputPort<double>("y", "Target y in frame_id"),
      BT::InputPort<double>("yaw", 0.0, "Target yaw in radians"),
      BT::InputPort<std::string>("frame_id", "map", "Reference frame"),
  });
}

bool NavigateToPoseAction::setGoal(Goal& goal) {
  double x = 0.0, y = 0.0, yaw = 0.0;
  std::string frame_id = "map";

  if (!getInput("x", x)) {
    RCLCPP_ERROR(logger(), "[NavigateToPose] missing required port 'x'");
    return false;
  }
  if (!getInput("y", y)) {
    RCLCPP_ERROR(logger(), "[NavigateToPose] missing required port 'y'");
    return false;
  }
  getInput("yaw", yaw);
  getInput("frame_id", frame_id);

  auto node = node_.lock();
  goal.pose.header.frame_id = frame_id;
  goal.pose.header.stamp = node ? node->now() : rclcpp::Time(0);
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  goal.pose.pose.orientation = tf2::toMsg(q);

  RCLCPP_INFO(logger(),
              "[NavigateToPose] sending goal: frame=%s x=%.3f y=%.3f yaw=%.3f",
              frame_id.c_str(), x, y, yaw);
  return true;
}

BT::NodeStatus NavigateToPoseAction::onResultReceived(const WrappedResult& wr) {
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger(), "[NavigateToPose] goal succeeded");
      return BT::NodeStatus::SUCCESS;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "[NavigateToPose] goal aborted");
      return BT::NodeStatus::FAILURE;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(logger(), "[NavigateToPose] goal canceled");
      return BT::NodeStatus::FAILURE;
    default:
      RCLCPP_ERROR(logger(), "[NavigateToPose] unknown result code");
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus NavigateToPoseAction::onFeedback(
    const std::shared_ptr<const Feedback> /*fb*/) {
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseAction::onFailure(BT::ActionNodeErrorCode error) {
  RCLCPP_ERROR(logger(), "[NavigateToPose] onFailure error code: %d", error);
  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_nav
