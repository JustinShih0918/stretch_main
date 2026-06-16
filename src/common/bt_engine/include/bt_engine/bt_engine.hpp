#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

class BTEngine : public rclcpp::Node {
public:
  BTEngine();

  void init();
  void registerNodes();
  void buildTree();
  void runTree();

  bool isStartReceived() const { return start_received_.load(); }
  bool isTreeReady() const { return tree_ready_; }

private:
  void startCallback(const std_msgs::msg::Empty::SharedPtr msg);

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::RosNodeParams params_;
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;

  std::string bt_xml_path_;
  std::string tree_name_;
  int tick_rate_hz_;

  std::atomic<bool> start_received_{false};
  bool tree_ready_{false};
};
