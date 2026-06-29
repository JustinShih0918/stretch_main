#pragma once

#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <std_msgs/msg/string.hpp>

namespace bt_nav {

// Publishes a free-form instruction / text prompt (std_msgs/String) to the
// perception node's instruction topic, switching the active landmark the VLM
// looks for (paper 2310.08873 interactive front end). This is where a future
// LLM instruction-parser plugs in. Returns SUCCESS once published.
//
// Usage in XML (topic_name is required by RosTopicPubNode):
//   <SetSemanticInstruction topic_name="/semantic_instruction"
//                           instruction="curtain"/>
class SetSemanticInstruction
    : public BT::RosTopicPubNode<std_msgs::msg::String> {
public:
  SetSemanticInstruction(const std::string& name, const BT::NodeConfig& conf,
                         const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  bool setMessage(std_msgs::msg::String& msg) override;
};

}  // namespace bt_nav
