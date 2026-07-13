#include "bt_nav/set_semantic_instruction.hpp"

namespace bt_nav {

SetSemanticInstruction::SetSemanticInstruction(const std::string& name,
                                               const BT::NodeConfig& conf,
                                               const BT::RosNodeParams& params)
    : BT::RosTopicPubNode<std_msgs::msg::String>(name, conf, params) {}

BT::PortsList SetSemanticInstruction::providedPorts() {
  return providedBasicPorts({
      BT::InputPort<std::string>("instruction",
                                 "Text prompt / instruction for the VLM "
                                 "(e.g. \"curtain\" or \"go through the curtain\")"),
  });
}

bool SetSemanticInstruction::setMessage(std_msgs::msg::String& msg) {
  std::string instruction;
  if (!getInput("instruction", instruction) || instruction.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[SetSemanticInstruction] missing/empty port 'instruction'");
    return false;
  }
  msg.data = instruction;
  RCLCPP_INFO(node_->get_logger(),
              "[SetSemanticInstruction] publishing prompt: '%s'",
              instruction.c_str());
  return true;
}

}  // namespace bt_nav
