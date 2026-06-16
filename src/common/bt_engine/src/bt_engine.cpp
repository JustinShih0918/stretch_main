#include "bt_engine/bt_engine.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "bt_nav/navigate_to_pose_action.hpp"

BTEngine::BTEngine() : rclcpp::Node("bt_engine") {
  const std::string default_xml =
      ament_index_cpp::get_package_share_directory("bt_engine") +
      "/bt/main_tree.xml";

  this->declare_parameter<std::string>("bt_xml_path", default_xml);
  this->declare_parameter<std::string>("tree_name", "MainTree");
  this->declare_parameter<int>("tick_rate_hz", 20);

  this->get_parameter("bt_xml_path", bt_xml_path_);
  this->get_parameter("tree_name", tree_name_);
  this->get_parameter("tick_rate_hz", tick_rate_hz_);

  start_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/start", 1,
      std::bind(&BTEngine::startCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
              "[BTEngine] xml=%s tree=%s tick_hz=%d",
              bt_xml_path_.c_str(), tree_name_.c_str(), tick_rate_hz_);
}

void BTEngine::init() {
  node_ = this->shared_from_this();
  params_.nh = node_;
}

void BTEngine::registerNodes() {
  factory_.registerNodeType<bt_nav::NavigateToPoseAction>("NavigateToPose",
                                                          params_);
  RCLCPP_INFO(this->get_logger(), "[BTEngine] registered NavigateToPose");
}

void BTEngine::buildTree() {
  try {
    factory_.registerBehaviorTreeFromFile(bt_xml_path_);
    tree_ = factory_.createTree(tree_name_);
    tree_ready_ = true;
    RCLCPP_INFO(this->get_logger(),
                "[BTEngine] tree built; waiting for /start ...");
  } catch (const std::exception& e) {
    RCLCPP_FATAL(this->get_logger(),
                 "[BTEngine] failed to build tree: %s", e.what());
    tree_ready_ = false;
  }
}

void BTEngine::startCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
  if (start_received_.exchange(true)) {
    RCLCPP_INFO(this->get_logger(), "[BTEngine] /start ignored (already running)");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "[BTEngine] /start received");
}

void BTEngine::runTree() {
  RCLCPP_INFO(this->get_logger(), "[BTEngine] tick loop start");
  rclcpp::Rate rate(tick_rate_hz_);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node_);
    status = tree_.tickOnce();
    rate.sleep();
  }

  RCLCPP_INFO(this->get_logger(), "[BTEngine] tree finished: %s",
              BT::toStr(status).c_str());
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto engine = std::make_shared<BTEngine>();
  engine->init();
  engine->registerNodes();
  engine->buildTree();

  if (!engine->isTreeReady()) {
    RCLCPP_FATAL(engine->get_logger(), "[BTEngine] aborting: tree not ready");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::Rate idle_rate(50);
  while (rclcpp::ok() && !engine->isStartReceived()) {
    rclcpp::spin_some(engine);
    idle_rate.sleep();
  }

  if (rclcpp::ok()) {
    engine->runTree();
  }

  rclcpp::shutdown();
  return 0;
}
