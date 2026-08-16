#include "evata_nav_guard/safety_pause.hpp"

#include <stdexcept>
#include <utility>

#ifdef EVATA_BTCPP_V3
#include "behaviortree_cpp_v3/bt_factory.h"
#else
#include "behaviortree_cpp/bt_factory.h"
#endif

namespace evata_nav_guard
{

SafetyPause::SafetyPause(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
  if (!config.blackboard) {
    throw std::runtime_error("EvataSafetyPause: BehaviorTree blackboard is null");
  }
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string health_topic = "/evata/system_healthy";
  (void)getInput("health_topic", health_topic);

  // Supervisor uses the same transient-local QoS, so a late-starting BT receives
  // the latest health state immediately. Initial state is deliberately unsafe.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  health_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    health_topic,
    qos,
    [this](std_msgs::msg::Bool::SharedPtr msg) {
      healthy_.store(msg->data);
      health_received_.store(true);
    });
}

BT::PortsList SafetyPause::providedPorts()
{
  return {
    BT::InputPort<std::string>("health_topic")
  };
}

BT::NodeStatus SafetyPause::tick()
{
  if (children_nodes_.size() != 1) {
    throw std::runtime_error("EvataSafetyPause must have exactly one child");
  }

  const bool ok = health_received_.load() && healthy_.load();

  if (!ok) {
    if (!paused_) {
      // Halt active Nav2 action children (e.g. FollowPath) exactly when entering
      // pause. The outer NavigateToPose BT remains RUNNING, so the user's goal
      // is not returned as FAILURE/ABORT by this guard.
      haltChildren();
      paused_ = true;
      RCLCPP_WARN(node_->get_logger(), "EVATA navigation paused: system unhealthy");
    }
    return BT::NodeStatus::RUNNING;
  }

  if (paused_) {
    paused_ = false;
    RCLCPP_INFO(node_->get_logger(), "EVATA navigation resumed: system healthy");
  }

  const auto status = children_nodes_.front()->executeTick();

  if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
    haltChildren();
  }

  return status;
}

void SafetyPause::halt()
{
  haltChildren();
  paused_ = true;
  BT::ControlNode::halt();
}

}  // namespace evata_nav_guard

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<evata_nav_guard::SafetyPause>("EvataSafetyPause");
}
