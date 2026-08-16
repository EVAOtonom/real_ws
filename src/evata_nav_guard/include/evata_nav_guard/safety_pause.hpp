#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#ifdef EVATA_BTCPP_V3
#include "behaviortree_cpp_v3/control_node.h"
#else
#include "behaviortree_cpp/control_node.h"
#endif

namespace evata_nav_guard
{

class SafetyPause : public BT::ControlNode
{
public:
  SafetyPause(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void halt() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_sub_;

  std::atomic_bool healthy_{false};
  std::atomic_bool health_received_{false};
  bool paused_{true};
};

}  // namespace evata_nav_guard
