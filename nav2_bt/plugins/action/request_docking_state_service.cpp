#include "nav2_bt/plugins/action/request_docking_state_service.hpp"
#include <iostream>

namespace nav2_behavior_tree // เปลี่ยนเป็น nav2_behavior_tree
{

RequestDockingStateService::RequestDockingStateService(
  const std::string & service_node_name,
  const BT::NodeConfiguration &conf)
  : BtServiceNode<obo_nav_msgs::srv::SetDockAction>(service_node_name, conf)
{
}

void RequestDockingStateService::on_tick()
{
  RCLCPP_INFO(node_->get_logger(), "══════════════════════════════");
  RCLCPP_INFO(node_->get_logger(), "🟢 \"%s\" Request Docking State on Tick 🟢", service_name_.c_str());
  RCLCPP_INFO(node_->get_logger(), "══════════════════════════════");

  std::cout << "\n";

}
BT::NodeStatus RequestDockingStateService::on_completion(
  std::shared_ptr<obo_nav_msgs::srv::SetDockAction::Response> response)
{
  if (response->action == 2 || response->action == 1) {
    RCLCPP_INFO(node_->get_logger(), "══════════════════════════════");
    RCLCPP_INFO(node_->get_logger(), "🟣 Received Docking State Value: %d 🟣", response->action);
    RCLCPP_INFO(node_->get_logger(), "══════════════════════════════");

    if (response->action == 1) {
        setOutput("dock_type", response->action);
        setOutput("dockin", true);
        setOutput("dockout", false);
        RCLCPP_INFO(node_->get_logger(), "dockin is true");

    } else if (response->action == 2) {
        setOutput("dockin", false);
        setOutput("dockout", true);
        RCLCPP_INFO(node_->get_logger(), "dockout is true");

    }

    setOutput("dock_pose", response->dock_pose);
    RCLCPP_INFO(node_->get_logger(), "══════════════════════════════");
    RCLCPP_INFO(node_->get_logger(), "🟣 Received Dock Pose 🟣");
    RCLCPP_INFO(node_->get_logger(), 
        "Position: [x: %f, y: %f, z: %f], Orientation: [x: %f, y: %f, z: %f, w: %f]",
        response->dock_pose.pose.position.x,
        response->dock_pose.pose.position.y,
        response->dock_pose.pose.position.z,
        response->dock_pose.pose.orientation.x,
        response->dock_pose.pose.orientation.y,
        response->dock_pose.pose.orientation.z,
        response->dock_pose.pose.orientation.w);
    RCLCPP_INFO(node_->get_logger(), "══════════════════════════════");

    setOutput("current_pose", response->current_pose);
    RCLCPP_INFO(node_->get_logger(), "══════════════════════════════");
    RCLCPP_INFO(node_->get_logger(), "🟣 Received Current Pose 🟣");
    RCLCPP_INFO(node_->get_logger(), 
        "Position: [x: %f, y: %f, z: %f], Orientation: [x: %f, y: %f, z: %f, w: %f]",
        response->current_pose.pose.position.x,
        response->current_pose.pose.position.y,
        response->current_pose.pose.position.z,
        response->current_pose.pose.orientation.x,
        response->current_pose.pose.orientation.y,
        response->current_pose.pose.orientation.z,
        response->current_pose.pose.orientation.w);
    RCLCPP_INFO(node_->get_logger(), "══════════════════════════════");

    RCLCPP_INFO(node_->get_logger(), "🔵 DONE FOR SET TO BLACKBOARD 🔵");
    std::cout << "\n";

    return BT::NodeStatus::SUCCESS;
}

  else {
    RCLCPP_INFO(node_->get_logger(), "══════════════════════════════");
    RCLCPP_INFO(node_->get_logger(), "🔴 FAILURE: Docking State Not Received! 🔴");
    RCLCPP_INFO(node_->get_logger(), "══════════════════════════════");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace nav2_behavior_tree // เปลี่ยนเป็น nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RequestDockingStateService>("RequestDockingState");
}
