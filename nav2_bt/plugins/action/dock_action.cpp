#include <memory>
#include <string>
#include "nav2_bt/plugins/action/dock_action.hpp"

namespace nav2_behavior_tree 
{

DockAction::DockAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<obo_nav_msgs::action::ChargePlugin>(xml_tag_name, action_name, conf)
{
}

void DockAction::on_tick()
{
  getInput("dock_pose", goal_.dock_pose);
  getInput("current_pose", goal_.current_pose);

  RCLCPP_INFO(node_->get_logger(), "══════════════════════════════");
  RCLCPP_INFO(
      node_->get_logger(), 
      "🟢 \"%s\" Dock Action on Tick 🟢",
      action_name_.c_str());
  RCLCPP_INFO(node_->get_logger(), "══════════════════════════════");
}

BT::NodeStatus DockAction::on_success()
{
  setOutput("current_pose", result_.result->last_pose);
  
  RCLCPP_INFO(node_->get_logger(), "🔵 Dock Action Succeeded 🔵");

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DockAction::on_aborted()
{
  RCLCPP_INFO(node_->get_logger(), "🔴 Dock Action Aborted 🔴");
  if (result_.result->reason == 1) {
        setOutput("soft_stop", true);
        setOutput("hard_stop", false);
        setOutput("cam_detection_failed", false);
        setOutput("lidar_detection_failed", false);
        setOutput("flashlight_failed", false);
        setOutput("move_base_failed", false);
        setOutput("localization_failed", false);
        setOutput("charging_failed", false);
        setOutput("timeout_failed", false);
        setOutput("unknow_failed", false);
    } else if (result_.result->reason == 2) {
        setOutput("soft_stop", false);
        setOutput("hard_stop", true);
        setOutput("cam_detection_failed", false);
        setOutput("lidar_detection_failed", false);
        setOutput("flashlight_failed", false);
        setOutput("move_base_failed", false);
        setOutput("localization_failed", false);
        setOutput("charging_failed", false);
        setOutput("timeout_failed", false);
        setOutput("unknow_failed", false);
    } else if (result_.result->reason == 3) {
        setOutput("soft_stop", false);
        setOutput("hard_stop", false);
        setOutput("cam_detection_failed", true);
        setOutput("lidar_detection_failed", false);
        setOutput("flashlight_failed", false);
        setOutput("move_base_failed", false);
        setOutput("localization_failed", false);
        setOutput("charging_failed", false);
        setOutput("timeout_failed", false);
        setOutput("unknow_failed", false);
    } else if (result_.result->reason == 4) {
        setOutput("soft_stop", false);
        setOutput("hard_stop", false);
        setOutput("cam_detection_failed", false);
        setOutput("lidar_detection_failed", true);
        setOutput("flashlight_failed", false);
        setOutput("move_base_failed", false);
        setOutput("localization_failed", false);
        setOutput("charging_failed", false);
        setOutput("timeout_failed", false);
        setOutput("unknow_failed", false);
    } else if (result_.result->reason == 5) {
        setOutput("soft_stop", false);
        setOutput("hard_stop", false);
        setOutput("cam_detection_failed", false);
        setOutput("lidar_detection_failed", false);
        setOutput("flashlight_failed", true);
        setOutput("move_base_failed", false);
        setOutput("localization_failed", false);
        setOutput("charging_failed", false);
        setOutput("timeout_failed", false);
        setOutput("unknow_failed", false);
    } else if (result_.result->reason == 6) {
        setOutput("soft_stop", false);
        setOutput("hard_stop", false);
        setOutput("cam_detection_failed", false);
        setOutput("lidar_detection_failed", false);
        setOutput("flashlight_failed", false);
        setOutput("move_base_failed", true);
        setOutput("localization_failed", false);
        setOutput("charging_failed", false);
        setOutput("timeout_failed", false);
        setOutput("unknow_failed", false);
    } else if (result_.result->reason == 7) {
        setOutput("soft_stop", false);
        setOutput("hard_stop", false);
        setOutput("cam_detection_failed", false);
        setOutput("lidar_detection_failed", false);
        setOutput("flashlight_failed", false);
        setOutput("move_base_failed", false);
        setOutput("localization_failed", true);
        setOutput("charging_failed", false);
        setOutput("timeout_failed", false);
        setOutput("unknow_failed", false);
    } else if (result_.result->reason == 8) {
        setOutput("soft_stop", false);
        setOutput("hard_stop", false);
        setOutput("cam_detection_failed", false);
        setOutput("lidar_detection_failed", false);
        setOutput("flashlight_failed", false);
        setOutput("move_base_failed", false);
        setOutput("localization_failed", false);
        setOutput("charging_failed", true);
        setOutput("timeout_failed", false);
        setOutput("unknow_failed", false);
    } else if (result_.result->reason == 9) {
        setOutput("soft_stop", false);
        setOutput("hard_stop", false);
        setOutput("cam_detection_failed", false);
        setOutput("lidar_detection_failed", false);
        setOutput("flashlight_failed", false);
        setOutput("move_base_failed", false);
        setOutput("localization_failed", false);
        setOutput("charging_failed", false);
        setOutput("timeout_failed", true);
        setOutput("unknow_failed", false);
    } else if (result_.result->reason == 10) {
        setOutput("soft_stop", false);
        setOutput("hard_stop", false);
        setOutput("cam_detection_failed", false);
        setOutput("lidar_detection_failed", false);
        setOutput("flashlight_failed", false);
        setOutput("move_base_failed", false);
        setOutput("localization_failed", false);
        setOutput("charging_failed", false);
        setOutput("timeout_failed", false);
        setOutput("unknow_failed", true);
    }

  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DockAction::on_cancelled()
{
  RCLCPP_INFO(node_->get_logger(), "🟡 Dock Action Cancelled 🟡");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::DockAction>(
        name, "dock_action", config);
    };

  factory.registerBuilder<nav2_behavior_tree::DockAction>(
    "DOCK_PLUGIN", builder);
}
