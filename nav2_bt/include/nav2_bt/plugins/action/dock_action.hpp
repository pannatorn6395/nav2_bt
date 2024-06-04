
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DOCK_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DOCK_ACTION_HPP_
#include <string>


#include "nav2_behavior_tree/bt_action_node.hpp"
#include "obo_nav_msgs/action/charge_plugin.hpp"

namespace nav2_behavior_tree
{

class DockAction : public nav2_behavior_tree::BtActionNode<obo_nav_msgs::action::ChargePlugin>
{
public:
  DockAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  BT::NodeStatus on_success() override;

  BT::NodeStatus on_aborted() override;

  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<bool>("soft_stop", "output for set blackboard"),
        BT::OutputPort<bool>("hard_stop", "output for set blackboard"),
        BT::OutputPort<bool>("cam_detection_failed", "output for set blackboard"),
        BT::OutputPort<bool>("lidar_detection_failed", "output for set blackboard"),
        BT::OutputPort<bool>("flashlight_failed", "output for set blackboard"),
        BT::OutputPort<bool>("move_base_failed", "output for set blackboard"),
        BT::OutputPort<bool>("localization_failed", "output for set blackboard"),
        BT::OutputPort<bool>("charging_failed", "output for set blackboard"),
        BT::OutputPort<bool>("timeout_failed", "output for set blackboard"),
        BT::OutputPort<bool>("unknow_failed", "output for set blackboard"),

        BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_pose", "current pose for docking"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("dock_pose", "dock pose for docking"),

      });
  }

private:
  bool is_recovery_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__DOCK_ACTION_HPP_
