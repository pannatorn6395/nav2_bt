#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REQUEST_DOCKING_STATE_SERVICE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REQUEST_DOCKING_STATE_SERVICE_HPP_

#include <string>
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "obo_nav_msgs/srv/set_dock_action.hpp"

namespace nav2_behavior_tree
{

class RequestDockingStateService : public nav2_behavior_tree::BtServiceNode<obo_nav_msgs::srv::SetDockAction>
{
public:
  RequestDockingStateService(
    const std::string & service_node_name,
    const BT::NodeConfiguration &conf);

  void on_tick() override;
  BT::NodeStatus on_completion(std::shared_ptr<obo_nav_msgs::srv::SetDockAction::Response> response) override;

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
      {
        //BT::InputPort<std::string>("topic", "topic of service"),
        //BT::OutputPort<obo_nav_msgs::srv::SetDockAction_Response>("outputkey", "output for set blackboard")
           BT::OutputPort<int8_t>("dock_type", "output for set blackboard")
          ,BT::OutputPort<bool>("dockin", "output for set blackboard")
          ,BT::OutputPort<bool>("dockout", "output for set blackboard")
          ,BT::OutputPort<geometry_msgs::msg::PoseStamped>("dock_pose", "dock pose for docking")
          ,BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_pose", "current pose for docking")



      }
    );
  }
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REQUEST_DOCKING_STATE_SERVICE_HPP_
