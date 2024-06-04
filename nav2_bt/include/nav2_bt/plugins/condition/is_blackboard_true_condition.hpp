#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BLACKBOARD_TRUE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BLACKBOARD_TRUE_CONDITION_HPP_

#include <string>
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

class IsBlackboardTrueCondition : public BT::ConditionNode
{
public:
  IsBlackboardTrueCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsBlackboardTrueCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "key_blackboard", "The blackboard key to check")
    };
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BLACKBOARD_TRUE_CONDITION_HPP_
