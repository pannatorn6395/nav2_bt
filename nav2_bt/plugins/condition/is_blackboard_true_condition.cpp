#include "nav2_bt/plugins/condition/is_blackboard_true_condition.hpp"

namespace nav2_behavior_tree
{

IsBlackboardTrueCondition::IsBlackboardTrueCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{}

BT::NodeStatus IsBlackboardTrueCondition::tick()
{
    std::string key_blackboard;
    getInput("key_blackboard", key_blackboard);  // Use getInput to retrieve the key_blackboard value
    if (!getInput("key_blackboard", key_blackboard))
    {
        throw BT::RuntimeError("Missing required input [key_blackboard]");
    }

    auto value_str = config().blackboard->template get<std::string>(key_blackboard);
    // Convert the string to a boolean
    bool value;
    if (value_str == "true" || value_str == "1") {
        value = true;
    } else if (value_str == "false" || value_str == "0") {
        value = false;
    } else {
        throw BT::RuntimeError("Invalid value for key_blackboard, expected true/false or 1/0 string");
    }
    std::cout << key_blackboard << (value ? ": true" : ": false") << std::endl;

    return value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::IsBlackboardTrueCondition>("IsBlackboardTrue");
}
