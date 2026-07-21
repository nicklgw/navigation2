#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/pop_next_goal_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

  PopNextGoal::PopNextGoal(
      const std::string &name,
      const BT::NodeConfiguration &conf)
      : BT::SyncActionNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  BT::NodeStatus PopNextGoal::tick()
  {
    geometry_msgs::msg::PoseStamped goal;
    std::vector<geometry_msgs::msg::PoseStamped> goals;
    getInput("goals", goals);
    if (goals.empty())
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "PopNextGoal goal list size: %ld ", goals.size());
      return BT::NodeStatus::FAILURE;
    }
    goal = goals.front();
    goals.erase(goals.begin());
    setOutput("next_goal", goal);
    setOutput("remain_goals", goals);

    std::string goal_chacker = "line_pid_goal_checker";

    setOutput("goal_checker_id", goal_chacker);
    setOutput("remain_size", (int)goals.size());
    config().blackboard->set<bool>("goal_arrived", false);
    RCLCPP_INFO(node_->get_logger(), "PopNextGoal set goal_arrived false");

    return BT::NodeStatus::SUCCESS;
  }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PopNextGoal>("PopNextGoal");
}
