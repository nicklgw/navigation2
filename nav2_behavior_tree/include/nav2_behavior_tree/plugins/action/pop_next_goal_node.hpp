#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__POP_NEXT_GOAL_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__POP_NEXT_GOAL_NODE_HPP_

#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

  class PopNextGoal : public BT::SyncActionNode
  {
  public:
    /**
     * @brief A constructor for nav2_behavior_tree::PopNextGoal
     *
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf  BT node configuration
     */
    PopNextGoal(
        const std::string &xml_tag_name,
        const BT::NodeConfiguration &conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
              "goals",
              "The goals list to pop"),

          BT::OutputPort<geometry_msgs::msg::PoseStamped>(
              "next_goal",
              "The goal selected"),

          BT::OutputPort<int>(
              "remain_size",
              "The goals list size after pop"),

          BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
              "remain_goals",
              "The goals list after pop"),

          BT::OutputPort<std::string>(
              "goal_checker_id",
              "The path goal checker when to follow")};
    }

  private:
    /**
     * @brief Function to perform some user-defined operation on tick
     */
    BT::NodeStatus tick() override;

    rclcpp::Node::SharedPtr node_;
  };

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__POP_NEXT_GOAL_NODE_HPP_
