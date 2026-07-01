// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>

#include <tf2/LinearMath/Quaternion.h>

#include "nav2_behavior_tree/plugins/action/compute_path_to_pose_action.hpp"

namespace nav2_behavior_tree
{

ComputePathToPoseAction::ComputePathToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputePathToPose>(xml_tag_name, action_name, conf)
{
}

void ComputePathToPoseAction::on_tick()
{
  getInput("goal", goal_.goal);
  getInput("planner_id", goal_.planner_id);
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }

  double goal_x = 0.0, goal_y = 0.0, goal_yaw = 0.0;
  if (getInput("goal_x", goal_x) && getInput("goal_y", goal_y) && getInput("goal_yaw", goal_yaw)) 
  {
    goal_.goal.header.stamp = node_->now();
    goal_.goal.header.frame_id = "map";
    goal_.goal.pose.position.x = goal_x;
    goal_.goal.pose.position.y = goal_y;
    goal_.goal.pose.position.z = 0.0;

    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, goal_yaw * M_PI / 180.0);
    goal_.goal.pose.orientation.x = q.x();
    goal_.goal.pose.orientation.y = q.y();
    goal_.goal.pose.orientation.z = q.z();
    goal_.goal.pose.orientation.w = q.w();
  }
}
BT::NodeStatus ComputePathToPoseAction::on_success()
{
  setOutput("path", result_.result->path);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathToPoseAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathToPoseAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
}

void ComputePathToPoseAction::halt()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  BtActionNode::halt();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputePathToPoseAction>(
        name, "compute_path_to_pose", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputePathToPoseAction>(
    "ComputePathToPose", builder);
}
