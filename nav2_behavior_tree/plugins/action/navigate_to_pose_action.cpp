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
#include <sstream>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

#include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"

namespace nav2_behavior_tree
{

NavigateToPoseAction::NavigateToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void NavigateToPoseAction::on_tick()
{
  if (!getInput("goal", goal_.pose)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "NavigateToPoseAction: goal not provided");
    return;
  }

  std::string goal_str;
  if (getInput("goal_str", goal_str)) // "x,y,yaw" format
  {
    if (!goal_str.empty()) 
    {
      auto goal_from_string = [this](const std::string & str, geometry_msgs::msg::PoseStamped & pose)->bool 
      {
        // Expected format: "x,y,yaw" where yaw is in degrees
        try {
          std::stringstream ss(str);
          std::string item;
          std::vector<std::string> parts;
          while (std::getline(ss, item, ',')) {
            if (!item.empty()) {
              parts.push_back(item);
            }
          }

          if (parts.size() != 3) {
            return false;
          }

          double x = std::stod(parts[0]);
          double y = std::stod(parts[1]);
          double yaw_deg = std::stod(parts[2]);
          double yaw = yaw_deg * M_PI / 180.0; // convert to radians

          pose.header.stamp = node_->now();
          pose.header.frame_id = "map";
          pose.pose.position.x = x;
          pose.pose.position.y = y;
          pose.pose.position.z = 0.0;

          // Convert yaw to quaternion
          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, yaw);
          pose.pose.orientation.x = q.x();
          pose.pose.orientation.y = q.y();
          pose.pose.orientation.z = q.z();
          pose.pose.orientation.w = q.w();

          return true;
        } catch (const std::exception & e) {
          (void)e;
          return false;
        }
      };

      if (!goal_from_string(goal_str, goal_.pose)) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "NavigateToPoseAction: failed to convert goal_str to PoseStamped");
        return;
      }
    }
  } 

  getInput("behavior_tree", goal_.behavior_tree);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::NavigateToPoseAction>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<nav2_behavior_tree::NavigateToPoseAction>(
    "NavigateToPose", builder);
}
