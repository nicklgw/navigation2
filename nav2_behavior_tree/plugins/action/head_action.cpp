// Copyright (c) 2018 Samsung Research America
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

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/head_action.hpp"

namespace nav2_behavior_tree
{

HeadAction::HeadAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::DummyBehavior>(xml_tag_name, action_name, conf)
{
  std::string command;
  getInput("head_action", command);
  if (command != "head_up" && command != "head_down") 
  {
    RCLCPP_WARN(
      node_->get_logger(), "Invalid head action. Please use 'head_up' or 'head_down'.");
  }

  goal_.command.data = command;
}

void HeadAction::on_tick()
{
  increment_recovery_count();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::HeadAction>(name, "head", config);
    };

  factory.registerBuilder<nav2_behavior_tree::HeadAction>("Head", builder);
}
