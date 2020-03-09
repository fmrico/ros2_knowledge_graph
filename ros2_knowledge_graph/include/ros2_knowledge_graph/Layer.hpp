// Copyright 2019 Intelligent Robotics Lab
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

#ifndef ROS2_KNOWLEDGE_GRAPH__LAYER_HPP_
#define ROS2_KNOWLEDGE_GRAPH__LAYER_HPP_

#include <memory>
#include <string>

#include "ros2_knowledge_graph/Types.hpp"

namespace ros2_knowledge_graph
{

class Layer
{
public:
  using Ptr = std::shared_ptr<ros2_knowledge_graph::Layer>;

  Layer() {}

  virtual void configure(rclcpp::Node::SharedPtr & node) {node_ = node;}

  virtual void on_add_node(Node & node) {}
  virtual void on_remove_node(const std::string node) {}
  virtual void on_get_node(Node & node) {}

  virtual void on_add_edge(Edge & edge) {}
  virtual void on_remove_edge(Edge & edge) {}
  virtual void on_get_edge(Edge & edge) {}

protected:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace ros2_knowledge_graph

#endif  // ROS2_KNOWLEDGE_GRAPH__LAYER_HPP_
