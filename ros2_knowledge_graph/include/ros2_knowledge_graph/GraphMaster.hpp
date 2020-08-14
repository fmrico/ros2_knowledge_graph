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

#ifndef ROS2_KNOWLEDGE_GRAPH__GRAPHMASTER_HPP_
#define ROS2_KNOWLEDGE_GRAPH__GRAPHMASTER_HPP_

#include <vector>
#include <list>
#include <random>
#include <string>
#include <memory>

#include "ros2_knowledge_graph_msgs/srv/graph_update_request.hpp"
#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ros2_knowledge_graph/Graph.hpp"

namespace ros2_knowledge_graph
{

class GraphMaster : public rclcpp::Node
{
public:
  GraphMaster();

  void update_req_cb(const std::shared_ptr<ros2_knowledge_graph_msgs::srv::GraphUpdateRequest::Request> request,
    std::shared_ptr<ros2_knowledge_graph_msgs::srv::GraphUpdateRequest::Response> response);


private:
  ros2_knowledge_graph::Graph graph_;

  rclcpp::Service<ros2_knowledge_graph_msgs::srv::GraphUpdateRequest>::SharedPtr service_;
  rclcpp::Publisher<ros2_knowledge_graph_msgs::msg::GraphUpdate>::SharedPtr update_pub_;

  mutable std::mutex mutex_;
};

}  // namespace ros2_knowledge_graph

#endif  // ROS2_KNOWLEDGE_GRAPH__GRAPHMASTER_HPP_
