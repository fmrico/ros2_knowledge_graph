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

#include <vector>
#include <list>
#include <random>
#include <string>
#include <memory>

#include "ros2_knowledge_graph_msgs/srv/graph_update_request.hpp"
#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ros2_knowledge_graph/Graph.hpp"
#include "ros2_knowledge_graph/GraphMaster.hpp"

namespace ros2_knowledge_graph
{

using std::placeholders::_1;
using std::placeholders::_2;

GraphMaster::GraphMaster()
: Node("graph_master")
{
  service_ = create_service<ros2_knowledge_graph_msgs::srv::GraphUpdateRequest>(
    "/graph_update_request", std::bind(&GraphMaster::update_req_cb, this, _1, _2));
  update_pub_ = create_publisher<ros2_knowledge_graph_msgs::msg::GraphUpdate>(
    "/graph_updates", rclcpp::QoS(1000).reliable());
}

void 
GraphMaster::update_req_cb(const std::shared_ptr<ros2_knowledge_graph_msgs::srv::GraphUpdateRequest::Request> request,
  std::shared_ptr<ros2_knowledge_graph_msgs::srv::GraphUpdateRequest::Response>      response)
{
  // std::cerr << "011s" << std::endl;

  std::lock_guard<std::mutex> lock(mutex_);
  const auto & author_id = request->node_id;
  const auto & element = request->element_type;
  const auto & operation = request->operation_type;
  const auto & object_data = request->object;
  const auto & ts = request->stamp;
  switch (element) {
    case ros2_knowledge_graph_msgs::srv::GraphUpdateRequest::Request::NODE:
      {
        switch (operation) {
          case ros2_knowledge_graph_msgs::srv::GraphUpdateRequest::Request::ADD:
            {
              ros2_knowledge_graph::Node node;
              node.from_string(object_data);
              response->success = graph_.add_node(node);
              RCLCPP_DEBUG(
                get_logger(), "[%lf]\t(%s)\tADD NODE %s",
                rclcpp::Time(ts).seconds(),
                request->node_id.c_str(), object_data.c_str());
              break;
            }
          case ros2_knowledge_graph_msgs::srv::GraphUpdateRequest::Request::REMOVE:
            {
              ros2_knowledge_graph::Node node;
              node.from_string(object_data);
              response->success = graph_.remove_node(node.name);
              RCLCPP_DEBUG(
                get_logger(), "[%lf]\t(%s)\tREMOVE NODE %s",
                rclcpp::Time(ts).seconds(),
                request->node_id.c_str(), object_data.c_str());
              break;
            }
        }
      }
      break;
    case ros2_knowledge_graph_msgs::srv::GraphUpdateRequest::Request::EDGE:
      {
        ros2_knowledge_graph::Edge edge;
        edge.from_string(object_data);
        switch (operation) {
          case ros2_knowledge_graph_msgs::srv::GraphUpdateRequest::Request::ADD:
            response->success = graph_.add_edge(edge);
            RCLCPP_DEBUG(
              get_logger(), "[%lf]\t(%s)\tADD EDGE %s",
              rclcpp::Time(ts).seconds(),
              request->node_id.c_str(), object_data.c_str());
            break;
          case ros2_knowledge_graph_msgs::srv::GraphUpdateRequest::Request::REMOVE:
            response->success = graph_.remove_edge(edge);
            RCLCPP_DEBUG(
              get_logger(), "[%lf]\t(%s)\tREMOVE NODE %s",
              rclcpp::Time(ts).seconds(),
              request->node_id.c_str(), object_data.c_str());
            break;
        }
      }
      break;
    case ros2_knowledge_graph_msgs::srv::GraphUpdateRequest::Request::GRAPH:
      {
        switch (operation) {
          case ros2_knowledge_graph_msgs::srv::GraphUpdateRequest::Request::SYNC:
            response->success = false;
            RCLCPP_DEBUG(
              get_logger(), "[%lf]\t(%s)\tSYNC %s",
              rclcpp::Time(ts).seconds(),
              request->node_id.c_str(), object_data.c_str());
            break;
          case ros2_knowledge_graph_msgs::srv::GraphUpdateRequest::Request::REQSYNC:
            response->success;
            RCLCPP_DEBUG(
              get_logger(), "[%lf]\t(%s)\tREQSYNC %s",
              rclcpp::Time(ts).seconds(),
              request->node_id.c_str(), object_data.c_str());
            break;
        }
      }
      break;
    }
  if (response->success) {
    ros2_knowledge_graph_msgs::msg::GraphUpdate out_msg;
    out_msg.object = graph_.to_string();
    update_pub_->publish(out_msg);
  }

  // std::cerr << "019s" << std::endl;

}

}  // namespace ros2_knowledge_graph
