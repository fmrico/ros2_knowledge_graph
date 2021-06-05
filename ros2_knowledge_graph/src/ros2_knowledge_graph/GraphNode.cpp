// Copyright 2021 Intelligent Robotics Lab
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


#include "ros2_knowledge_graph_msgs/msg/graph.hpp"
#include "ros2_knowledge_graph_msgs/msg/node.hpp"
#include "ros2_knowledge_graph_msgs/msg/edge.hpp"

#include "ros2_knowledge_graph/GraphNode.hpp"

namespace ros2_knowledge_graph
{

using std::placeholders::_1;

GraphNode::GraphNode(rclcpp::Node::SharedPtr provided_node)
: node_(provided_node)
{
  graph_ = std::make_unique<ros2_knowledge_graph_msgs::msg::Graph>();
  graph_->seq = 0;

  graph_pub_ = node_->create_publisher<ros2_knowledge_graph_msgs::msg::Graph>(
    "graph", rclcpp::QoS(100).transient_local().keep_last(100).reliable());
  graph_sub_ = node_->create_subscription<ros2_knowledge_graph_msgs::msg::Graph>(
    "graph", rclcpp::QoS(100).transient_local().keep_last(100).reliable(),
    std::bind(&GraphNode::graph_callback, this,_1));
}

bool
GraphNode::remove_node(const std::string node)
{
  bool removed = false;
  auto it = graph_->nodes.begin();
  while (!removed && it != graph_->nodes.end()) {
    if (it->node_name == node) {
      it = graph_->nodes.erase(it);
      removed = true;
    } else {
      ++it;
    }
  }

  if (removed) {
    auto ite = graph_->edges.begin();
    while (ite != graph_->edges.end()) {
      if (ite->source_node_id == node || ite->source_node_id == node) {
        ite = graph_->edges.erase(ite);
      } else {
        ++ite;
      }
    }
  }

  if (removed) {
    publish_graph();
  }

  return removed;
}

bool
GraphNode::exist_node(const std::string node)
{
  return get_node(node).has_value();
}

std::optional<ros2_knowledge_graph_msgs::msg::Node>
GraphNode::get_node(const std::string node)
{
  auto it = graph_->nodes.begin();
  while (it != graph_->nodes.end()) {
    if (it->node_name == node) {
      return *it;
    } else {
      ++it;
    }
  }

  return {};
}

const std::vector<std::string>
GraphNode::get_node_names()
{
  std::vector<std::string> ret;
  for (const auto & node : graph_->nodes) {
    ret.push_back(node.node_name);
  }
  return ret;
}

bool
GraphNode::remove_edge(const ros2_knowledge_graph_msgs::msg::Edge & edge)
{
  bool removed = false;

  auto it = graph_->edges.begin();
  while (it != graph_->edges.end()) {
    if (it->source_node_id == edge.source_node_id &&
      it->target_node_id == edge.target_node_id &&
      it->content.type == edge.content.type)
    {
      if (it->content.type == ros2_knowledge_graph_msgs::msg::Content::STRING) {
        if (it->content.string_value == edge.content.string_value) {
          it = graph_->edges.erase(it);
          removed = true;
        } else {
          ++it;
        }
      } else {
        it = graph_->edges.erase(it);
        removed = true;
      }
    } else {
      ++it;
    }
  }

  if (removed) {
    publish_graph();
  }

  return removed;
}

std::vector<ros2_knowledge_graph_msgs::msg::Edge>
GraphNode::get_edges(
  const std::string & source, const std::string & target, uint8_t type)
{
  std::vector<ros2_knowledge_graph_msgs::msg::Edge> ret;

  for (const auto & edge : graph_->edges) {
    if (edge.source_node_id == source &&
      edge.target_node_id == target &&
      edge.content.type == type) {
      ret.push_back(edge);
    }
  }

  return ret;
}

size_t
GraphNode::get_num_edges() const
{
  return graph_->edges.size();
}

size_t
GraphNode::get_num_nodes() const
{
  return graph_->nodes.size();
}

void
GraphNode::updateNode(const ros2_knowledge_graph_msgs::msg::Node & node)
{
  bool found = false;
  auto it = graph_->nodes.begin();
  while (!found && it != graph_->nodes.end()) {
    if (it->node_name == node.node_name) {
      *it = node;
      found = true;
    } 
    ++it;
  }

  if (!found) {
    graph_->nodes.push_back(node);
  }

  publish_graph();
}

bool
GraphNode::updateEdge(const ros2_knowledge_graph_msgs::msg::Edge & edge)
{
  if (!exist_node(edge.source_node_id)) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "Node source [" << edge.source_node_id << "] doesn't exist adding edge");
    return false;
  }

  if (!exist_node(edge.target_node_id)) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "Node target [" << edge.target_node_id << "] doesn't exist adding edge");
    return false;
  }

  bool found = false;
  auto it = graph_->edges.begin();
  while (!found && it != graph_->edges.end()) {
    if (it->source_node_id == edge.source_node_id &&
      it->target_node_id == edge.target_node_id &&
      it->content.type == edge.content.type &&
      it->content.type != ros2_knowledge_graph_msgs::msg::Content::STRING)
    {
      *it = edge;
      found = true;
    } 
    ++it;
  }

  ros2_knowledge_graph_msgs::msg::Edge mod_edge = edge;
  if (!found) {
    if (mod_edge.content.type == ros2_knowledge_graph_msgs::msg::Content::STATICTF || 
      mod_edge.content.type == ros2_knowledge_graph_msgs::msg::Content::TF)
    {
      mod_edge.content.tf_value.header.frame_id = edge.source_node_id;
      mod_edge.content.tf_value.child_frame_id = edge.target_node_id;
      publish_tf(mod_edge.content.tf_value);
    }
    graph_->edges.push_back(mod_edge);

    publish_graph();
  } else {
    if (mod_edge.content.type != ros2_knowledge_graph_msgs::msg::Content::STATICTF && 
      mod_edge.content.type != ros2_knowledge_graph_msgs::msg::Content::TF)
    {
      publish_graph();
    } else {
      mod_edge.content.tf_value.header.frame_id = edge.source_node_id;
      mod_edge.content.tf_value.child_frame_id = edge.target_node_id;
      publish_tf(mod_edge.content.tf_value);
    }
  }
  
  return true;
}

void
GraphNode::publish_graph()
{
  graph_->seq++;
  graph_pub_->publish(*graph_);
}

void
GraphNode::graph_callback(ros2_knowledge_graph_msgs::msg::Graph::UniquePtr msg)
{
  if (msg->seq > graph_->seq) {
    graph_ = std::move(msg);
  }
}

void
GraphNode::publish_tf(const geometry_msgs::msg::TransformStamped & transform)
{

}
}  // namespace ros2_knowledge_graph
