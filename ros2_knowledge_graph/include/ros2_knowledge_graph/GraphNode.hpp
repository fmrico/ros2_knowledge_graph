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

#ifndef ROS2_KNOWLEDGE_GRAPH__GRAPHNODE_HPP_
#define ROS2_KNOWLEDGE_GRAPH__GRAPHNODE_HPP_

#include <optional>

#include "ros2_knowledge_graph_msgs/msg/graph.hpp"
#include "ros2_knowledge_graph_msgs/msg/node.hpp"
#include "ros2_knowledge_graph_msgs/msg/edge.hpp"

#include "rclcpp/rclcpp.hpp"

namespace ros2_knowledge_graph
{

class GraphNode
{
public:
  explicit GraphNode(rclcpp::Node::SharedPtr provided_node);

  bool remove_node(const std::string node);
  bool exist_node(const std::string node);
  std::optional<ros2_knowledge_graph_msgs::msg::Node> get_node(const std::string node);

  bool remove_edge(const ros2_knowledge_graph_msgs::msg::Edge & edge);
  std::vector<ros2_knowledge_graph_msgs::msg::Edge> get_edges(
    const std::string & source, const std::string & target, uint8_t type);

  template<class T>
  std::vector<ros2_knowledge_graph_msgs::msg::Edge> get_edges(
    const std::string & source, const std::string & target)
  {
    return {};
  }

  const std::vector<ros2_knowledge_graph_msgs::msg::Node> & get_nodes() {return graph_->nodes;}
  const std::vector<ros2_knowledge_graph_msgs::msg::Edge> & get_edges() {return graph_->edges;}
  const std::vector<std::string> get_node_names();

  size_t get_num_edges() const;
  size_t get_num_nodes() const;

  void updateNode(const ros2_knowledge_graph_msgs::msg::Node & node);
  bool updateEdge(const ros2_knowledge_graph_msgs::msg::Edge & edge);

protected:
  rclcpp::Node::SharedPtr node_;
  ros2_knowledge_graph_msgs::msg::Graph::UniquePtr graph_;

  void publish_graph();
  void publish_tf(const geometry_msgs::msg::TransformStamped & transform);
  void graph_callback(ros2_knowledge_graph_msgs::msg::Graph::UniquePtr msg);
private:
  rclcpp::Publisher<ros2_knowledge_graph_msgs::msg::Graph>::SharedPtr graph_pub_;
  rclcpp::Subscription<ros2_knowledge_graph_msgs::msg::Graph>::SharedPtr graph_sub_;
};

template<>
std::vector<ros2_knowledge_graph_msgs::msg::Edge>
GraphNode::get_edges<bool>(
  const std::string & source, const std::string & target)
{
  return get_edges(source, target, ros2_knowledge_graph_msgs::msg::Content::BOOL);
}

template<>
std::vector<ros2_knowledge_graph_msgs::msg::Edge>
GraphNode::get_edges<int>(
  const std::string & source, const std::string & target)
{
  return get_edges(source, target, ros2_knowledge_graph_msgs::msg::Content::INT);
}

template<>
std::vector<ros2_knowledge_graph_msgs::msg::Edge>
GraphNode::get_edges<float>(
  const std::string & source, const std::string & target)
{
  return get_edges(source, target, ros2_knowledge_graph_msgs::msg::Content::FLOAT);
}

template<>
std::vector<ros2_knowledge_graph_msgs::msg::Edge>
GraphNode::get_edges<double>(
  const std::string & source, const std::string & target)
{
  return get_edges(source, target, ros2_knowledge_graph_msgs::msg::Content::DOUBLE);
}

template<>
std::vector<ros2_knowledge_graph_msgs::msg::Edge>
GraphNode::get_edges<std::string>(
  const std::string & source, const std::string & target)
{
  return get_edges(source, target, ros2_knowledge_graph_msgs::msg::Content::STRING);
}

template<>
std::vector<ros2_knowledge_graph_msgs::msg::Edge>
GraphNode::get_edges<geometry_msgs::msg::PoseStamped>(
  const std::string & source, const std::string & target)
{
  return get_edges(source, target, ros2_knowledge_graph_msgs::msg::Content::POSE);
}

template<>
std::vector<ros2_knowledge_graph_msgs::msg::Edge>
GraphNode::get_edges<geometry_msgs::msg::TransformStamped>(
  const std::string & source, const std::string & target)
{
  auto tf_result = get_edges(source, target, ros2_knowledge_graph_msgs::msg::Content::TF);
  auto tfs_result = get_edges(source, target, ros2_knowledge_graph_msgs::msg::Content::STATICTF);
  
  tf_result.insert(tf_result.begin(), tfs_result.begin(), tfs_result.end());

  return tf_result;
}


}  // namespace ros2_knowledge_graph

#endif  // ROS2_KNOWLEDGE_GRAPH__GRAPHNODE_HPP_
