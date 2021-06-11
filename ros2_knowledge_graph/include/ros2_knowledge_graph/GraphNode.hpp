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

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <optional>
#include <string>
#include <vector>
#include <memory>

#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"
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

  bool remove_node(const std::string node, bool sync = true);
  bool exist_node(const std::string node);
  std::optional<ros2_knowledge_graph_msgs::msg::Node> get_node(const std::string node);

  bool remove_edge(const ros2_knowledge_graph_msgs::msg::Edge & edge, bool sync = true);
  std::vector<ros2_knowledge_graph_msgs::msg::Edge> get_edges(
    const std::string & source, const std::string & target, uint8_t type);

  template<class T>
  std::vector<ros2_knowledge_graph_msgs::msg::Edge> get_edges(
    const std::string & source, const std::string & target)
  {
    return {};
  }

  std::vector<ros2_knowledge_graph_msgs::msg::Edge> get_edges_from_node_by_data(
    const std::string & source, const std::string & expr);

  const std::vector<ros2_knowledge_graph_msgs::msg::Node> & get_nodes() {return graph_->nodes;}
  const std::vector<ros2_knowledge_graph_msgs::msg::Edge> & get_edges()
  {
    update_tf_edges();
    return graph_->edges;
  }
  const std::vector<std::string> get_node_names();

  size_t get_num_edges() const;
  size_t get_num_nodes() const;

  void update_node(const ros2_knowledge_graph_msgs::msg::Node & node, bool sync = true);
  bool update_edge(const ros2_knowledge_graph_msgs::msg::Edge & edge, bool sync = true);

protected:
  rclcpp::Node::SharedPtr node_;
  ros2_knowledge_graph_msgs::msg::Graph::UniquePtr graph_;
  std::string graph_id_;
  rclcpp::Time last_ts_;

  void publish_tf(const ros2_knowledge_graph_msgs::msg::Content & content);
  void update_tf_edges();
  void update_tf_edge(ros2_knowledge_graph_msgs::msg::Edge & edge);
  void update_callback(ros2_knowledge_graph_msgs::msg::GraphUpdate::UniquePtr msg);
  void reqsync_timer_callback();

private:
  rclcpp::Publisher<ros2_knowledge_graph_msgs::msg::GraphUpdate>::SharedPtr update_pub_;
  rclcpp::Subscription<ros2_knowledge_graph_msgs::msg::GraphUpdate>::SharedPtr update_sub_;

  rclcpp::TimerBase::SharedPtr reqsync_timer_;
  rclcpp::Time start_time_;

  tf2::BufferCore buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  friend class GraphFactory;
};

class GraphFactory
{
public:
  static GraphNode * getInstance(rclcpp::Node::SharedPtr provided_node)
  {
    if (instance_ == nullptr) {
      instance_ = new GraphNode(provided_node);
    } else if (provided_node != instance_->node_) {
      RCLCPP_WARN(
        provided_node->get_logger(), "Using already existing node [%s]",
        instance_->node_->get_name());
    }
    return instance_;
  }

private:
  static GraphNode * instance_;
};

GraphNode * GraphFactory::instance_ = nullptr;

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
