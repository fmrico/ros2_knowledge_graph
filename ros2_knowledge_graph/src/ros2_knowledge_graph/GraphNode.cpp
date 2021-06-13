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


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <string>
#include <vector>
#include <memory>
#include <regex>

#include "ros2_knowledge_graph_msgs/msg/graph.hpp"
#include "ros2_knowledge_graph_msgs/msg/node.hpp"
#include "ros2_knowledge_graph_msgs/msg/edge.hpp"
#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"

#include "ros2_knowledge_graph/GraphNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_publisher.hpp"


namespace ros2_knowledge_graph
{

using std::placeholders::_1;
using namespace std::chrono_literals;

GraphNode::GraphNode(rclcpp::Node::SharedPtr provided_node)
: GraphNode(
    provided_node->get_node_base_interface(),
    provided_node->get_node_logging_interface(),
    provided_node->get_node_timers_interface(),
    provided_node->get_node_topics_interface(),
    provided_node->get_node_clock_interface())
{
}

GraphNode::GraphNode(rclcpp_lifecycle::LifecycleNode::SharedPtr provided_node)
:  GraphNode(
    provided_node->get_node_base_interface(),
    provided_node->get_node_logging_interface(),
    provided_node->get_node_timers_interface(),
    provided_node->get_node_topics_interface(),
    provided_node->get_node_clock_interface())
{
}


GraphNode::GraphNode(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock)
: node_base_(node_base),
  node_logging_(node_logging),
  node_timers_(node_timers_),
  node_topics_(node_topics),
  node_clock_(node_clock)
{
  tf_listener_= std::make_shared<tf2_ros::TransformListener>(buffer_);

  graph_ = std::make_unique<ros2_knowledge_graph_msgs::msg::Graph>();
  graph_id_ = node_base_->get_name() + std::to_string(node_clock_->get_clock()->now().seconds());

  auto qos_100_rel = rclcpp::QoS(1000).reliable();
  update_pub_  = node_topics_->create_publisher(
    "graph_update", 
    rclcpp::create_publisher_factory<
      ros2_knowledge_graph_msgs::msg::GraphUpdate,
      std::allocator<void>, rclcpp::Publisher<ros2_knowledge_graph_msgs::msg::GraphUpdate, std::allocator<void>>(
        rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>()),
    qos_100_rel);

  /*update_pub_ = node_topics_->create_publisher<ros2_knowledge_graph_msgs::msg::GraphUpdate>(
    "graph_update", rclcpp::QoS(1000).reliable());
  update_sub_ = node_topics_->create_subscription<ros2_knowledge_graph_msgs::msg::GraphUpdate>(
    "graph_update", rclcpp::QoS(100).reliable(),
    std::bind(&GraphNode::update_callback, this, _1));
  tf_publisher_ = node_topics_->create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf",rclcpp::QoS(100));
  static_tf_publisher_ = node_topics_->create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(100).transient_local());
*/
  last_ts_ = node_clock_->get_clock()->now();
  start_time_ = node_clock_->get_clock()->now();

  reqsync_timer_ = rclcpp::create_wall_timer(
    100ms,
    std::bind(&GraphNode::reqsync_timer_callback, this),
    nullptr,
    node_base_.get(),
    node_timers_.get());

  reqsync_timer_callback();
}

void
GraphNode::reqsync_timer_callback()
{
  if ((node_clock_->get_clock()->now() - start_time_).seconds() > 1.0) {
    reqsync_timer_ = nullptr;
  }

  ros2_knowledge_graph_msgs::msg::GraphUpdate hello_msg;
  hello_msg.stamp = node_clock_->get_clock()->now();
  hello_msg.node_id = graph_id_;
  hello_msg.operation_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::REQSYNC;
  hello_msg.element_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::GRAPH;
  hello_msg.graph = *graph_;
  update_pub_->publish(hello_msg);
}

bool
GraphNode::remove_node(const std::string node, bool sync)
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
    if (sync) {
      ros2_knowledge_graph_msgs::msg::GraphUpdate update_msg;
      update_msg.stamp = node_clock_->get_clock()->now();
      update_msg.node_id = graph_id_;
      update_msg.operation_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::REMOVE;
      update_msg.element_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::NODE;
      update_msg.removed_node = node;
      update_pub_->publish(update_msg);
    }

    last_ts_ = node_clock_->get_clock()->now();
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
GraphNode::remove_edge(const ros2_knowledge_graph_msgs::msg::Edge & edge, bool sync)
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
    if (sync) {
      ros2_knowledge_graph_msgs::msg::GraphUpdate update_msg;
      update_msg.stamp = node_clock_->get_clock()->now();
      update_msg.node_id = graph_id_;
      update_msg.operation_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::REMOVE;
      update_msg.element_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::EDGE;
      update_msg.edge = edge;
      update_pub_->publish(update_msg);
    }

    last_ts_ = node_clock_->get_clock()->now();
  }

  return removed;
}

std::vector<ros2_knowledge_graph_msgs::msg::Edge>
GraphNode::get_edges(
  const std::string & source, const std::string & target, uint8_t type)
{
  std::vector<ros2_knowledge_graph_msgs::msg::Edge> ret;

  for (auto & edge : graph_->edges) {
    if (edge.source_node_id == source &&
      edge.target_node_id == target &&
      edge.content.type == type)
    {
      if (type == ros2_knowledge_graph_msgs::msg::Content::TF ||
        type == ros2_knowledge_graph_msgs::msg::Content::STATICTF)
      {
        update_tf_edge(edge);
      }
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
GraphNode::update_node(const ros2_knowledge_graph_msgs::msg::Node & node, bool sync)
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

  if (sync) {
    ros2_knowledge_graph_msgs::msg::GraphUpdate update_msg;
    update_msg.stamp = node_clock_->get_clock()->now();
    update_msg.node_id = graph_id_;
    update_msg.operation_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::UPDATE;
    update_msg.element_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::NODE;
    update_msg.node = node;
    update_pub_->publish(update_msg);
  }

  last_ts_ = node_clock_->get_clock()->now();
}

bool
GraphNode::update_edge(const ros2_knowledge_graph_msgs::msg::Edge & edge, bool sync)
{
  if (!exist_node(edge.source_node_id)) {
    RCLCPP_ERROR_STREAM(
      node_logging_->get_logger(),
      "Node source [" << edge.source_node_id << "] doesn't exist adding edge");
    return false;
  }

  if (!exist_node(edge.target_node_id)) {
    RCLCPP_ERROR_STREAM(
     node_logging_->get_logger(),
      "Node target [" << edge.target_node_id << "] doesn't exist adding edge");
    return false;
  }

  bool found = false;
  auto it = graph_->edges.begin();
  while (!found && it != graph_->edges.end()) {
    if (it->source_node_id == edge.source_node_id &&
      it->target_node_id == edge.target_node_id &&
      it->content.type == edge.content.type && (
        it->content.type != ros2_knowledge_graph_msgs::msg::Content::STRING ||
        (it->content.type == ros2_knowledge_graph_msgs::msg::Content::STRING &&
        it->content.string_value == edge.content.string_value)))
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
      publish_tf(mod_edge.content);
    }
    graph_->edges.push_back(mod_edge);

    if (sync) {
      ros2_knowledge_graph_msgs::msg::GraphUpdate update_msg;
      update_msg.stamp = node_clock_->get_clock()->now();
      update_msg.node_id = graph_id_;
      update_msg.operation_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::UPDATE;
      update_msg.element_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::EDGE;
      update_msg.edge = mod_edge;
      update_pub_->publish(update_msg);
    }

    last_ts_ = node_clock_->get_clock()->now();
  } else {
    if (mod_edge.content.type != ros2_knowledge_graph_msgs::msg::Content::STATICTF &&
      mod_edge.content.type != ros2_knowledge_graph_msgs::msg::Content::TF)
    {
      if (sync) {
        ros2_knowledge_graph_msgs::msg::GraphUpdate update_msg;
        update_msg.stamp = node_clock_->get_clock()->now();
        update_msg.node_id = graph_id_;
        update_msg.operation_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::UPDATE;
        update_msg.element_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::EDGE;
        update_msg.edge = mod_edge;
        update_pub_->publish(update_msg);
      }

      last_ts_ = node_clock_->get_clock()->now();
    } else {
      mod_edge.content.tf_value.header.frame_id = edge.source_node_id;
      mod_edge.content.tf_value.child_frame_id = edge.target_node_id;
      publish_tf(mod_edge.content);
    }
  }

  return true;
}

void
GraphNode::update_callback(ros2_knowledge_graph_msgs::msg::GraphUpdate::UniquePtr msg)
{
  const auto & author_id = msg->node_id;
  const auto & element = msg->element_type;
  const auto & operation = msg->operation_type;
  const auto & ts = msg->stamp;

  if (author_id == graph_id_) {
    return;
  }

  if (rclcpp::Time(ts) < last_ts_ &&
    operation != ros2_knowledge_graph_msgs::msg::GraphUpdate::REQSYNC &&
    operation != ros2_knowledge_graph_msgs::msg::GraphUpdate::SYNC)
  {
    RCLCPP_ERROR(
     node_logging_->get_logger(), "UNORDERER UPDATE [%zd] %lf > %lf",
      operation,
      last_ts_.seconds(), rclcpp::Time(ts).seconds());
  }

  switch (element) {
    case ros2_knowledge_graph_msgs::msg::GraphUpdate::NODE:
      {
        if (author_id == graph_id_) {
          return;
        }

        switch (operation) {
          case ros2_knowledge_graph_msgs::msg::GraphUpdate::UPDATE:
            {
              update_node(msg->node, false);
              break;
            }

          case ros2_knowledge_graph_msgs::msg::GraphUpdate::REMOVE:
            {
              remove_node(msg->removed_node, false);
              break;
            }
        }
      }
      break;

    case ros2_knowledge_graph_msgs::msg::GraphUpdate::EDGE:
      {
        if (author_id == graph_id_) {
          return;
        }

        switch (operation) {
          case ros2_knowledge_graph_msgs::msg::GraphUpdate::UPDATE:
            update_edge(msg->edge, false);
            break;

          case ros2_knowledge_graph_msgs::msg::GraphUpdate::REMOVE:
            remove_edge(msg->edge, false);
            break;
        }
      }
      break;

    case ros2_knowledge_graph_msgs::msg::GraphUpdate::GRAPH:
      {
        switch (operation) {
          case ros2_knowledge_graph_msgs::msg::GraphUpdate::SYNC:

            if (msg->target_node == graph_id_) {
              reqsync_timer_ = nullptr;

              *graph_ = msg->graph;
              last_ts_ = ts;
            }
            break;

          case ros2_knowledge_graph_msgs::msg::GraphUpdate::REQSYNC:

            if (msg->node_id != node_base_->get_name()) {
              ros2_knowledge_graph_msgs::msg::GraphUpdate out_msg;
              out_msg.stamp = node_clock_->get_clock()->now();
              out_msg.node_id = graph_id_;
              out_msg.target_node = msg->node_id;
              out_msg.operation_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::SYNC;
              out_msg.element_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::GRAPH;
              out_msg.graph = *graph_;
              update_pub_->publish(out_msg);
            }
            break;
        }
      }
      break;
  }
}

void
GraphNode::publish_tf(const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::TF) {
    tf2_msgs::msg::TFMessage msg;
    msg.transforms.push_back(content.tf_value);
    tf_publisher_->publish(msg);
  } else if (content.type == ros2_knowledge_graph_msgs::msg::Content::STATICTF) {
    tf2_msgs::msg::TFMessage msg;
    msg.transforms.push_back(content.tf_value);
    static_tf_publisher_->publish(msg);
  } else {
    RCLCPP_ERROR(node_logging_->get_logger(), "Trying to publish tf with a non-tf edge");
  }
}

void
GraphNode::update_tf_edges()
{
  for (auto & edge : graph_->edges) {
    if (edge.content.type == ros2_knowledge_graph_msgs::msg::Content::TF ||
      edge.content.type == ros2_knowledge_graph_msgs::msg::Content::STATICTF)
    {
      update_tf_edge(edge);
    }
  }
}

void
GraphNode::update_tf_edge(ros2_knowledge_graph_msgs::msg::Edge & edge)
{
  try {
    edge.content.tf_value = buffer_.lookupTransform(
      edge.source_node_id, edge.target_node_id, tf2::TimePointZero);
  } catch (tf2::LookupException e) {
  }
}

std::vector<ros2_knowledge_graph_msgs::msg::Edge>
GraphNode::get_edges_from_node_by_data(
  const std::string & source, const std::string & expr)
{
  std::vector<ros2_knowledge_graph_msgs::msg::Edge> ret;

  for (auto & edge : graph_->edges) {
    if ( (edge.source_node_id == source) &&
      (edge.content.type == ros2_knowledge_graph_msgs::msg::Content::STRING) )
    {
      if (std::regex_match(edge.content.string_value, std::regex(expr))) {
        ret.push_back(edge);
      }
    }
  }
  return ret;
}

}  // namespace ros2_knowledge_graph
