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

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <list>
#include <map>
#include <algorithm>
#include <sstream>
#include <vector>
#include <mutex>

#include "ros2_knowledge_graph/GraphNode.hpp"

#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"


namespace ros2_knowledge_graph
{

GraphNode::GraphNode(const std::string & provided_node_name)
: node_name_(provided_node_name + "_graph"),
  lp_loader_("ros2_knowledge_graph", "ros2_knowledge_graph::Layer"),
  started_(false)
{
}

void
GraphNode::start()
{
  node_ = std::make_shared<rclcpp::Node>(node_name_);

  std::vector<std::string> default_id, default_type;
  default_type.push_back("ros2_kg_tf_plugin::TFLayer");
  default_id.push_back("TFLayer");

  node_->declare_parameter("layer_plugin_ids", default_id);
  node_->declare_parameter("layer_plugin_types", default_type);
  node_->get_parameter("layer_plugin_ids", layer_ids_);
  node_->get_parameter("layer_plugin_types", layer_types_);

  for (uint i = 0; i != layer_types_.size(); i++) {
    try {
      ros2_knowledge_graph::Layer::Ptr layer =
        lp_loader_.createUniqueInstance(layer_types_[i]);
      layer->configure(node_);
      RCLCPP_DEBUG(
        node_->get_logger(), "Created layer : %s of type %s",
        layer_ids_[i].c_str(), layer_types_[i].c_str());
      layers_.insert({layer_ids_[i], layer});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(node_->get_logger(), "Failed to create layer. Exception: %s", ex.what());
      exit(-1);
    }

    started_ = true;
  }

  last_ts_ = node_->now();

  using namespace std::placeholders;
  update_pub_ = node_->create_publisher<ros2_knowledge_graph_msgs::msg::GraphUpdate>(
    "/graph_updates", rclcpp::QoS(1000).reliable());
  update_sub_ = node_->create_subscription<ros2_knowledge_graph_msgs::msg::GraphUpdate>(
    "/graph_updates", rclcpp::QoS(1000).reliable(),
    std::bind(&GraphNode::update_callback, this, _1));

  rclcpp::spin_some(node_);

  sync_spin_t_ = std::thread(
    [this] {
      rclcpp::spin(this->node_);
    });
  sync_spin_t_.detach();

  RCLCPP_DEBUG(node_->get_logger(), "Waiting 2 seconds for a complete startup...");
  rclcpp::sleep_for(std::chrono::seconds(2));
  RCLCPP_DEBUG(node_->get_logger(), "Starting");


  std::lock_guard<std::mutex> lock(mutex_);

  RCLCPP_DEBUG(node_->get_logger(), "Sending SYNC");
  ros2_knowledge_graph_msgs::msg::GraphUpdate msg;
  msg.stamp = node_->now();
  msg.node_id = node_->get_name();
  msg.operation_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::REQSYNC;
  msg.element_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::GRAPH;
  msg.object = graph_.to_string();
  update_pub_->publish(msg);
}

void
GraphNode::update_callback(const ros2_knowledge_graph_msgs::msg::GraphUpdate::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const auto & author_id = msg->node_id;
  const auto & element = msg->element_type;
  const auto & operation = msg->operation_type;
  const auto & object_data = msg->object;
  const auto & ts = msg->stamp;

  if (author_id == node_->get_name()) {
    return;
  }

  if (rclcpp::Time(ts) < last_ts_ &&
    operation != ros2_knowledge_graph_msgs::msg::GraphUpdate::REQSYNC &&
    operation != ros2_knowledge_graph_msgs::msg::GraphUpdate::SYNC)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "UNORDERER UPDATE [%zd] %lf > %lf in [%s]",
      operation,
      last_ts_.seconds(), rclcpp::Time(ts).seconds(), msg->object.c_str());
  }

  switch (element) {
    case ros2_knowledge_graph_msgs::msg::GraphUpdate::NODE:
      {
        if (author_id == node_->get_name()) {
          return;
        }

        switch (operation) {
          case ros2_knowledge_graph_msgs::msg::GraphUpdate::ADD:
            {
              Node node;
              node.from_string(object_data);
              graph_.add_node(node);
              RCLCPP_DEBUG(
                node_->get_logger(), "[%lf]\t(%s)\tADD NODE %s",
                rclcpp::Time(ts).seconds(),
                msg->node_id.c_str(), object_data.c_str());
              break;
            }

          case ros2_knowledge_graph_msgs::msg::GraphUpdate::REMOVE:
            {
              Node node;
              node.from_string(object_data);
              graph_.remove_node(node.name);
              RCLCPP_DEBUG(
                node_->get_logger(), "[%lf]\t(%s)\tREMOVE NODE %s",
                rclcpp::Time(ts).seconds(),
                msg->node_id.c_str(), object_data.c_str());
              break;
            }
        }
      }
      break;

    case ros2_knowledge_graph_msgs::msg::GraphUpdate::EDGE:
      {
        if (author_id == node_->get_name()) {
          return;
        }

        Edge edge;
        edge.from_string(object_data);

        switch (operation) {
          case ros2_knowledge_graph_msgs::msg::GraphUpdate::ADD:
            graph_.add_edge(edge);

            RCLCPP_DEBUG(
              node_->get_logger(), "[%lf]\t(%s)\tADD EDGE %s",
              rclcpp::Time(ts).seconds(),
              msg->node_id.c_str(), object_data.c_str());
            break;

          case ros2_knowledge_graph_msgs::msg::GraphUpdate::REMOVE:
            graph_.remove_edge(edge);
            RCLCPP_DEBUG(
              node_->get_logger(), "[%lf]\t(%s)\tREMOVE NODE %s",
              rclcpp::Time(ts).seconds(),
              msg->node_id.c_str(), object_data.c_str());
            break;
        }
      }
      break;

    case ros2_knowledge_graph_msgs::msg::GraphUpdate::GRAPH:
      {
        switch (operation) {
          case ros2_knowledge_graph_msgs::msg::GraphUpdate::SYNC:
            RCLCPP_DEBUG(
              node_->get_logger(), "[%lf]\t(%s)\tSYNC %s",
              rclcpp::Time(ts).seconds(),
              msg->node_id.c_str(), object_data.c_str());

            if (msg->target_node == node_->get_name()) {
              graph_.from_string(object_data);
              last_ts_ = ts;
            }
            break;

          case ros2_knowledge_graph_msgs::msg::GraphUpdate::REQSYNC:

            RCLCPP_DEBUG(
              node_->get_logger(), "[%lf]\t(%s)\tREQSYNC %s",
              rclcpp::Time(ts).seconds(),
              msg->node_id.c_str(), object_data.c_str());

            if (msg->node_id != node_->get_name()) {
              ros2_knowledge_graph_msgs::msg::GraphUpdate out_msg;
              out_msg.stamp = node_->now();
              out_msg.node_id = node_->get_name();
              out_msg.target_node = msg->node_id;
              out_msg.operation_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::SYNC;
              out_msg.element_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::GRAPH;
              out_msg.object = graph_.to_string();
              update_pub_->publish(out_msg);
            }
            break;
        }
      }
      break;
  }
}


bool
GraphNode::add_node(const Node & node)
{
  std::lock_guard<std::mutex> lock(mutex_);

  Node modificable_node = node;

  for (const auto & layer_id : layer_ids_) {
    layers_[layer_id]->on_add_node(modificable_node);
  }

  if (graph_.can_add_node(modificable_node)) {
    graph_.add_node(modificable_node);
    last_ts_ = node_->now();

    ros2_knowledge_graph_msgs::msg::GraphUpdate msg;
    msg.stamp = last_ts_;
    msg.node_id = node_->get_name();
    msg.operation_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::ADD;
    msg.element_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::NODE;
    msg.object = modificable_node.to_string();

    update_pub_->publish(msg);

    return true;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "NCB unable to add Node [%s]",
      modificable_node.to_string().c_str());

    return false;
  }
}

bool
GraphNode::remove_node(const std::string node)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & layer_id : layer_ids_) {
    layers_[layer_id]->on_remove_node(node);
  }

  if (graph_.can_remove_node(node)) {
    graph_.remove_node(node);
    last_ts_ = node_->now();

    ros2_knowledge_graph_msgs::msg::GraphUpdate msg;
    msg.stamp = last_ts_;
    msg.node_id = node_->get_name();
    msg.operation_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::REMOVE;
    msg.element_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::NODE;
    msg.object = Node{node, ""}.to_string();

    update_pub_->publish(msg);

    return true;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "NCB Unable to remove Node [%s]",
      Node{node, ""}.to_string().c_str());

    return false;
  }
}

bool
GraphNode::exist_node(const std::string node)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_.exist_node(node);
}

boost::optional<Node>
GraphNode::get_node(const std::string node)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto opt_node = graph_.get_node(node);

  if (opt_node.has_value()) {
    Node modificable_node = opt_node.value();
    for (const auto & layer_id : layer_ids_) {
      layers_[layer_id]->on_get_node(modificable_node);
    }
    return modificable_node;
  } else {
    return {};
  }
}

bool
GraphNode::add_edge(const Edge & edge)
{
  Edge modificable_edge = edge;

  for (const auto & layer_id : layer_ids_) {
    layers_[layer_id]->on_add_edge(modificable_edge);
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (graph_.can_add_edge(modificable_edge) && !graph_.exist_edge(modificable_edge)) {
    graph_.add_edge(modificable_edge);
    last_ts_ = node_->now();

    ros2_knowledge_graph_msgs::msg::GraphUpdate msg;
    msg.stamp = last_ts_;
    msg.node_id = node_->get_name();
    msg.operation_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::ADD;
    msg.element_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::EDGE;
    msg.object = modificable_edge.to_string();
    update_pub_->publish(msg);
    return true;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "Unable to add Edge [%s]",
      modificable_edge.to_string().c_str());

    return false;
  }
}

bool
GraphNode::remove_edge(const Edge & edge)
{
  std::lock_guard<std::mutex> lock(mutex_);

  Edge modificable_edge = edge;

  for (const auto & layer_id : layer_ids_) {
    layers_[layer_id]->on_remove_edge(modificable_edge);
  }

  if (graph_.can_remove_edge(modificable_edge)) {
    graph_.remove_edge(modificable_edge);
    last_ts_ = node_->now();

    ros2_knowledge_graph_msgs::msg::GraphUpdate msg;
    msg.stamp = last_ts_;
    msg.node_id = node_->get_name();
    msg.operation_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::REMOVE;
    msg.element_type = ros2_knowledge_graph_msgs::msg::GraphUpdate::EDGE;
    msg.object = modificable_edge.to_string();

    update_pub_->publish(msg);
    return true;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "Unable to remove Edge [%s]",
      modificable_edge.to_string().c_str());

    return false;
  }
}

bool
GraphNode::exist_edge(const Edge & edge)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_.exist_edge(edge);
}


void
GraphNode::get_edges(
  const std::string & source, const std::string & target,
  const std::string & type, std::vector<Edge> & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto opt_edges = graph_.get_edges(source, target);

  std::vector<Edge> edges;
  if (opt_edges.has_value()) {
    for (auto & edge : *opt_edges.value()) {
      Edge modificable_edge = edge;

      for (const auto & layer_id : layer_ids_) {
        layers_[layer_id]->on_get_edge(modificable_edge);
      }
      result.push_back(modificable_edge);
    }
  }
}

std::string
GraphNode::to_string() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_.to_string();
}

void
GraphNode::from_string(const std::string & graph_str)
{
  std::lock_guard<std::mutex> lock(mutex_);
  graph_.from_string(graph_str);
}

size_t
GraphNode::get_num_edges() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_.get_num_edges();
}

size_t
GraphNode::get_num_nodes() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_.get_num_nodes();
}


const std::map<std::string, Node> &
GraphNode::get_nodes()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_.get_nodes();
}

const std::map<ConnectionT, std::vector<Edge>> &
GraphNode::get_edges()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_.get_edges();
}

std::vector<std::string>
GraphNode::get_node_names_by_id(const std::string & expr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_.get_node_names_by_id(expr);
}

std::vector<std::string>
GraphNode::get_node_names_by_type(const std::string & type)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_.get_node_names_by_type(type);
}

std::vector<Edge>
GraphNode::get_edges_from_node(const std::string & node_src_id, const std::string & type)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_.get_edges_from_node(node_src_id, type);
}

std::vector<Edge>
GraphNode::get_edges_from_node_by_data(
  const std::string & node_src_id,
  const std::string & expr,
  const std::string & type)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_.get_edges_from_node_by_data(node_src_id, expr, type);
}

std::vector<Edge>
GraphNode::get_edges_by_data(const std::string & expr, const std::string & type)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return graph_.get_edges_by_data(expr, type);
}

}  // namespace ros2_knowledge_graph
