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

#ifndef ROS2_KNOWLEDGE_GRAPH__GRAPHNODE_HPP_
#define ROS2_KNOWLEDGE_GRAPH__GRAPHNODE_HPP_

#include <boost/optional.hpp>

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <utility>
#include <mutex>
#include <unordered_map>

#include "ros2_knowledge_graph/Graph.hpp"
#include "ros2_knowledge_graph/Layer.hpp"

#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ros2_knowledge_graph
{

using ConnectionT = std::pair<std::string, std::string>;

class GraphNode
{
public:
  using LayerMap = std::unordered_map<std::string, ros2_knowledge_graph::Layer::Ptr>;

  explicit GraphNode(const std::string & provided_node_name);

  void start();
  bool is_started() {return started_;}

  bool add_node(const Node & node);
  bool remove_node(const std::string node);
  bool exist_node(const std::string node);
  boost::optional<Node> get_node(const std::string node);

  bool add_edge(const Edge & edge);

  bool remove_edge(const Edge & edge);
  bool exist_edge(const Edge & edge);
  void get_edges(
    const std::string & source, const std::string & target, const std::string & type,
    std::vector<Edge> & result);

  const std::map<std::string, Node> & get_nodes();
  const std::map<ConnectionT, std::vector<Edge>> & get_edges();

  std::string to_string() const;
  void from_string(const std::string & graph_str);

  size_t get_num_edges() const;
  size_t get_num_nodes() const;

  std::vector<std::string> get_node_names_by_id(const std::string & expr);
  std::vector<std::string> get_node_names_by_type(const std::string & type);
  std::vector<Edge> get_edges_from_node(
    const std::string & node_src_id,
    const std::string & type = "");
  std::vector<Edge> get_edges_from_node_by_data(
    const std::string & node_src_id,
    const std::string & expr,
    const std::string & type = "");
  std::vector<Edge> get_edges_by_data(const std::string & expr, const std::string & type = "");

protected:
  rclcpp::Node::SharedPtr node_;

private:
  bool started_;

  std::string node_name_;

  Graph graph_;

  rclcpp::Publisher<ros2_knowledge_graph_msgs::msg::GraphUpdate>::SharedPtr update_pub_;
  rclcpp::Subscription<ros2_knowledge_graph_msgs::msg::GraphUpdate>::SharedPtr update_sub_;

  void update_callback(const ros2_knowledge_graph_msgs::msg::GraphUpdate::SharedPtr msg);
  mutable std::mutex mutex_;

  std::thread sync_spin_t_;
  rclcpp::Time last_ts_;

  pluginlib::ClassLoader<ros2_knowledge_graph::Layer> lp_loader_;
  LayerMap layers_;
  std::vector<std::string> layer_ids_, layer_types_;
  std::string layer_ids_concat_, current_layer_;
};

}  // namespace ros2_knowledge_graph

#endif  // ROS2_KNOWLEDGE_GRAPH__GRAPHNODE_HPP_
