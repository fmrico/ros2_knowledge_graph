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

#ifndef ROS2_KNOWLEDGE_GRAPH__GRAPHUTILS_HPP_
#define ROS2_KNOWLEDGE_GRAPH__GRAPHUTILS_HPP_

#include <optional>
#include <iostream>
#include <vector>

#include <type_traits>

#include "ros2_knowledge_graph_msgs/msg/graph.hpp"
#include "ros2_knowledge_graph_msgs/msg/node.hpp"
#include "ros2_knowledge_graph_msgs/msg/edge.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


namespace ros2_knowledge_graph
{

ros2_knowledge_graph_msgs::msg::Node new_node(
  const std::string & node_name,
  const std::string & node_class)
{
  ros2_knowledge_graph_msgs::msg::Node ret;
  ret.node_name = node_name;
  ret.node_class = node_class;

  return ret;
}

template<class T>
ros2_knowledge_graph_msgs::msg::Content new_content(const T & content, const bool static_tf = false)
{
  ros2_knowledge_graph_msgs::msg::Content ret;

  if constexpr (std::is_same<T, bool>::value) {
    ret.bool_value = content;
    ret.type = ros2_knowledge_graph_msgs::msg::Content::BOOL;
  } else if constexpr (std::is_same<T, int>::value) {
    ret.int_value = content;
    ret.type = ros2_knowledge_graph_msgs::msg::Content::INT;
  } else if constexpr (std::is_same<T, float>::value) {
    ret.float_value = content;
    ret.type = ros2_knowledge_graph_msgs::msg::Content::FLOAT;
  } else if constexpr (std::is_same<T, double>::value) {
    ret.double_value = content;
    ret.type = ros2_knowledge_graph_msgs::msg::Content::DOUBLE;
  } else if constexpr (std::is_same<T, std::string>::value) {
    ret.string_value = content;
    ret.type = ros2_knowledge_graph_msgs::msg::Content::STRING;
  } else if constexpr (std::is_same<T, std::vector<bool>>::value) {
    ret.bool_vector = content;
    ret.type = ros2_knowledge_graph_msgs::msg::Content::VBOOL;
  } else if constexpr (std::is_same<T, std::vector<int>>::value) {
    ret.int_vector = content;
    ret.type = ros2_knowledge_graph_msgs::msg::Content::VINT;
  } else if constexpr (std::is_same<T, std::vector<float>>::value) {
    ret.float_vector = content;
    ret.type = ros2_knowledge_graph_msgs::msg::Content::VFLOAT;
  } else if constexpr (std::is_same<T, std::vector<double>>::value) {
    ret.double_vector = content;
    ret.type = ros2_knowledge_graph_msgs::msg::Content::VDOUBLE;
  } else if constexpr (std::is_same<T, std::vector<std::string>>::value) {
    ret.string_vector = content;
    ret.type = ros2_knowledge_graph_msgs::msg::Content::VSTRING;
  } else if constexpr (std::is_same<T, geometry_msgs::msg::PoseStamped>::value) {
    ret.pose_value = content;
    ret.type = ros2_knowledge_graph_msgs::msg::Content::POSE;
  } else if constexpr (std::is_same<T, geometry_msgs::msg::TransformStamped>::value) {
    ret.tf_value = content;
    if (static_tf) {
      ret.type = ros2_knowledge_graph_msgs::msg::Content::STATICTF;
    } else {
      ret.type = ros2_knowledge_graph_msgs::msg::Content::TF;
    }
  } else if constexpr (std::is_same<T, std::vector<geometry_msgs::msg::PoseStamped>>::value) {
    ret.pose_vector = content;
    ret.type = ros2_knowledge_graph_msgs::msg::Content::VPOSE;
  } else if constexpr (std::is_same<T, std::vector<geometry_msgs::msg::TransformStamped>>::value) {
    ret.tf_vector = content;
    ret.type = ros2_knowledge_graph_msgs::msg::Content::VTF;
  } else {
    ret.type = ros2_knowledge_graph_msgs::msg::Content::ERROR;
  }

  return ret;
}

template<class T>
ros2_knowledge_graph_msgs::msg::Edge new_edge(
  const std::string & edge_source,
  const std::string & edge_target,
  const T & content, bool static_tf = false)
{
  ros2_knowledge_graph_msgs::msg::Edge ret;
  ret.source_node_id = edge_source;
  ret.target_node_id = edge_target;
  ret.content = new_content<T>(content, static_tf);

  return ret;
}

template<class T>
std::optional<T> get_content(const ros2_knowledge_graph_msgs::msg::Content & content)
{
  return {};
}

template<>
std::optional<bool> get_content(const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::BOOL) {
    return content.bool_value;
  } else {
    return {};
  }
}

template<>
std::optional<int> get_content(const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::INT) {
    return content.int_value;
  } else {
    return {};
  }
}

template<>
std::optional<float> get_content(const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::FLOAT) {
    return content.float_value;
  } else {
    return {};
  }
}

template<>
std::optional<double> get_content(const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::DOUBLE) {
    return content.double_value;
  } else {
    return {};
  }
}

template<>
std::optional<std::string> get_content(
  const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::STRING) {
    return content.string_value;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<bool>> get_content(
  const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::VBOOL) {
    return content.bool_vector;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<int>> get_content(
  const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::VINT) {
    return content.int_vector;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<float>> get_content(
  const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::VFLOAT) {
    return content.float_vector;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<double>> get_content(
  const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::VDOUBLE) {
    return content.double_vector;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<std::string>> get_content(
  const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::VSTRING) {
    return content.string_vector;
  } else {
    return {};
  }
}

template<>
std::optional<geometry_msgs::msg::PoseStamped> get_content(
  const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::POSE) {
    return content.pose_value;
  } else {
    return {};
  }
}

template<>
std::optional<geometry_msgs::msg::TransformStamped> get_content(
  const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::TF ||
    content.type == ros2_knowledge_graph_msgs::msg::Content::STATICTF) 
  {
    return content.tf_value;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<geometry_msgs::msg::PoseStamped>> get_content(
  const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::VPOSE) {
    return content.pose_vector;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<geometry_msgs::msg::TransformStamped>> get_content(
  const ros2_knowledge_graph_msgs::msg::Content & content)
{
  if (content.type == ros2_knowledge_graph_msgs::msg::Content::VTF) {
    return content.tf_vector;
  } else {
    return {};
  }
}

template<class T>
bool add_property(ros2_knowledge_graph_msgs::msg::Node & node, const std::string key, const T & content)
{
  bool found = false;
  auto newc = new_content<T>(content);

  if (newc.type == ros2_knowledge_graph_msgs::msg::Content::ERROR) {
    std::cerr << "Adding a property of type ERROR" << std::endl;
    return false;
  }

  auto it = node.properties.begin();
  while (!found && it!= node.properties.end()) {
    if (it->key == key) {
      found = true;
      it->value = newc;
    } else {
      ++it;
    }
  }

  if (!found) {
    ros2_knowledge_graph_msgs::msg::Property prop;
    prop.key = key;
    prop.value = newc;
    node.properties.push_back(prop);
  }

  return true;
}

template<class T>
std::optional<T> get_property(ros2_knowledge_graph_msgs::msg::Node & node, const std::string key)
{
  auto it = node.properties.begin();
  while (it!= node.properties.end()) {
    if (it->key == key) {
      return get_content<T>(it->value);
    } else {
      ++it;
    }
  }

  return {};
}

uint8_t
get_property_type(ros2_knowledge_graph_msgs::msg::Node & node, const std::string key)
{
  auto it = node.properties.begin();
  while (it!= node.properties.end()) {
    if (it->key == key) {
      return it->value.type;
    } else {
      ++it;
    }
  }

  return ros2_knowledge_graph_msgs::msg::Content::ERROR;
}

const std::vector<std::string>
get_properties(ros2_knowledge_graph_msgs::msg::Node & node)
{
  std::vector<std::string> ret;
  for (const auto & property : node.properties) {
    ret.push_back(property.key);
  }

  return ret;
}

}  // namespace ros2_knowledge_graph

#endif  // ROS2_KNOWLEDGE_GRAPH__GRAPHUTILS_HPP_
