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

#ifndef ROS2_KNOWLEDGE_GRAPH__TYPES_HPP_
#define ROS2_KNOWLEDGE_GRAPH__TYPES_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <assert.h>
#include <string>
#include <vector>
#include <map>

namespace ros2_knowledge_graph
{

std::vector<std::string> tokenize(const std::string & text, const std::string & delim);

struct Node
{
  std::string name;
  std::string type;

  std::map<std::string, std::string> properties {};

  std::string to_string() const
  {
    std::string ret;
    ret = "node::" + name + "::" + type;

    for (const auto & property : properties) {
      ret = ret + "::" + property.first + ":" + property.second;
    }

    return ret;
  }

  void from_string(const std::string & node_str)
  {
    auto tokens = tokenize(node_str, "::");
    name = tokens[1];
    type = tokens[2];

    for (int i = 3; i < tokens.size(); i++) {
      auto prop_token = tokenize(tokens[i], ":");
      properties[prop_token[0]] = prop_token[1];
    }
  }
};

class Edge
{
public:
  std::string content;
  std::string type;

  std::string source;
  std::string target;

  std::string to_string() const
  {
    return "edge::" + source + "->" + target + "::" + content + "::" + type;
  }

  void from_string(const std::string & edge_str)
  {
    auto tokens = tokenize(edge_str, "::");
    assert(tokens.size() == 4);
    auto conn_tokens = tokenize(tokens[1], "->");
    assert(conn_tokens.size() == 2);

    source = conn_tokens[0];
    target = conn_tokens[1];

    content = tokens[2];
    type = tokens[3];
  }
};

bool operator==(const Node & op1, const Node & op2);
bool operator==(const Edge & op1, const Edge & op2);

}  // namespace ros2_knowledge_graph

#endif  // ROS2_KNOWLEDGE_GRAPH__TYPES_HPP_
