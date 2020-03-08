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

#include <string>
#include <vector>
#include <regex>
#include <iostream>
#include <memory>

#include "ros2_knowledge_graph/Graph.hpp"
#include "ros2_knowledge_graph/Types.hpp"

#include "gtest/gtest.h"


TEST(ros2_knowledge_graph, graph_operations)
{
  ros2_knowledge_graph::Graph graph;

  graph.add_node(ros2_knowledge_graph::Node{"paco", "person"});
  ASSERT_TRUE(graph.exist_node("paco"));
  graph.remove_node("paco");
  ASSERT_FALSE(graph.exist_node("paco"));
  ASSERT_EQ(graph.get_num_nodes(), 0);

  ASSERT_FALSE(graph.exist_node("r2d2"));
  graph.add_node(ros2_knowledge_graph::Node{"r2d2", "robot"});
  ASSERT_TRUE(graph.exist_node("r2d2"));
  ASSERT_FALSE(graph.exist_node("kitchen"));
  graph.add_node(ros2_knowledge_graph::Node{"kitchen", "room"});
  ASSERT_TRUE(graph.exist_node("kitchen"));

  graph.add_node(ros2_knowledge_graph::Node{"room1", "room"});
  graph.add_node(ros2_knowledge_graph::Node{"room2", "room"});

  graph.remove_node("r2d2");
  ASSERT_FALSE(graph.exist_node("r2d2"));
  graph.add_node(ros2_knowledge_graph::Node{"r2d2", "robot"});
  ASSERT_TRUE(graph.exist_node("r2d2"));

  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_TRUE(graph.exist_edge(ros2_knowledge_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_TRUE(graph.remove_edge(ros2_knowledge_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_FALSE(graph.exist_edge(ros2_knowledge_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"is_near", "symbolic", "kitchen", "r2d2"}));
  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"is_verynear", "symbolic", "kitchen", "r2d2"}));
  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"is_verynear_very", "sympedal", "kitchen", "r2d2"}));
  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"related", "symbolic", "kitchen", "r2d2"}));
  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"related", "metric", "kitchen", "r2d2"}));

  ASSERT_EQ(graph.get_node_names_by_id("room[[:alnum:]_]*").size(), 2);
  ASSERT_EQ(graph.get_node_names_by_id("kitchen").size(), 1);
  ASSERT_EQ(graph.get_node_names_by_type("room").size(), 3);
  ASSERT_EQ(graph.get_node_names_by_type("robot").size(), 1);
  ASSERT_EQ(graph.get_edges_from_node("kitchen").size(), 5);
  ASSERT_EQ(graph.get_edges_from_node("kitchen", "symbolic").size(), 3);
  ASSERT_EQ(graph.get_edges_from_node_by_data("kitchen", "is[[:alnum:]_]*").size(), 3);
  ASSERT_EQ(graph.get_edges_from_node_by_data("kitchen", "is[[:alnum:]_]*", "symbolic").size(), 2);
  ASSERT_EQ(graph.get_edges_by_data("is[[:alnum:]_]*").size(), 4);

  ASSERT_EQ(graph.get_num_edges(), 6);
  graph.remove_node("r2d2");
  ASSERT_EQ(graph.get_num_edges(), 0);


  std::string graph_str = graph.to_string();
  ros2_knowledge_graph::Graph graph2;
  graph2.from_string(graph_str);
  std::string graph2_str = graph2.to_string();

  ASSERT_EQ(graph_str, graph2_str);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
