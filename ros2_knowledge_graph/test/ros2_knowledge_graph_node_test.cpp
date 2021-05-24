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
#include <random>

#include "ros2_knowledge_graph/GraphNode.hpp"

#include "gtest/gtest.h"


TEST(ros2_knowledge_graphnode, graph_operations)
{
  ros2_knowledge_graph::GraphNode graph("graph");

  graph.start();

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
  ASSERT_TRUE(
    graph.add_edge(
      ros2_knowledge_graph::Edge{
    "is_verynear", "symbolic", "kitchen", "r2d2"}));
  ASSERT_TRUE(
    graph.add_edge(
      ros2_knowledge_graph::Edge{
    "is_verynear_very", "sympedal", "kitchen", "r2d2"}));
  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"related", "symbolic", "kitchen", "r2d2"}));
  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"related", "metric", "kitchen", "r2d2"}));

  ASSERT_EQ(graph.get_node_names_by_id("room[[:alnum:]_]*").size(), 2);
  ASSERT_EQ(graph.get_node_names_by_id("kitchen").size(), 1);
  ASSERT_EQ(graph.get_node_names_by_type("room").size(), 3);
  ASSERT_EQ(graph.get_node_names_by_type("robot").size(), 1);
  ASSERT_EQ(graph.get_edges_from_node("kitchen").size(), 5);
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

TEST(ros2_knowledge_graphnode, graph_twin)
{
  ros2_knowledge_graph::GraphNode graph("graph");
  ros2_knowledge_graph::GraphNode graph2("graph2");

  graph.start();
  graph2.start();

  graph.add_node(
    ros2_knowledge_graph::Node{"paco", "person", {{"mobile", "555-456"}, {"age", "42"}}});

  graph.get_node("paco").value().properties["gender"] = "male";

  auto test_node = rclcpp::Node::make_shared("test_node");
  auto start = test_node->now();
  while ((test_node->now() - start).seconds() < 0.001) {}

  ASSERT_TRUE(graph.exist_node("paco"));
  ASSERT_TRUE(graph2.exist_node("paco"));

  ASSERT_EQ(graph2.get_node("paco").value().properties["age"], "42");
  ASSERT_EQ(graph2.get_node("paco").value().properties["gender"], "male");

  graph.remove_node("paco");
  start = test_node->now();
  while ((test_node->now() - start).seconds() < 0.001) {}

  ASSERT_FALSE(graph.exist_node("paco"));
  ASSERT_EQ(graph.get_num_nodes(), 0);
  ASSERT_FALSE(graph2.exist_node("paco"));
  ASSERT_EQ(graph2.get_num_nodes(), 0);

  ASSERT_FALSE(graph.exist_node("r2d2"));
  ASSERT_FALSE(graph2.exist_node("r2d2"));
  graph.add_node(ros2_knowledge_graph::Node{"r2d2", "robot"});

  start = test_node->now();
  while ((test_node->now() - start).seconds() < 0.001) {}

  ASSERT_TRUE(graph.exist_node("r2d2"));
  ASSERT_FALSE(graph.exist_node("kitchen"));
  ASSERT_TRUE(graph2.exist_node("r2d2"));
  ASSERT_FALSE(graph2.exist_node("kitchen"));
  graph.add_node(ros2_knowledge_graph::Node{"kitchen", "room"});

  start = test_node->now();
  while ((test_node->now() - start).seconds() < 0.001) {}

  ASSERT_TRUE(graph.exist_node("kitchen"));
  ASSERT_TRUE(graph2.exist_node("kitchen"));

  graph.add_node(ros2_knowledge_graph::Node{"room1", "room"});
  graph.add_node(ros2_knowledge_graph::Node{"room2", "room"});

  graph.remove_node("r2d2");

  start = test_node->now();
  while ((test_node->now() - start).seconds() < 0.001) {}

  ASSERT_FALSE(graph.exist_node("r2d2"));
  ASSERT_FALSE(graph2.exist_node("r2d2"));
  graph.add_node(ros2_knowledge_graph::Node{"r2d2", "robot"});

  start = test_node->now();
  while ((test_node->now() - start).seconds() < 0.001) {}

  ASSERT_TRUE(graph.exist_node("r2d2"));
  ASSERT_TRUE(graph2.exist_node("r2d2"));

  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));

  start = test_node->now();
  while ((test_node->now() - start).seconds() < 0.001) {}

  ASSERT_TRUE(graph.exist_edge(ros2_knowledge_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_TRUE(graph2.exist_edge(ros2_knowledge_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_TRUE(graph.remove_edge(ros2_knowledge_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));

  start = test_node->now();
  while ((test_node->now() - start).seconds() < 0.001) {}

  ASSERT_FALSE(graph.exist_edge(ros2_knowledge_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_FALSE(graph2.exist_edge(ros2_knowledge_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"is", "symbolic", "r2d2", "kitchen"}));
  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"is_near", "symbolic", "kitchen", "r2d2"}));
  ASSERT_TRUE(
    graph.add_edge(
      ros2_knowledge_graph::Edge{
    "is_verynear", "symbolic", "kitchen", "r2d2"}));
  ASSERT_TRUE(
    graph.add_edge(
      ros2_knowledge_graph::Edge{
    "is_verynear_very", "sympedal", "kitchen", "r2d2"}));
  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"related", "symbolic", "kitchen", "r2d2"}));
  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"related", "metric", "kitchen", "r2d2"}));

  start = test_node->now();
  while ((test_node->now() - start).seconds() < 0.001) {}


  ASSERT_EQ(graph.get_node_names_by_id("room[[:alnum:]_]*").size(), 2);
  ASSERT_EQ(graph2.get_node_names_by_id("room[[:alnum:]_]*").size(), 2);
  ASSERT_EQ(graph.get_node_names_by_id("kitchen").size(), 1);
  ASSERT_EQ(graph2.get_node_names_by_id("kitchen").size(), 1);
  ASSERT_EQ(graph.get_node_names_by_type("room").size(), 3);
  ASSERT_EQ(graph2.get_node_names_by_type("room").size(), 3);
  ASSERT_EQ(graph.get_node_names_by_type("robot").size(), 1);
  ASSERT_EQ(graph2.get_node_names_by_type("robot").size(), 1);
  ASSERT_EQ(graph.get_edges_from_node("kitchen").size(), 5);
  ASSERT_EQ(graph2.get_edges_from_node_by_data("kitchen", "is[[:alnum:]_]*").size(), 3);
  ASSERT_EQ(graph.get_edges_from_node_by_data("kitchen", "is[[:alnum:]_]*").size(), 3);
  ASSERT_EQ(graph2.get_edges_from_node_by_data("kitchen", "is[[:alnum:]_]*").size(), 3);
  ASSERT_EQ(graph.get_edges_from_node_by_data("kitchen", "is[[:alnum:]_]*", "symbolic").size(), 2);
  ASSERT_EQ(graph2.get_edges_from_node_by_data("kitchen", "is[[:alnum:]_]*", "symbolic").size(), 2);
  ASSERT_EQ(graph.get_edges_by_data("is[[:alnum:]_]*").size(), 4);
  ASSERT_EQ(graph2.get_edges_by_data("is[[:alnum:]_]*").size(), 4);

  ASSERT_EQ(graph.get_num_edges(), 6);
  graph.remove_node("r2d2");

  start = test_node->now();
  while ((test_node->now() - start).seconds() < 0.001) {}

  ASSERT_EQ(graph.get_num_edges(), 0);
  ASSERT_EQ(graph2.get_num_edges(), 0);

  ASSERT_EQ(graph.to_string(), graph2.to_string());
}

TEST(ros2_knowledge_graphnode, graph_stress)
{
  const int NUM_GRAPHS = 10;
  std::vector<std::shared_ptr<ros2_knowledge_graph::GraphNode>> graph_vector(NUM_GRAPHS);
  for (int i = 0; i < NUM_GRAPHS; i++) {
    graph_vector[i] = std::make_shared<ros2_knowledge_graph::GraphNode>(
      "graph_" + std::to_string(i));
  }

  auto test_node = rclcpp::Node::make_shared("test_node");
  std::default_random_engine generator;
  std::uniform_int_distribution<int> graph_dist(0, NUM_GRAPHS - 1);
  std::uniform_int_distribution<int> node_dist(1, 10);
  std::uniform_int_distribution<uint8_t> op_dist(0, 5);
  std::uniform_int_distribution<uint8_t> type_dist(0, 1);

  rclcpp::Rate rate(200);
  auto start = test_node->now();
  while (rclcpp::ok() && (test_node->now() - start).seconds() < 40) {
    uint8_t type = type_dist(generator);
    uint8_t operation = op_dist(generator);
    int graph_id = graph_dist(generator);
    int node_id = node_dist(generator);

    if (!graph_vector[graph_id]->is_started()) {
      graph_vector[graph_id]->start();
    }

    switch (type) {
      case ros2_knowledge_graph_msgs::msg::GraphUpdate::NODE:
        switch (operation) {
          case ros2_knowledge_graph_msgs::msg::GraphUpdate::ADD:
            graph_vector[graph_id]->add_node(
              ros2_knowledge_graph::Node{"node_" + std::to_string(node_id), "test"});
            break;
          case ros2_knowledge_graph_msgs::msg::GraphUpdate::REMOVE:
            graph_vector[graph_id]->remove_node("node_" + std::to_string(node_id));
            break;
        }
        break;
      case ros2_knowledge_graph_msgs::msg::GraphUpdate::EDGE:
        if (operation == 0) {
          graph_vector[graph_id]->remove_edge(
            ros2_knowledge_graph::Edge{
          "link", "test",
          "node_" + std::to_string(node_dist(generator)),
          "node_" + std::to_string(node_dist(generator))});
        } else {
          graph_vector[graph_id]->add_edge(
            ros2_knowledge_graph::Edge{
          "link", "test",
          "node_" + std::to_string(node_dist(generator)),
          "node_" + std::to_string(node_dist(generator))});
        }
        break;
    }

    rate.sleep();
  }

  for (int i = 1; i < NUM_GRAPHS; i++) {
    ASSERT_EQ(graph_vector[i]->to_string(), graph_vector[0]->to_string());
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
