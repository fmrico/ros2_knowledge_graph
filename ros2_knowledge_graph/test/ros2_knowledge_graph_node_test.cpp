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

#include <random>
#include <memory>
#include <string>
#include <vector>

#include "ros2_knowledge_graph_msgs/msg/graph.hpp"
#include "ros2_knowledge_graph_msgs/msg/node.hpp"
#include "ros2_knowledge_graph_msgs/msg/edge.hpp"
#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"

#include "ros2_knowledge_graph/GraphNode.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"

#include "rclcpp/rclcpp.hpp"

#include "gtest/gtest.h"

class GraphTest : public ros2_knowledge_graph::GraphNode
{
public:
  explicit GraphTest(rclcpp::Node::SharedPtr provided_node)
  : GraphNode(provided_node)
  {}

  const ros2_knowledge_graph_msgs::msg::Graph & get_graph() {return *graph_;}

  void update_tfs() {update_tf_edges();}
};

TEST(ros2_knowledge_graphnode, graph_operations)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  GraphTest graph_node(test_node);

  auto node_1 = ros2_knowledge_graph::new_node("r2d2", "robot");
  ASSERT_TRUE(graph_node.get_graph().nodes.empty());

  graph_node.update_node(node_1);

  ASSERT_FALSE(graph_node.get_graph().nodes.empty());
  ASSERT_EQ(graph_node.get_graph().nodes.size(), 1u);
  ASSERT_EQ(graph_node.get_nodes().size(), 1u);
  ASSERT_EQ(graph_node.get_nodes()[0], node_1);
  ASSERT_EQ(graph_node.get_node_names().size(), 1u);
  ASSERT_EQ(graph_node.get_node_names()[0], "r2d2");

  graph_node.remove_node("r2d2");
  ASSERT_TRUE(graph_node.get_graph().nodes.empty());
  ASSERT_EQ(graph_node.get_graph().nodes.size(), 0);
  ASSERT_EQ(graph_node.get_nodes().size(), 0);
  ASSERT_EQ(graph_node.get_node_names().size(), 0);

  graph_node.update_node(node_1);
  auto node_2 = ros2_knowledge_graph::new_node("paco", "person");
  graph_node.update_node(node_2);

  ASSERT_FALSE(graph_node.get_graph().nodes.empty());
  ASSERT_EQ(graph_node.get_graph().nodes.size(), 2u);
  ASSERT_EQ(graph_node.get_nodes().size(), 2u);
  ASSERT_EQ(graph_node.get_node_names().size(), 2u);

  ASSERT_FALSE(graph_node.get_node("c3po").has_value());
  ASSERT_TRUE(graph_node.get_node("r2d2").has_value());
  auto node_1_ret = graph_node.get_node("r2d2").value();
  ASSERT_TRUE(ros2_knowledge_graph::get_properties(node_1_ret).empty());
  ros2_knowledge_graph::add_property(node_1_ret, "wheels", 2);
  ros2_knowledge_graph::add_property(node_1_ret, "wheels", 4);
  graph_node.update_node(node_1_ret);


  ASSERT_FALSE(graph_node.get_graph().nodes.empty());
  ASSERT_EQ(graph_node.get_graph().nodes.size(), 2u);
  ASSERT_EQ(graph_node.get_nodes().size(), 2u);
  ASSERT_EQ(graph_node.get_node_names().size(), 2u);
  ASSERT_TRUE(graph_node.exist_node("r2d2"));
  ASSERT_FALSE(graph_node.exist_node("c3po"));

  auto node_2_ret = graph_node.get_node("r2d2").value();
  ASSERT_FALSE(ros2_knowledge_graph::get_properties(node_2_ret).empty());
  ASSERT_EQ(ros2_knowledge_graph::get_properties(node_2_ret)[0], "wheels");
  ASSERT_EQ(
    ros2_knowledge_graph::get_property_type(node_2_ret, "wheels"),
    ros2_knowledge_graph_msgs::msg::Content::INT);
  auto prop_wheels = ros2_knowledge_graph::get_property<int>(node_2_ret, "wheels");
  ASSERT_TRUE(prop_wheels.has_value());
  ASSERT_EQ(prop_wheels.value(), 4);


  ASSERT_TRUE(graph_node.get_graph().edges.empty());
  ASSERT_EQ(graph_node.get_num_edges(), 0);
  ASSERT_EQ(graph_node.get_num_nodes(), 2);

  auto edge_1 = ros2_knowledge_graph::new_edge<std::string>("r2d2", "paco", "talks");
  graph_node.update_edge(edge_1);

  ASSERT_EQ(graph_node.get_num_edges(), 1);

  ASSERT_TRUE(graph_node.get_edges<int>("r2d2", "paco").empty());
  ASSERT_TRUE(graph_node.get_edges<bool>("r2d2", "paco").empty());
  ASSERT_TRUE(graph_node.get_edges<float>("r2d2", "paco").empty());
  ASSERT_TRUE(graph_node.get_edges<double>("r2d2", "paco").empty());
  ASSERT_TRUE(graph_node.get_edges<geometry_msgs::msg::PoseStamped>("r2d2", "paco").empty());
  ASSERT_TRUE(graph_node.get_edges<geometry_msgs::msg::TransformStamped>("r2d2", "paco").empty());

  auto edge_1_ret_1 = graph_node.get_edges<std::string>("r2d2", "paco");
  ASSERT_FALSE(edge_1_ret_1.empty());
  ASSERT_EQ(edge_1_ret_1.size(), 1u);

  auto content_edge_1_ret_1 =
    ros2_knowledge_graph::get_content<std::string>(edge_1_ret_1[0].content);
  ASSERT_EQ(content_edge_1_ret_1.value(), "talks");

  auto edge_2 = ros2_knowledge_graph::new_edge<std::string>("r2d2", "paco", "sees");
  graph_node.update_edge(edge_2);

  ASSERT_EQ(graph_node.get_num_edges(), 2);
  auto edge_2_ret_1 = graph_node.get_edges<std::string>("r2d2", "paco");
  ASSERT_EQ(edge_2_ret_1.size(), 2u);


  auto content_edge_1_ret_2 =
    ros2_knowledge_graph::get_content<std::string>(edge_2_ret_1[0].content);
  ASSERT_EQ(content_edge_1_ret_2.value(), "talks");
  auto content_edge_2_ret_2 =
    ros2_knowledge_graph::get_content<std::string>(edge_2_ret_1[1].content);
  ASSERT_EQ(content_edge_2_ret_2.value(), "sees");

  ASSERT_EQ(graph_node.get_num_edges(), 2);

  auto edge_int_1 = ros2_knowledge_graph::new_edge("r2d2", "paco", 1);
  graph_node.update_edge(edge_int_1);
  auto edge_int_2 = ros2_knowledge_graph::new_edge("r2d2", "paco", 2);
  graph_node.update_edge(edge_int_2);

  ASSERT_EQ(graph_node.get_num_edges(), 3);
  auto edges_ret_1 = graph_node.get_edges<int>("r2d2", "paco");
  ASSERT_EQ(edges_ret_1.size(), 1u);
  auto edges_ret_1_content = edges_ret_1.front();
  auto edges_ret_1_value = ros2_knowledge_graph::get_content<int>(edges_ret_1_content.content);
  ASSERT_TRUE(edges_ret_1_value.has_value());

  auto edge_int_1_ret =
    ros2_knowledge_graph::get_content<int>(
    graph_node.get_edges<int>("r2d2", "paco").front().content);

  ASSERT_EQ(edge_int_1_ret.value(), 2);

  geometry_msgs::msg::TransformStamped tf1;
  tf1.transform.translation.x = 7.0;
  auto edge_tf_1 = ros2_knowledge_graph::new_edge("r2d2", "paco", tf1);
  graph_node.update_edge(edge_tf_1);
  auto edge_tf_2 = ros2_knowledge_graph::new_edge("r2d2", "paco", tf1);
  graph_node.update_edge(edge_tf_2);

  ASSERT_EQ(graph_node.get_num_edges(), 4);

  graph_node.remove_edge(edge_tf_1);

  ASSERT_EQ(graph_node.get_num_edges(), 3);
  auto edges_ret_tf = graph_node.get_edges<geometry_msgs::msg::TransformStamped>("r2d2", "paco");
  ASSERT_TRUE(edges_ret_tf.empty());

  graph_node.remove_edge(edge_2);

  ASSERT_EQ(graph_node.get_num_edges(), 2);
  auto edges_ret_st = graph_node.get_edges<std::string>("r2d2", "paco");
  ASSERT_EQ(edges_ret_st.size(), 1u);
  ASSERT_EQ(
    ros2_knowledge_graph::get_content<std::string>(edges_ret_st[0].content).value(), "talks");

  graph_node.remove_node("r2d2");
  ASSERT_EQ(graph_node.get_num_edges(), 0);
}

TEST(ros2_knowledge_graphnode, graph_comms)
{
  auto node1 = rclcpp::Node::make_shared("test_node_1");
  auto node2 = rclcpp::Node::make_shared("test_node_2");

  GraphTest graph_1(node1);
  GraphTest graph_2(node1);
  GraphTest graph_3(node2);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node1);
  exe.add_node(node2);

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  auto node_1 = ros2_knowledge_graph::new_node("r2d2", "robot");
  graph_1.update_node(node_1);

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  ASSERT_TRUE(graph_1.exist_node("r2d2"));
  ASSERT_TRUE(graph_2.exist_node("r2d2"));
  ASSERT_TRUE(graph_3.exist_node("r2d2"));

  auto node_2 = ros2_knowledge_graph::new_node("paco", "person");
  graph_3.update_node(node_2);

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  ASSERT_TRUE(graph_1.exist_node("paco"));
  ASSERT_TRUE(graph_2.exist_node("paco"));
  ASSERT_TRUE(graph_3.exist_node("paco"));

  ros2_knowledge_graph::add_property(node_2, "age", 42);
  graph_2.update_node(node_2);

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  auto node_2_1 = graph_1.get_node("paco");
  auto node_2_3 = graph_3.get_node("paco");
  ASSERT_TRUE(node_2_1.has_value());
  ASSERT_TRUE(node_2_3.has_value());
  auto p_age_2_1 = ros2_knowledge_graph::get_property<int>(node_2_1.value(), "age");
  auto p_age_2_3 = ros2_knowledge_graph::get_property<int>(node_2_3.value(), "age");

  ASSERT_TRUE(p_age_2_1.has_value());
  ASSERT_TRUE(p_age_2_3.has_value());
  ASSERT_EQ(p_age_2_1.value(), 42);
  ASSERT_EQ(p_age_2_3.value(), 42);

  auto node_table = ros2_knowledge_graph::new_node("table1", "table");
  graph_3.update_node(node_table);
  auto edge_sees = ros2_knowledge_graph::new_edge<std::string>("r2d2", "paco", "sees");
  graph_1.update_edge(edge_sees);

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  ASSERT_EQ(graph_1.get_num_nodes(), 3);
  ASSERT_EQ(graph_2.get_num_nodes(), 3);
  ASSERT_EQ(graph_3.get_num_nodes(), 3);

  auto edges_sees_1 = graph_1.get_edges<std::string>("r2d2", "paco");
  auto edges_sees_2 = graph_2.get_edges<std::string>("r2d2", "paco");
  auto edges_sees_3 = graph_3.get_edges<std::string>("r2d2", "paco");

  ASSERT_EQ(edges_sees_1.size(), 1);
  ASSERT_EQ(edges_sees_2.size(), 1);
  ASSERT_EQ(edges_sees_3.size(), 1);

  graph_1.remove_node("table1");
  GraphTest graph_4(node2);

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  ASSERT_EQ(graph_1.get_num_nodes(), 2);
  ASSERT_EQ(graph_2.get_num_nodes(), 2);
  ASSERT_EQ(graph_3.get_num_nodes(), 2);
  ASSERT_EQ(graph_4.get_num_nodes(), 2);

  ASSERT_EQ(graph_1.get_graph(), graph_2.get_graph());
  ASSERT_EQ(graph_1.get_graph(), graph_3.get_graph());
  ASSERT_EQ(graph_1.get_graph(), graph_4.get_graph());

  auto edge_to_remove = ros2_knowledge_graph::new_edge<std::string>("r2d2", "paco", "sees");
  ASSERT_TRUE(graph_1.remove_edge(edge_to_remove));

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  auto edges_sees_1b = graph_1.get_edges<std::string>("r2d2", "paco");
  auto edges_sees_2b = graph_2.get_edges<std::string>("r2d2", "paco");
  auto edges_sees_3b = graph_3.get_edges<std::string>("r2d2", "paco");

  ASSERT_EQ(edges_sees_1b.size(), 0);
  ASSERT_EQ(edges_sees_2b.size(), 0);
  ASSERT_EQ(edges_sees_3b.size(), 0);

  finish = true;
  t.join();
}

TEST(ros2_knowledge_graphnode, graph_comms_tf)
{
  auto node1 = rclcpp::Node::make_shared("test_node_1");
  auto node2 = rclcpp::Node::make_shared("test_node_2");

  GraphTest graph_1(node1);
  GraphTest graph_2(node1);
  GraphTest graph_3(node2);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node1);
  exe.add_node(node2);

  std::vector<ros2_knowledge_graph_msgs::msg::GraphUpdate> updates;
  auto sub = node2->create_subscription<ros2_knowledge_graph_msgs::msg::GraphUpdate>(
    "graph_update", rclcpp::QoS(100).reliable(),
    [&updates](ros2_knowledge_graph_msgs::msg::GraphUpdate::SharedPtr msg) {
      updates.push_back(*msg);
    });

  std::vector<tf2_msgs::msg::TFMessage> tf_messages;
  auto sub_tf = node2->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(100),
    [&tf_messages](tf2_msgs::msg::TFMessage::SharedPtr msg) {
      tf_messages.push_back(*msg);
    });

  std::vector<tf2_msgs::msg::TFMessage> tfs_messages;
  auto sub_tfs = node2->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(100).transient_local(),
    [&tfs_messages](tf2_msgs::msg::TFMessage::SharedPtr msg) {
      tfs_messages.push_back(*msg);
    });


  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  auto node_1 = ros2_knowledge_graph::new_node("r2d2", "robot");
  auto node_2 = ros2_knowledge_graph::new_node("paco", "person");
  auto node_3 = ros2_knowledge_graph::new_node("table1", "table");
  graph_1.update_node(node_1);
  graph_1.update_node(node_2);
  graph_1.update_node(node_3);

  auto edge_string_1 = ros2_knowledge_graph::new_edge<std::string>("paco", "table1", "sees");
  graph_1.update_edge(edge_string_1);

  geometry_msgs::msg::TransformStamped tf1;
  tf1.transform.translation.x = 0.0;
  auto edge_tf_1 = ros2_knowledge_graph::new_edge("r2d2", "paco", tf1);
  graph_1.update_edge(edge_tf_1);
  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  updates.clear();
  tf_messages.clear();
  tfs_messages.clear();
  {
    rclcpp::Rate rate(20);
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 1.0) {
      edge_tf_1.content.tf_value.transform.translation.x =
        edge_tf_1.content.tf_value.transform.translation.x + 1.0;
      graph_1.update_edge(edge_tf_1);
      rate.sleep();
    }
  }

  auto tf_edge = graph_1.get_edges<geometry_msgs::msg::TransformStamped>("r2d2", "paco");
  ASSERT_EQ(tf_edge.size(), 1u);

  auto content_tf_opt =
    ros2_knowledge_graph::get_content<geometry_msgs::msg::TransformStamped>(tf_edge[0].content);
  ASSERT_TRUE(content_tf_opt.has_value());
  ASSERT_GT(content_tf_opt.value().transform.translation.x, 15.0);

  ASSERT_TRUE(updates.empty());
  ASSERT_TRUE(tfs_messages.empty());
  ASSERT_FALSE(tf_messages.empty());


  geometry_msgs::msg::TransformStamped tf2;
  tf2.transform.translation.x = 0.0;
  auto edge_tf_2 = ros2_knowledge_graph::new_edge("paco", "r2d2", tf2, true);
  graph_1.update_edge(edge_tf_2);
  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  updates.clear();
  tf_messages.clear();
  tfs_messages.clear();
  {
    rclcpp::Rate rate(20);
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 1.0) {
      edge_tf_2.content.tf_value.transform.translation.x =
        edge_tf_2.content.tf_value.transform.translation.x + 1.0;
      graph_1.update_edge(edge_tf_2);
      rate.sleep();
    }
  }

  auto tf_edge2 = graph_1.get_edges<geometry_msgs::msg::TransformStamped>("paco", "r2d2");
  ASSERT_EQ(tf_edge2.size(), 1u);

  auto content_tf_opt2 =
    ros2_knowledge_graph::get_content<geometry_msgs::msg::TransformStamped>(tf_edge2[0].content);
  ASSERT_TRUE(content_tf_opt2.has_value());
  ASSERT_GT(content_tf_opt2.value().transform.translation.x, 15.0);

  ASSERT_TRUE(updates.empty());
  ASSERT_FALSE(tfs_messages.empty());
  ASSERT_TRUE(tf_messages.empty());

  auto tf_edge3 = graph_1.get_edges<geometry_msgs::msg::TransformStamped>("paco", "table1");
  ASSERT_EQ(tf_edge3.size(), 0);

  graph_1.update_tfs();
  graph_2.update_tfs();
  graph_3.update_tfs();

  ASSERT_EQ(graph_1.get_graph(), graph_2.get_graph());
  ASSERT_EQ(graph_1.get_graph(), graph_3.get_graph());

  finish = true;
  t.join();
}

TEST(ros2_knowledge_graphnode, graph_stress)
{
  auto node1 = rclcpp::Node::make_shared("test_node_1");

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node1);

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  const int NUM_GRAPHS = 10;
  std::vector<std::shared_ptr<GraphTest>> graph_vector(NUM_GRAPHS);
  for (int i = 0; i < NUM_GRAPHS; i++) {
    graph_vector[i] = std::make_shared<GraphTest>(node1);
  }

  auto test_node = rclcpp::Node::make_shared("test_node");
  std::default_random_engine generator;
  std::uniform_int_distribution<int> graph_dist(0, NUM_GRAPHS - 1);
  std::uniform_int_distribution<int> node_dist(1, 10);
  std::uniform_int_distribution<uint8_t> op_dist(0, 5);
  std::uniform_int_distribution<uint8_t> type_dist(0, 1);

  rclcpp::Rate rate(200);
  auto start = test_node->now();
  while (rclcpp::ok() && (test_node->now() - start).seconds() < 5) {
    uint8_t type = type_dist(generator);
    uint8_t operation = op_dist(generator);
    int graph_id = graph_dist(generator);
    int node_id = node_dist(generator);

    switch (type) {
      case ros2_knowledge_graph_msgs::msg::GraphUpdate::NODE:
        switch (operation) {
          case ros2_knowledge_graph_msgs::msg::GraphUpdate::UPDATE:
            {
              ros2_knowledge_graph_msgs::msg::Node node;
              node.node_name = "node_" + std::to_string(node_id);
              node.node_class = "test";
              graph_vector[graph_id]->update_node(node);
            }
            break;
          case ros2_knowledge_graph_msgs::msg::GraphUpdate::REMOVE:
            graph_vector[graph_id]->remove_node("node_" + std::to_string(node_id));
            break;
        }
        break;
      case ros2_knowledge_graph_msgs::msg::GraphUpdate::EDGE:
        if (operation == 0) {
          ros2_knowledge_graph_msgs::msg::Edge edge;
          edge.source_node_id = "node_" + std::to_string(node_dist(generator));
          edge.target_node_id = "node_" + std::to_string(node_dist(generator));
          edge.content.type = ros2_knowledge_graph_msgs::msg::Content::BOOL;

          graph_vector[graph_id]->remove_edge(edge);
        } else {
          ros2_knowledge_graph_msgs::msg::Edge edge;
          edge.source_node_id = "node_" + std::to_string(node_dist(generator));
          edge.target_node_id = "node_" + std::to_string(node_dist(generator));
          edge.content.type = ros2_knowledge_graph_msgs::msg::Content::BOOL;

          graph_vector[graph_id]->update_edge(edge);
        }
        break;
    }

    rate.sleep();
  }

  for (int i = 1; i < NUM_GRAPHS; i++) {
    ASSERT_EQ(graph_vector[i]->get_graph(), graph_vector[0]->get_graph());
  }

  finish = true;
  t.join();
}


TEST(ros2_knowledge_graphnode, graph_comms_factory)
{
  auto node1 = rclcpp::Node::make_shared("test_node_1");
  auto node2 = rclcpp::Node::make_shared("test_node_2");

  auto graph_1 = ros2_knowledge_graph::GraphFactory::getInstance(node1);
  auto graph_2 = ros2_knowledge_graph::GraphFactory::getInstance(node1);
  auto graph_3 = ros2_knowledge_graph::GraphFactory::getInstance(node2);

  ASSERT_EQ(graph_1, graph_2);
  ASSERT_EQ(graph_1, graph_3);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node1);
  exe.add_node(node2);

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  auto node_1 = ros2_knowledge_graph::new_node("r2d2", "robot");
  graph_1->update_node(node_1);

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  ASSERT_TRUE(graph_1->exist_node("r2d2"));
  ASSERT_TRUE(graph_2->exist_node("r2d2"));
  ASSERT_TRUE(graph_3->exist_node("r2d2"));

  auto node_2 = ros2_knowledge_graph::new_node("paco", "person");
  graph_3->update_node(node_2);

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  ASSERT_TRUE(graph_1->exist_node("paco"));
  ASSERT_TRUE(graph_2->exist_node("paco"));
  ASSERT_TRUE(graph_3->exist_node("paco"));

  ros2_knowledge_graph::add_property(node_2, "age", 42);
  graph_2->update_node(node_2);

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  auto node_2_1 = graph_1->get_node("paco");
  auto node_2_3 = graph_3->get_node("paco");
  ASSERT_TRUE(node_2_1.has_value());
  ASSERT_TRUE(node_2_3.has_value());
  auto p_age_2_1 = ros2_knowledge_graph::get_property<int>(node_2_1.value(), "age");
  auto p_age_2_3 = ros2_knowledge_graph::get_property<int>(node_2_3.value(), "age");

  ASSERT_TRUE(p_age_2_1.has_value());
  ASSERT_TRUE(p_age_2_3.has_value());
  ASSERT_EQ(p_age_2_1.value(), 42);
  ASSERT_EQ(p_age_2_3.value(), 42);

  auto node_table = ros2_knowledge_graph::new_node("table1", "table");
  graph_3->update_node(node_table);
  auto edge_sees = ros2_knowledge_graph::new_edge<std::string>("r2d2", "paco", "sees");
  graph_1->update_edge(edge_sees);

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  ASSERT_EQ(graph_1->get_num_nodes(), 3);
  ASSERT_EQ(graph_2->get_num_nodes(), 3);
  ASSERT_EQ(graph_3->get_num_nodes(), 3);

  auto edges_sees_1 = graph_1->get_edges<std::string>("r2d2", "paco");
  auto edges_sees_2 = graph_2->get_edges<std::string>("r2d2", "paco");
  auto edges_sees_3 = graph_3->get_edges<std::string>("r2d2", "paco");

  ASSERT_EQ(edges_sees_1.size(), 1);
  ASSERT_EQ(edges_sees_2.size(), 1);
  ASSERT_EQ(edges_sees_3.size(), 1);

  graph_1->remove_node("table1");
  GraphTest graph_4(node2);

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  ASSERT_EQ(graph_1->get_num_nodes(), 2);
  ASSERT_EQ(graph_2->get_num_nodes(), 2);
  ASSERT_EQ(graph_3->get_num_nodes(), 2);
  ASSERT_EQ(graph_4.get_num_nodes(), 2);

  auto edge_to_remove = ros2_knowledge_graph::new_edge<std::string>("r2d2", "paco", "sees");
  ASSERT_TRUE(graph_1->remove_edge(edge_to_remove));

  {
    auto start = node1->now();
    while ((node1->now() - start).seconds() < 0.1) {}
  }

  auto edges_sees_1b = graph_1->get_edges<std::string>("r2d2", "paco");
  auto edges_sees_2b = graph_2->get_edges<std::string>("r2d2", "paco");
  auto edges_sees_3b = graph_3->get_edges<std::string>("r2d2", "paco");

  ASSERT_EQ(edges_sees_1b.size(), 0);
  ASSERT_EQ(edges_sees_2b.size(), 0);
  ASSERT_EQ(edges_sees_3b.size(), 0);

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
