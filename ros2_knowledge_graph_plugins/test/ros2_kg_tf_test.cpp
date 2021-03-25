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
#include "ros2_kg_tf_plugin/TFLayer.hpp"
#include "gtest/gtest.h"

TEST(ros2_knowledge_graphnode, tf_edges)
{
  tf2::BufferCore tfBuffer;
  tf2_ros::TransformListener tf_listener(tfBuffer);

  auto test_node = rclcpp::Node::make_shared("test_node");
  ros2_knowledge_graph::GraphNode graph("graph");

  graph.start();

  graph.add_node(ros2_knowledge_graph::Node{"paco", "person"});
  graph.add_node(ros2_knowledge_graph::Node{"r2d2", "robot"});
  graph.add_node(ros2_knowledge_graph::Node{"room1", "room"});

  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"1:0:0:0:0:0", "tf", "room1", "paco"}));
  ASSERT_TRUE(graph.add_edge(ros2_knowledge_graph::Edge{"2:0:0:0:0:0", "tf_static", "room1", "r2d2"}));

  {
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 2.0);
  }

  std::vector<ros2_knowledge_graph::Edge> tf_edges;
  graph.get_edges("room1", "paco", "tf", tf_edges);
  ASSERT_FALSE(tf_edges.empty());
  tf_edges.clear();
  graph.get_edges("room1", "paco", "tf_static", tf_edges);
  ASSERT_FALSE(tf_edges.empty());

  graph.get_edges("room1", "r2d2", "tf_static", tf_edges);
  ASSERT_FALSE(tf_edges.empty());
  tf_edges.clear();
  graph.get_edges("room1", "paco", "tf", tf_edges);
  ASSERT_FALSE(tf_edges.empty());

  tf_edges.clear();
  {
    auto tf = tfBuffer.lookupTransform("room1", "paco", tf2::TimePointZero);
    ASSERT_EQ(tf.transform.translation.x, 1.0);
    ASSERT_EQ(tf.transform.translation.y, 0.0);
    ASSERT_EQ(tf.transform.translation.z, 0.0);
    ASSERT_EQ(tf.transform.rotation.x, 0.0);
    ASSERT_EQ(tf.transform.rotation.y, 0.0);
    ASSERT_EQ(tf.transform.rotation.z, 0.0);
    ASSERT_EQ(tf.transform.rotation.w, 1.0);
  }

  tf_edges.clear();
  {
    auto tf = tfBuffer.lookupTransform("paco", "room1", tf2::TimePointZero);
    ASSERT_EQ(tf.transform.translation.x, -1.0);
    ASSERT_EQ(tf.transform.translation.y, 0.0);
    ASSERT_EQ(tf.transform.translation.z, 0.0);
    ASSERT_EQ(tf.transform.rotation.x, 0.0);
    ASSERT_EQ(tf.transform.rotation.y, 0.0);
    ASSERT_EQ(tf.transform.rotation.z, 0.0);
    ASSERT_EQ(tf.transform.rotation.w, 1.0);
  }

  tf_edges.clear();
  {
    auto tf = tfBuffer.lookupTransform("paco", "r2d2", tf2::TimePointZero);
    ASSERT_EQ(tf.transform.translation.x, 1.0);
    ASSERT_EQ(tf.transform.translation.y, 0.0);
    ASSERT_EQ(tf.transform.translation.z, 0.0);
    ASSERT_EQ(tf.transform.rotation.x, 0.0);
    ASSERT_EQ(tf.transform.rotation.y, 0.0);
    ASSERT_EQ(tf.transform.rotation.z, 0.0);
    ASSERT_EQ(tf.transform.rotation.w, 1.0);
  }
}  
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
