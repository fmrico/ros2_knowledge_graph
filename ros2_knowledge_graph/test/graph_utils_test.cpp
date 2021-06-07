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

#include "ros2_knowledge_graph_msgs/msg/graph.hpp"
#include "ros2_knowledge_graph_msgs/msg/node.hpp"
#include "ros2_knowledge_graph_msgs/msg/edge.hpp"

#include "ros2_knowledge_graph/graph_utils.hpp"

#include "rclcpp/rclcpp.hpp"

#include "gtest/gtest.h"

TEST(graph_utils, content_ops)
{
  auto content_bool = ros2_knowledge_graph::new_content<bool>(true);
  ASSERT_TRUE(content_bool.bool_value);
  ASSERT_EQ(content_bool.type, ros2_knowledge_graph_msgs::msg::Content::BOOL);
  auto content_bool2 = ros2_knowledge_graph::new_content(true);
  ASSERT_TRUE(content_bool2.bool_value);
  ASSERT_EQ(content_bool2.type, ros2_knowledge_graph_msgs::msg::Content::BOOL);
  auto content_int = ros2_knowledge_graph::new_content(1);
  ASSERT_EQ(content_int.int_value, 1);
  ASSERT_EQ(content_int.type, ros2_knowledge_graph_msgs::msg::Content::INT);
  auto content_float = ros2_knowledge_graph::new_content(1.0f);
  ASSERT_EQ(content_float.float_value, 1.0f);
  ASSERT_EQ(content_float.type, ros2_knowledge_graph_msgs::msg::Content::FLOAT);
  auto content_double = ros2_knowledge_graph::new_content(1.0);
  ASSERT_EQ(content_double.double_value, 1.0);
  ASSERT_EQ(content_double.type, ros2_knowledge_graph_msgs::msg::Content::DOUBLE);
  auto content_string = ros2_knowledge_graph::new_content<std::string>("1.0");
  ASSERT_EQ(content_string.string_value, "1.0");
  ASSERT_EQ(content_string.type, ros2_knowledge_graph_msgs::msg::Content::STRING);

  auto content_vbool = ros2_knowledge_graph::new_content(std::vector<bool>{true});
  ASSERT_FALSE(content_vbool.bool_vector.empty());
  ASSERT_TRUE(content_vbool.bool_vector[0]);
  ASSERT_EQ(content_vbool.type, ros2_knowledge_graph_msgs::msg::Content::VBOOL);

  auto content_vint = ros2_knowledge_graph::new_content(std::vector<int>{1, 2});
  ASSERT_FALSE(content_vint.int_vector.empty());
  ASSERT_EQ(content_vint.int_vector[0], 1);
  ASSERT_EQ(content_vint.int_vector[1], 2);
  ASSERT_EQ(content_vint.type, ros2_knowledge_graph_msgs::msg::Content::VINT);

  auto content_vfloat = ros2_knowledge_graph::new_content(std::vector<float>{1.0, 2.0});
  ASSERT_FALSE(content_vfloat.float_vector.empty());
  ASSERT_EQ(content_vfloat.float_vector[0], 1);
  ASSERT_EQ(content_vfloat.float_vector[1], 2);
  ASSERT_EQ(content_vfloat.type, ros2_knowledge_graph_msgs::msg::Content::VFLOAT);

  auto content_vdouble = ros2_knowledge_graph::new_content(std::vector<double>{1.0, 2.0});
  ASSERT_FALSE(content_vdouble.double_vector.empty());
  ASSERT_EQ(content_vdouble.double_vector[0], 1);
  ASSERT_EQ(content_vdouble.double_vector[1], 2);
  ASSERT_EQ(content_vdouble.type, ros2_knowledge_graph_msgs::msg::Content::VDOUBLE);

  auto content_vstring = ros2_knowledge_graph::new_content(std::vector<std::string>{"1.0", "2.0"});
  ASSERT_FALSE(content_vstring.string_vector.empty());
  ASSERT_EQ(content_vstring.string_vector[0], "1.0");
  ASSERT_EQ(content_vstring.string_vector[1], "2.0");
  ASSERT_EQ(content_vstring.type, ros2_knowledge_graph_msgs::msg::Content::VSTRING);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 7.0;
  auto content_pose = ros2_knowledge_graph::new_content(pose);
  ASSERT_EQ(content_pose.pose_value.pose.position.x, 7.0);
  ASSERT_EQ(content_pose.type, ros2_knowledge_graph_msgs::msg::Content::POSE);

  geometry_msgs::msg::TransformStamped tf1;
  tf1.transform.translation.x = 7.0;
  auto content_tf = ros2_knowledge_graph::new_content(tf1);
  ASSERT_EQ(content_tf.tf_value.transform.translation.x, 7.0);
  ASSERT_EQ(content_tf.type, ros2_knowledge_graph_msgs::msg::Content::TF);

  auto content_tf2 = ros2_knowledge_graph::new_content(tf1, true);
  ASSERT_EQ(content_tf2.tf_value.transform.translation.x, 7.0);
  ASSERT_EQ(content_tf2.type, ros2_knowledge_graph_msgs::msg::Content::STATICTF);

  auto content_vpose =
    ros2_knowledge_graph::new_content(std::vector<geometry_msgs::msg::PoseStamped>{pose});
  ASSERT_FALSE(content_vpose.pose_vector.empty());
  ASSERT_EQ(content_vpose.pose_vector[0].pose.position.x, 7.0);
  ASSERT_EQ(content_vpose.type, ros2_knowledge_graph_msgs::msg::Content::VPOSE);

  auto content_vtf =
    ros2_knowledge_graph::new_content(std::vector<geometry_msgs::msg::TransformStamped>{tf1});
  ASSERT_FALSE(content_vtf.tf_vector.empty());
  ASSERT_EQ(content_vtf.tf_vector[0].transform.translation.x, 7.0);
  ASSERT_EQ(content_vtf.type, ros2_knowledge_graph_msgs::msg::Content::VTF);

  auto content_error = ros2_knowledge_graph::new_content(geometry_msgs::msg::Quaternion());
  ASSERT_EQ(content_error.type, ros2_knowledge_graph_msgs::msg::Content::ERROR);

  auto content_bool_error = ros2_knowledge_graph::get_content<bool>(content_float);
  ASSERT_FALSE(content_bool_error.has_value());

  auto content_bool_error2 =
    ros2_knowledge_graph::get_content<geometry_msgs::msg::Quaternion>(content_float);
  ASSERT_FALSE(content_bool_error2.has_value());

  auto content_bool_ret = ros2_knowledge_graph::get_content<bool>(content_bool);
  ASSERT_TRUE(content_bool_ret.has_value());
  ASSERT_TRUE(content_bool_ret.value());

  auto content_int_ret = ros2_knowledge_graph::get_content<int>(content_int);
  ASSERT_TRUE(content_int_ret.has_value());
  ASSERT_EQ(content_int_ret.value(), 1);

  auto content_float_ret = ros2_knowledge_graph::get_content<float>(content_float);
  ASSERT_TRUE(content_float_ret.has_value());
  ASSERT_EQ(content_float_ret.value(), 1.0);

  auto content_double_ret = ros2_knowledge_graph::get_content<double>(content_double);
  ASSERT_TRUE(content_double_ret.has_value());
  ASSERT_EQ(content_double_ret.value(), 1.0);

  auto content_string_ret = ros2_knowledge_graph::get_content<std::string>(content_string);
  ASSERT_TRUE(content_string_ret.has_value());
  ASSERT_EQ(content_string_ret.value(), "1.0");

  auto content_vbool_ret = ros2_knowledge_graph::get_content<std::vector<bool>>(content_vbool);
  ASSERT_TRUE(content_vbool_ret.has_value());
  ASSERT_EQ(content_vbool_ret.value().size(), 1);
  ASSERT_TRUE(content_vbool_ret.value()[0]);

  auto content_vint_ret = ros2_knowledge_graph::get_content<std::vector<int>>(content_vint);
  ASSERT_TRUE(content_vint_ret.has_value());
  ASSERT_EQ(content_vint_ret.value().size(), 2);
  ASSERT_EQ(content_vint_ret.value()[0], 1);
  ASSERT_EQ(content_vint_ret.value()[1], 2);

  auto content_vfloat_ret = ros2_knowledge_graph::get_content<std::vector<float>>(content_vfloat);
  ASSERT_TRUE(content_vfloat_ret.has_value());
  ASSERT_EQ(content_vfloat_ret.value().size(), 2);
  ASSERT_EQ(content_vfloat_ret.value()[0], 1.0);
  ASSERT_EQ(content_vfloat_ret.value()[1], 2.0);

  auto content_vdouble_ret =
    ros2_knowledge_graph::get_content<std::vector<double>>(content_vdouble);
  ASSERT_TRUE(content_vdouble_ret.has_value());
  ASSERT_EQ(content_vdouble_ret.value().size(), 2);
  ASSERT_EQ(content_vdouble_ret.value()[0], 1.0);
  ASSERT_EQ(content_vdouble_ret.value()[1], 2.0);

  auto content_vstring_ret =
    ros2_knowledge_graph::get_content<std::vector<std::string>>(content_vstring);
  ASSERT_TRUE(content_vstring_ret.has_value());
  ASSERT_EQ(content_vstring_ret.value().size(), 2);
  ASSERT_EQ(content_vstring_ret.value()[0], "1.0");
  ASSERT_EQ(content_vstring_ret.value()[1], "2.0");

  auto content_pose_ret =
    ros2_knowledge_graph::get_content<geometry_msgs::msg::PoseStamped>(content_pose);
  ASSERT_TRUE(content_pose_ret.has_value());
  ASSERT_EQ(content_pose_ret.value().pose.position.x, 7.0);

  auto content_tf_ret =
    ros2_knowledge_graph::get_content<geometry_msgs::msg::TransformStamped>(content_tf);
  ASSERT_TRUE(content_tf_ret.has_value());
  ASSERT_EQ(content_tf_ret.value().transform.translation.x, 7.0);

  auto content_tf2_ret =
    ros2_knowledge_graph::get_content<geometry_msgs::msg::TransformStamped>(content_tf2);
  ASSERT_TRUE(content_tf2_ret.has_value());
  ASSERT_EQ(content_tf2_ret.value().transform.translation.x, 7.0);

  auto content_vpose_ret =
    ros2_knowledge_graph::get_content<std::vector<geometry_msgs::msg::PoseStamped>>(content_vpose);
  ASSERT_TRUE(content_vpose_ret.has_value());
  ASSERT_EQ(content_vpose_ret.value().size(), 1);
  ASSERT_EQ(content_vpose_ret.value()[0].pose.position.x, 7.0);

  auto content_vtf_ret =
    ros2_knowledge_graph::get_content<std::vector<geometry_msgs::msg::TransformStamped>>(
    content_vtf);
  ASSERT_TRUE(content_vtf_ret.has_value());
  ASSERT_EQ(content_vtf_ret.value().size(), 1);
  ASSERT_EQ(content_vtf_ret.value()[0].transform.translation.x, 7.0);
}

TEST(graph_utils, node_ops)
{
  auto node_1 = ros2_knowledge_graph::new_node("r2d2", "robot");
  ASSERT_EQ(node_1.node_name, "r2d2");
  ASSERT_EQ(node_1.node_class, "robot");
  ASSERT_TRUE(node_1.properties.empty());

  ASSERT_TRUE(ros2_knowledge_graph::add_property(node_1, "bool_key", true));
  ASSERT_TRUE(ros2_knowledge_graph::add_property(node_1, "int_key", 1));
  ASSERT_TRUE(ros2_knowledge_graph::add_property(node_1, "int_key2", 2));
  ASSERT_TRUE(ros2_knowledge_graph::add_property(node_1, "float_key", 1.0f));
  ASSERT_TRUE(ros2_knowledge_graph::add_property(node_1, "double_key", 1.0));
  ASSERT_TRUE(ros2_knowledge_graph::add_property<std::string>(node_1, "string_key", "1.0"));

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 7.0;
  ASSERT_TRUE(ros2_knowledge_graph::add_property(node_1, "pose_key", pose));

  geometry_msgs::msg::TransformStamped tf1;
  tf1.transform.translation.x = 7.0;
  ASSERT_TRUE(ros2_knowledge_graph::add_property(node_1, "tf_key", tf1));

  ASSERT_TRUE(ros2_knowledge_graph::add_property(node_1, "vbool_key", std::vector<bool>{true}));
  ASSERT_TRUE(ros2_knowledge_graph::add_property(node_1, "vint_key", std::vector<int>{1, 2}));
  ASSERT_TRUE(
    ros2_knowledge_graph::add_property(node_1, "vfloat_key", std::vector<float>{1.0f, 2.0f}));
  ASSERT_TRUE(
    ros2_knowledge_graph::add_property(node_1, "vdouble_key", std::vector<double>{1.0, 2.0}));
  ASSERT_TRUE(
    ros2_knowledge_graph::add_property(
      node_1, "vstring_key", std::vector<std::string>{"1.0", "2.0"}));
  ASSERT_TRUE(
    ros2_knowledge_graph::add_property(
      node_1, "vpose_key", std::vector<geometry_msgs::msg::PoseStamped>{pose}));
  ASSERT_TRUE(
    ros2_knowledge_graph::add_property(
      node_1, "vtf_key", std::vector<geometry_msgs::msg::TransformStamped>{tf1}));

  ASSERT_EQ(ros2_knowledge_graph::get_properties(node_1).size(), 15u);

  auto no_ret = ros2_knowledge_graph::get_property<bool>(node_1, "no_exist_key");
  ASSERT_FALSE(no_ret.has_value());

  auto erro_bool_ret = ros2_knowledge_graph::get_property<int>(node_1, "bool_key");
  ASSERT_FALSE(erro_bool_ret.has_value());

  auto bool_type = ros2_knowledge_graph::get_property_type(node_1, "bool_key");
  ASSERT_EQ(bool_type, ros2_knowledge_graph_msgs::msg::Content::BOOL);
  auto bool_ret = ros2_knowledge_graph::get_property<bool>(node_1, "bool_key");
  ASSERT_TRUE(bool_ret.has_value());
  ASSERT_TRUE(bool_ret.value());

  auto int_type = ros2_knowledge_graph::get_property_type(node_1, "int_key");
  ASSERT_EQ(int_type, ros2_knowledge_graph_msgs::msg::Content::INT);
  auto int_ret = ros2_knowledge_graph::get_property<int>(node_1, "int_key");
  ASSERT_TRUE(int_ret.has_value());
  ASSERT_EQ(int_ret.value(), 1);

  auto vint_type = ros2_knowledge_graph::get_property_type(node_1, "vint_key");
  ASSERT_EQ(vint_type, ros2_knowledge_graph_msgs::msg::Content::VINT);
  auto vint_ret = ros2_knowledge_graph::get_property<std::vector<int>>(node_1, "vint_key");
  ASSERT_TRUE(vint_ret.has_value());
  ASSERT_EQ(vint_ret.value().size(), 2);
  ASSERT_EQ(vint_ret.value()[0], 1);
  ASSERT_EQ(vint_ret.value()[1], 2);
}

TEST(graph_utils, edge_ops)
{
  auto edge_1 = ros2_knowledge_graph::new_edge<bool>("r2d2", "human", true);


  auto bool_value = ros2_knowledge_graph::get_content<bool>(edge_1.content);
  ASSERT_TRUE(bool_value.has_value());
  ASSERT_TRUE(bool_value.value());
  ASSERT_EQ(edge_1.content.type, ros2_knowledge_graph_msgs::msg::Content::BOOL);

  auto edge_2 = ros2_knowledge_graph::new_edge<int>("r2d2", "human", 1);

  auto int_value = ros2_knowledge_graph::get_content<int>(edge_2.content);
  ASSERT_EQ(edge_2.content.type, ros2_knowledge_graph_msgs::msg::Content::INT);
  ASSERT_TRUE(int_value.has_value());
  ASSERT_EQ(int_value.value(), 1);
  auto error_value = ros2_knowledge_graph::get_content<double>(edge_1.content);
  ASSERT_FALSE(error_value.has_value());


  auto edge_string = ros2_knowledge_graph::new_edge<std::string>("r2d2", "human", "talks");
  ASSERT_EQ(edge_string.content.type, ros2_knowledge_graph_msgs::msg::Content::STRING);
  auto string_value = ros2_knowledge_graph::get_content<std::string>(edge_string.content);
  ASSERT_TRUE(string_value.has_value());
  ASSERT_EQ(string_value.value(), "talks");


  geometry_msgs::msg::TransformStamped tf1;
  tf1.transform.translation.x = 7.0;
  auto edge_tf_1 = ros2_knowledge_graph::new_edge("r2d2", "human", tf1);
  ASSERT_EQ(edge_tf_1.content.type, ros2_knowledge_graph_msgs::msg::Content::TF);
  auto tf_value =
    ros2_knowledge_graph::get_content<geometry_msgs::msg::TransformStamped>(edge_tf_1.content);
  ASSERT_TRUE(tf_value.has_value());
  ASSERT_EQ(tf_value.value(), tf1);

  auto edge_tf_2 = ros2_knowledge_graph::new_edge("r2d2", "human", tf1, true);
  ASSERT_EQ(edge_tf_2.content.type, ros2_knowledge_graph_msgs::msg::Content::STATICTF);
  auto tf_value2 =
    ros2_knowledge_graph::get_content<geometry_msgs::msg::TransformStamped>(edge_tf_2.content);
  ASSERT_TRUE(tf_value2.has_value());
  ASSERT_EQ(tf_value2.value(), tf1);
}

TEST(graph_utils, string)
{
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::BOOL), "bool");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::INT), "int");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::FLOAT), "float");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::DOUBLE), "double");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::STRING), "string");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::VBOOL), "bool[]");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::VINT), "int[]");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::VFLOAT), "float[]");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::VDOUBLE), "double[]");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::VSTRING), "string[]");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::POSE), "pose");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::TF), "tf");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::STATICTF),
    "static tf");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::VPOSE), "pose[]");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::VTF), "tf[]");
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(ros2_knowledge_graph_msgs::msg::Content::ERROR), "error");

  ros2_knowledge_graph_msgs::msg::Content bool_content;
  bool_content.type = ros2_knowledge_graph_msgs::msg::Content::BOOL;
  bool_content.bool_value = true;
  ASSERT_EQ(ros2_knowledge_graph::to_string(bool_content), "true");

  ros2_knowledge_graph_msgs::msg::Content int_content;
  int_content.type = ros2_knowledge_graph_msgs::msg::Content::INT;
  int_content.int_value = 1;
  ASSERT_EQ(ros2_knowledge_graph::to_string(int_content), "1");

  ros2_knowledge_graph_msgs::msg::Content float_content;
  float_content.type = ros2_knowledge_graph_msgs::msg::Content::FLOAT;
  float_content.float_value = 1.0;
  ASSERT_EQ(ros2_knowledge_graph::to_string(float_content), "1.000000");

  ros2_knowledge_graph_msgs::msg::Content double_content;
  double_content.type = ros2_knowledge_graph_msgs::msg::Content::DOUBLE;
  double_content.double_value = 1.0;
  ASSERT_EQ(ros2_knowledge_graph::to_string(double_content), "1.000000");

  ros2_knowledge_graph_msgs::msg::Content string_content;
  string_content.type = ros2_knowledge_graph_msgs::msg::Content::STRING;
  string_content.string_value = "Hi";
  ASSERT_EQ(ros2_knowledge_graph::to_string(string_content), "Hi");

  ros2_knowledge_graph_msgs::msg::Content vbool_content;
  vbool_content.type = ros2_knowledge_graph_msgs::msg::Content::VBOOL;
  vbool_content.bool_vector = {true, false};
  ASSERT_EQ(ros2_knowledge_graph::to_string(vbool_content), "[ true false]");

  ros2_knowledge_graph_msgs::msg::Content vint_content;
  vint_content.type = ros2_knowledge_graph_msgs::msg::Content::VINT;
  vint_content.int_vector = {1, 2};
  ASSERT_EQ(ros2_knowledge_graph::to_string(vint_content), "[ 1 2]");

  ros2_knowledge_graph_msgs::msg::Content vfloat_content;
  vfloat_content.type = ros2_knowledge_graph_msgs::msg::Content::VFLOAT;
  vfloat_content.float_vector = {1.0, 2.0};
  ASSERT_EQ(ros2_knowledge_graph::to_string(vfloat_content), "[ 1.000000 2.000000]");

  ros2_knowledge_graph_msgs::msg::Content vdouble_content;
  vdouble_content.type = ros2_knowledge_graph_msgs::msg::Content::VDOUBLE;
  vdouble_content.double_vector = {1.0, 2.0};
  ASSERT_EQ(ros2_knowledge_graph::to_string(vdouble_content), "[ 1.000000 2.000000]");

  ros2_knowledge_graph_msgs::msg::Content vstring_content;
  vstring_content.type = ros2_knowledge_graph_msgs::msg::Content::VSTRING;
  vstring_content.string_vector = {"Hi", "World"};
  ASSERT_EQ(ros2_knowledge_graph::to_string(vstring_content), "[ Hi World]");

  ros2_knowledge_graph_msgs::msg::Content pose_content;
  pose_content.type = ros2_knowledge_graph_msgs::msg::Content::POSE;
  pose_content.pose_value.pose.position.x = 1.0;
  pose_content.pose_value.pose.position.y = 2.0;
  pose_content.pose_value.pose.position.z = 3.0;
  ASSERT_EQ(ros2_knowledge_graph::to_string(pose_content), "(1.000000 2.000000 3.000000)");

  ros2_knowledge_graph_msgs::msg::Content tf_content;
  tf_content.type = ros2_knowledge_graph_msgs::msg::Content::TF;
  tf_content.tf_value.header.frame_id = "dad";
  tf_content.tf_value.child_frame_id = "son";
  tf_content.tf_value.transform.translation.x = 1.0;
  tf_content.tf_value.transform.translation.y = 2.0;
  tf_content.tf_value.transform.translation.z = 3.0;
  tf_content.tf_value.transform.rotation.x = 0.0;
  tf_content.tf_value.transform.rotation.y = 0.0;
  tf_content.tf_value.transform.rotation.z = 0.0;
  tf_content.tf_value.transform.rotation.w = 1.0;
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(tf_content),
    "[dad -> son] (1.000000 2.000000 3.000000)");

  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("bool"),
    ros2_knowledge_graph_msgs::msg::Content::BOOL);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("int"),
    ros2_knowledge_graph_msgs::msg::Content::INT);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("float"),
    ros2_knowledge_graph_msgs::msg::Content::FLOAT);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("double"),
    ros2_knowledge_graph_msgs::msg::Content::DOUBLE);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("string"),
    ros2_knowledge_graph_msgs::msg::Content::STRING);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("bool[]"),
    ros2_knowledge_graph_msgs::msg::Content::VBOOL);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("int[]"),
    ros2_knowledge_graph_msgs::msg::Content::VINT);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("float[]"),
    ros2_knowledge_graph_msgs::msg::Content::VFLOAT);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("double[]"),
    ros2_knowledge_graph_msgs::msg::Content::VDOUBLE);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("string[]"),
    ros2_knowledge_graph_msgs::msg::Content::VSTRING);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("pose"),
    ros2_knowledge_graph_msgs::msg::Content::POSE);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("tf"),
    ros2_knowledge_graph_msgs::msg::Content::TF);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("static tf"),
    ros2_knowledge_graph_msgs::msg::Content::STATICTF);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("pose[]"),
    ros2_knowledge_graph_msgs::msg::Content::VPOSE);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("tf[]"),
    ros2_knowledge_graph_msgs::msg::Content::VTF);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("error"),
    ros2_knowledge_graph_msgs::msg::Content::ERROR);
  ASSERT_EQ(
    ros2_knowledge_graph::type_from_string("notype"),
    ros2_knowledge_graph_msgs::msg::Content::ERROR);

  ros2_knowledge_graph_msgs::msg::Node node;
  node.node_name = "r2d2";
  node.node_class = "robot";
  ros2_knowledge_graph_msgs::msg::Property prop;
  prop.key = "owner";
  prop.value.type = ros2_knowledge_graph_msgs::msg::Content::STRING;
  prop.value.string_value = "Anakin Skywalker";
  node.properties.push_back(prop);

  ASSERT_EQ(
    ros2_knowledge_graph::to_string(node), "r2d2 (robot)\nowner: [Anakin Skywalker]");

  ros2_knowledge_graph_msgs::msg::Edge edge;
  edge.source_node_id = "r2d2";
  edge.target_node_id = "paco";
  edge.content.type = ros2_knowledge_graph_msgs::msg::Content::INT;
  edge.content.int_value = 42;
  ASSERT_EQ(
    ros2_knowledge_graph::to_string(edge), "r2d2 -> paco [int]{42}");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
