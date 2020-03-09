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

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


#include <memory>
#include <string>
#include <vector>

#include "ros2_knowledge_graph/Layer.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "ros2_kg_tf_plugin/TFLayer.hpp"

namespace ros2_kg_tf_plugin
{

TFLayer::TFLayer()
{
  static_tf_broadcaster_ = nullptr;
  tfBuffer_ = nullptr;
  tf_listener_ = nullptr;
  tf_broadcaster_ = nullptr;
}

void
TFLayer::configure(rclcpp::Node::SharedPtr & node)
{
  Layer::configure(node);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  tfBuffer_ = std::make_shared<tf2::BufferCore>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, node_, false);
}

void
TFLayer::on_add_edge(ros2_knowledge_graph::Edge & edge)
{
  if (edge.type == "tf" || edge.type == "tf_static") {
    std::vector<std::string> values = ros2_knowledge_graph::tokenize(edge.content, ":");

    if (values.size() == 6) {
      double x = std::stod(values[0]);
      double y = std::stod(values[1]);
      double z = std::stod(values[2]);
      double r = std::stod(values[3]);
      double p = std::stod(values[4]);
      double yw = std::stod(values[5]);

      geometry_msgs::msg::TransformStamped msg;
      msg.header.frame_id = edge.source;
      msg.header.stamp = node_->now();
      msg.child_frame_id = edge.target;
      msg.transform.translation.x = x;
      msg.transform.translation.y = y;
      msg.transform.translation.z = z;
      tf2::Quaternion q;
      q.setEuler(yw, p, r);
      msg.transform.rotation.x = q.x();
      msg.transform.rotation.y = q.y();
      msg.transform.rotation.z = q.z();
      msg.transform.rotation.w = q.w();

      if (edge.type == "tf") {
        tf_broadcaster_->sendTransform(msg);
      } else {
        static_tf_broadcaster_->sendTransform(msg);
      }
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "TFLayer: Error parsing TF [%s]. Use x:y:z:roll:pitch:yaw",
        edge.content.c_str());
    }

    edge.content = "";
  }
}

void
TFLayer::on_get_edge(ros2_knowledge_graph::Edge & edge)
{
  if (edge.type == "tf" || edge.type == "tf_static") {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tfBuffer_->lookupTransform(edge.source, edge.target, tf2::TimePointZero);

      auto & r = tf.transform.rotation;
      auto & t = tf.transform.translation;

      tf2::Quaternion q(r.x, r.y, r.z, r.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      edge.content = std::to_string(t.x) + ":" + std::to_string(t.y) + ":" +
        std::to_string(t.z) + ":" + std::to_string(roll) + ":" +
        std::to_string(pitch) + ":" + std::to_string(yaw);
    } catch (std::exception & e) {
      RCLCPP_WARN(node_->get_logger(),
        "TFLayer  [%s, %s]not connected by TFs",
        edge.source.c_str(), edge.target.c_str());
    }
  }
}

}  // namespace ros2_kg_tf_plugin

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  ros2_kg_tf_plugin::TFLayer,
  ros2_knowledge_graph::Layer)
