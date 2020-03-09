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

#ifndef ROS2_KG_TF_PLUGIN__TFLAYER_HPP_
#define ROS2_KG_TF_PLUGIN__TFLAYER_HPP_

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>

#include "ros2_knowledge_graph/Layer.hpp"

namespace ros2_kg_tf_plugin
{

class TFLayer : public ros2_knowledge_graph::Layer
{
public:
  TFLayer();

  void configure(rclcpp::Node::SharedPtr & node);

  void on_add_edge(ros2_knowledge_graph::Edge & edge);
  void on_get_edge(ros2_knowledge_graph::Edge & edge);

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2::BufferCore> tfBuffer_;
};

}  // namespace ros2_kg_tf_plugin

#endif  // ROS2_KG_TF_PLUGIN__TFLAYER_HPP_
