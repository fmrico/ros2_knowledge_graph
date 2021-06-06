#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Intelligent Robotics Core S.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Francisco Martin Rico - fmrico at gmail.com


import rclpy

import tf2_py
import tf2_ros

import rclpy.node
from rclpy.qos import InvalidQoSProfileException
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from ros2_knowledge_graph_msgs.msg import GraphUpdate, Graph, Node, Edge, Content, Property
from builtin_interfaces.msg import Time

class Ros2KnowledgeGraphImpl(rclpy.node.Node):

    def __init__(self):
        super().__init__('rqt_ros2_knowledge_graph')

        self.update_sub = self.create_subscription(
            GraphUpdate,
            '/graph_update',
            self.graph_update_callback,
            qos_profile=QoSProfile(
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=100,
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            )

        self.graph_pub = self.create_publisher(GraphUpdate,
            '/graph_update',
            qos_profile=QoSProfile(
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=100,
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            )

        self.graph = Graph()
        self.graph_id = self.get_name() + str(self.get_clock().now().nanoseconds);

        msg = GraphUpdate()
        msg.operation_type = GraphUpdate.REQSYNC
        msg.element_type = GraphUpdate.GRAPH
        print(self.get_clock().now().seconds_nanoseconds())
        msg.stamp.sec = self.get_clock().now().seconds_nanoseconds()[0]
        msg.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()[1]
        msg.node_id = self.graph_id

        print("Sending graph")
        self.graph_pub.publish(msg)

    def __repr__(self):
        ret = "Nodes: " + str(len(self.graph.nodes)) + '\n'
        for i in self.graph.nodes:
            ret = ret + str(i) + '\n'
        ret = ret + "Edges: " + str(len(self.graph.edges)) + '\n'
        for i in self.graph.edges:
            ret = ret + str(i) + '\n'
        return ret

    def init_graph(self, recv_graph):
        self.graph = recv_graph

    def update_node(self, recv_node):
        found = False
        for node in self.graph.nodes:
          if node.node_name == recv_node.node_name:
            node = recv_node
            found = True
            break
        
        if not found:
          self.graph.nodes.append(recv_node)

    def remove_node(self, node_to_remove):
        self.graph.nodes = [x for x in self.graph.nodes if x.node_name != node_to_remove]

        new_edges = [x for x in self.graph.edges if x.source_node_id != node_to_remove and x.target_node_id != node_to_remove]
        self.graph.edges = new_edges

    def exist_node(self, node_name):
      for node in self.graph.nodes:
          if node.node_name == node_name:
            return True
      return False

    def update_edge(self, recv_edge):
        if not self.exist_node(recv_edge.source_node_id):
          return
        if not self.exist_node(recv_edge.target_node_id):
          return

        found = False
        for edge in self.graph.edges:
          if edge == recv_edge:
            edge = recv_edge
            found = True
            break

        if not found:
          self.graph.edges.append(recv_edge)

    def remove_edge(self, recv_edge):
        self.graph.edges.remove(recv_edge)
        
    def graph_update_callback(self, msg):
        self.get_logger().info('I heard: a new graph or update')


        # UPDATE NODE
        if msg.operation_type == 0 and msg.element_type == 0:
            self.update_node(msg.node)

        # UPDATE EDGE
        if msg.operation_type == 0 and msg.element_type == 1:
            self.update_edge(msg.edge)

        # REMOVE NODE
        if msg.operation_type == 1 and msg.element_type == 0:
            self.remove_node(msg.removed_node)

        # REMOVE EDGE
        if msg.operation_type == 1 and msg.element_type == 1:
            self.remove_edge(msg.edge)

        # SYNC
        if msg.operation_type == 2 and msg.target_node == self.graph_id:
            self.init_graph(msg.graph)
