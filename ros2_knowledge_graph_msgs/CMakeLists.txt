cmake_minimum_required(VERSION 3.5)
project(ros2_knowledge_graph_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Content.msg"
  "msg/Property.msg"
  "msg/Node.msg"
  "msg/Edge.msg"
  "msg/Graph.msg"
  "msg/GraphUpdate.msg"
 DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
