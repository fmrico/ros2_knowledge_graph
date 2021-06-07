cmake_minimum_required(VERSION 3.5)
project(ros2_knowledge_graph)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_knowledge_graph_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
    rclcpp
    ros2_knowledge_graph_msgs
    tf2_geometry_msgs
    tf2
    tf2_ros
)

include_directories(include)

set(BICAGRAPH_SOURCES
  src/ros2_knowledge_graph/GraphNode.cpp
)

add_library(${PROJECT_NAME} SHARED
  src/ros2_knowledge_graph/GraphNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
