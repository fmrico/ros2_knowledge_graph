cmake_minimum_required(VERSION 3.5)
project(ros2_knowledge_graph_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
    rclcpp
    ros2_knowledge_graph
    pluginlib
    geometry_msgs
    tf2_ros
    tf2
)

include_directories(include)

add_library(TFLayer SHARED src/ros2_kg_tf_plugin/TFLayer.cpp)
target_compile_definitions(TFLayer PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_target_dependencies(TFLayer ${dependencies})

install(TARGETS
  TFLayer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(TFLayer)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(ros2_knowledge_graph ros2_kg_plugins.xml)

ament_package()
