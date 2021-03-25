cmake_minimum_required(VERSION 3.5)
project(ros2_knowledge_graph_terminal)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)

set(dependencies
    rclcpp
    ros2_knowledge_graph
)

include_directories(include)

add_library(terminal SHARED
  src/ros2_knowledge_graph_terminal/Terminal.cpp)
ament_target_dependencies(terminal ${dependencies})
target_compile_definitions(terminal PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_executable(${PROJECT_NAME}
  src/terminal_node.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} terminal readline)

install(TARGETS
  terminal
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
