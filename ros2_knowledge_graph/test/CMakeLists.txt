ament_add_gtest(
  ros2_knowledge_graph_node_test ros2_knowledge_graph_node_test.cpp)
ament_target_dependencies(ros2_knowledge_graph_node_test ${dependencies})
target_link_libraries(ros2_knowledge_graph_node_test ${PROJECT_NAME})

ament_add_gtest(graph_utils_test graph_utils_test.cpp)
ament_target_dependencies(graph_utils_test ${dependencies})
target_link_libraries(graph_utils_test ${PROJECT_NAME})
