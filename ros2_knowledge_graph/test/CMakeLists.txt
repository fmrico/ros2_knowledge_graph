ament_add_gtest(ros2_knowledge_graph_test ros2_knowledge_graph_test.cpp)
ament_target_dependencies(ros2_knowledge_graph_test ${dependencies})
target_link_libraries(ros2_knowledge_graph_test ${PROJECT_NAME})



add_executable(test_graph_A_node test_graph_A_node.cpp)
ament_target_dependencies(test_graph_A_node ${dependencies})
target_link_libraries(test_graph_A_node ${PROJECT_NAME})

add_executable(test_graph_B_node test_graph_B_node.cpp)
ament_target_dependencies(test_graph_B_node ${dependencies})
target_link_libraries(test_graph_B_node ${PROJECT_NAME})

add_executable(test_graph_C_node test_graph_C_node.cpp)
ament_target_dependencies(test_graph_C_node ${dependencies})
target_link_libraries(test_graph_C_node ${PROJECT_NAME})