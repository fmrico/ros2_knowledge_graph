uint8 UPDATE=0
uint8 REMOVE=1
uint8 SYNC=2
uint8 REQSYNC=3

uint8 NODE=0
uint8 EDGE=1
uint8 GRAPH=2

builtin_interfaces/Time stamp
string node_id
string target_node

uint8 operation_type
uint8 element_type

string removed_node
ros2_knowledge_graph_msgs/Node node
ros2_knowledge_graph_msgs/Edge edge
ros2_knowledge_graph_msgs/Graph graph
