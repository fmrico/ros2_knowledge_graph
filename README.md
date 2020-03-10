# ros2_knowledge_graph

Many times, when you are developing a Software Architecture for robots, you need shared memory where to store information. There are many strategies for this. One of them is a blackboard. Another approach is a graph in which we store elements as graph nodes and relations as graph edges.

ROS2 knowledge graph provides you a way to share a graph between  ros2 nodes running in an application. The graph is distributed in all the ros2 nodes. Each ros2 node contains a replica that is synchronized with all the other replicas, guaranteeing a Strong Eventual Consistency.

This video shows the functionality of this package:

[![](https://img.youtube.com/vi/c84_3-lgorQ/0.jpg)](https://www.youtube.com/watch?v=c84_3-lgorQ&feature=youtu.be "Click to play on You Tube")

To use it, you only have to create a instance of a `ros2_knowledge_graph::GraphNode`

```
#include "ros2_knowledge_graph/GraphNode.hpp"

class NodeA : public rclcpp::Node
{
public:
  NodeA()
  : rclcpp::Node("node_A")
  {
    graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>("node_A");
    graph_->start();
  }

private:
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NodeA>();

}
```

The complete API of a `ros2_knowledge_graph::GraphNode` is at  [ros2_knowledge_graph/include/ros2_knowledge_graph/GraphNode.hpp](ros2_knowledge_graph/include/ros2_knowledge_graph/GraphNode.hpp), but it is mainly:

```
  bool add_node(const Node & node);
  bool remove_node(const std::string node);
  bool exist_node(const std::string node);
  std::optional<Node> get_node(const std::string node);

  bool add_edge(const Edge & edge);

  bool remove_edge(const Edge & edge);
  bool exist_edge(const Edge & edge);
  void get_edges(
    const std::string & source, const std::string & target, const std::string & type,
    std::vector<Edge> & result);
```

I use [Pluginlib](https://github.com/ros/pluginlib/tree/eloquent) to add special funtionality to nodes and edges depending on its type. Check [Plugins](https://github.com/fmrico/ros2_knowledge_graph/tree/master/ros2_knowledge_graph_plugins) for an example. 

Have fun!!!
