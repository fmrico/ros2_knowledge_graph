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


#include <vector>
#include <list>
#include <random>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "ros2_knowledge_graph/GraphNode.hpp"

std::vector<std::string> tokenize(const std::string & text)
{
  std::vector<std::string> ret;
  size_t start = 0, end = 0;

  while (end != std::string::npos) {
    end = text.find(" ", start);
    ret.push_back(text.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }
  return ret;
}


class GraphTerminal
{
public:
  explicit GraphTerminal(const std::string & id)
  : graph_id_(id)
  {
    
  }

  void run_console()
  {
    graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(graph_id_);
    graph_->start();

    std::string line;
    bool success = true;

    std::cout << "ROS2 Knowledge Graph console. Type \"quit\" to finish" << std::endl;
    std::cout << "> ";
    while (std::getline(std::cin, line)) {
      if (line == "quit") {
        break;
      }

      process_command(line);
      std::cout << "> ";
    }

    std::cout << "Finishing..." << std::endl;
  }

  void list_nodes(void)
  {
    auto nodes = graph_->get_nodes();

    std::cout << "total nodes: " << graph_->get_num_nodes() << std::endl;
    for (const auto & node : nodes) {
      std::cout << "\t" << node.second.to_string() << std::endl;
    }
  }

  void list_edges(void)
  {
    std::cout << "total string edges: " << graph_->get_num_edges() << std::endl;
    for (const auto & pair : graph_->get_edges()) {
      for (const auto & edge : pair.second) {
          std::cout << "\t" << edge.to_string() << std::endl;
      }
    }
  }

  void process_list(const std::vector<std::string> & command)
  {
    if (command.size() == 1) {
      list_nodes();
      list_edges();
    } else if ((command.size() == 2) && (command[1] == "nodes")) {
      list_nodes();
    } else if ((command.size() == 2) && (command[1] == "edges")) {
      list_edges();
    } else {
      std::cout << "\tUsage: list [nodes|edges]" << std::endl;
    }
  }

  void process_add(const std::vector<std::string> & command)
  {
    if (command.size() > 1) {
      if (command[1] == "node") {
        if (command.size() != 4) {
          std::cout << "\tUsage: \n\t\tadd node id type" << std::endl;
        } else {
          graph_->add_node(ros2_knowledge_graph::Node{command[2], command[3]});
        }
      } else if (command[1] == "edge") {
        if (command.size() != 5) {
          std::cout << "\t\tadd edge source target data" << std::endl;
        } else {
          graph_->add_edge(ros2_knowledge_graph::Edge{command[4], "no_type", command[2], command[3]});
        }
      } else {
        std::cout << "\tUsage: \n\t\tadd [node|edge]..." << std::endl;
      }  
    } else {
      std::cout << "\tUsage: \n\t\tadd [node|edge]..." << std::endl;
    }
  }


  void process_remove(const std::vector<std::string> & command)
  {
    if (command.size() > 1) {
      if (command[1] == "node") {
        if (command.size() != 3) {
          std::cout << "\tUsage: \n\t\tremove node id" << std::endl;
        } else {
          graph_->remove_node(command[2]);
        }
      } else if (command[1] == "edge") {
        if (command.size() != 5) {
          std::cout << "\t\tremove edge source target data" << std::endl;
        } else {
          graph_->remove_edge(
              ros2_knowledge_graph::Edge{command[4], "no_type", command[2], command[3]});
        }  
      } else {
        std::cout << "\tUsage: \n\t\tremove [node|edge]..." << std::endl;
      }
    } else {
      std::cout << "\tUsage: \n\t\tremove [node|edge]..." << std::endl;
    }
  }

  void process_command(const std::string & command)
  {
    std::vector<std::string> tokens = tokenize(command);

    if (tokens.empty()) {
      return;
    }

    if (tokens[0] == "list") {
      process_list(tokens);
    } else if (tokens[0] == "add") {
      process_add(tokens);
    } else if (tokens[0] == "remove") {
      process_remove(tokens);
    } else {
      std::cout << "Command not found" << std::endl;
    }
  }

private:
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
  std::string graph_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1, 10000);

  GraphTerminal terminal("graph_terminal" + std::to_string(dis(gen)));
  terminal.run_console();

  rclcpp::shutdown();

  return 0;
}
