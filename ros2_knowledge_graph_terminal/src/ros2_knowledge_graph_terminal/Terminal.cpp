// Copyright 2020 Intelligent Robotics Lab
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

#include <stdio.h>
#include <readline/readline.h>

#include <readline/history.h>

#include <regex>
#include <vector>
#include <list>
#include <string>
#include <memory>
#include <sstream>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "ros2_knowledge_graph_msgs/msg/graph.hpp"
#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"
#include "ros2_knowledge_graph_msgs/msg/node.hpp"
#include "ros2_knowledge_graph_msgs/msg/edge.hpp"

#include "ros2_knowledge_graph/graph_utils.hpp"

#include "ros2_knowledge_graph_terminal/Terminal.hpp"


namespace ros2_knowledge_graph_terminal
{

std::vector<std::string> tokenize(const std::string & text, const std::string delim)
{
  if (text.empty()) {
    return {};
  }

  std::vector<std::string> ret;
  size_t start = 0, end = 0;

  while (end != std::string::npos) {
    end = text.find(delim, start);
    ret.push_back(text.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }
  return ret;
}

void pop_front(std::vector<std::string> & tokens)
{
  if (!tokens.empty()) {
    tokens.erase(tokens.begin(), tokens.begin() + 1);
  }
}

// LCOV_EXCL_START
char * completion_generator(const char * text, int state)
{
  // This function is called with state=0 the first time; subsequent calls are
  // with a nonzero state. state=0 can be used to perform one-time
  // initialization for this completion session.
  static std::vector<std::string> matches;
  static size_t match_index = 0;

  std::vector<std::string> vocabulary{"add", "remove", "get", "print"};
  std::vector<std::string> vocabulary_add{"node", "edge"};
  std::vector<std::string> vocabulary_remove{"node", "edge"};
  std::vector<std::string> vocabulary_get{"node", "edge", "nodes", "edges"};

  if (state == 0) {
    // During initialization, compute the actual matches for 'text' and keep
    // them in a static vector.
    matches.clear();
    match_index = 0;

    // Collect a vector of matches: vocabulary words that begin with text.
    std::string textstr = std::string(text);

    auto current_text = tokenize(rl_line_buffer);
    std::vector<std::string> * current_vocabulary = nullptr;

    if (current_text.size() <= 1) {
      current_vocabulary = &vocabulary;
    } else {
      if (current_text.size() == 2) {
        if (current_text[0] == "add") {
          current_vocabulary = &vocabulary_add;
        } else if (current_text[0] == "remove") {
          current_vocabulary = &vocabulary_remove;
        } else if (current_text[0] == "get") {
          current_vocabulary = &vocabulary_get;
        }
      }
    }

    if (current_vocabulary == nullptr) {
      return nullptr;
    }

    for (auto word : *current_vocabulary) {
      if (word.size() >= textstr.size() &&
        word.compare(0, textstr.size(), textstr) == 0)
      {
        matches.push_back(word);
      }
    }
  }

  if (match_index >= matches.size()) {
    // We return nullptr to notify the caller no more matches are available.
    return nullptr;
  } else {
    // Return a malloc'd char* for the match. The caller frees it.
    return strdup(matches[match_index++].c_str());
  }
}

char ** completer(const char * text, int start, int end)
{
  // Don't do filename completion even if our generator finds no matches.
  rl_attempted_completion_over = 1;

  // Note: returning nullptr here will make readline use the default filename
  // completer.

  return rl_completion_matches(text, completion_generator);
}


Terminal::Terminal()
: rclcpp::Node(
    "graph_terminal_" +
    std::to_string(static_cast<int>(rclcpp::Clock().now().seconds())))
{
}

void
Terminal::run_console()
{
  graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());
  std::string line;
  bool success = true;

  std::cout << "ROS2 Knowledge Graph console. Type \"quit\" to finish" << std::endl;

  rl_attempted_completion_function = completer;

  bool finish = false;
  while (!finish) {
    char * line = readline("> ");

    if (line == NULL || (strcmp(line, "quit") == 0)) {
      finish = true;
    }

    if (strlen(line) > 0) {
      add_history(line);

      std::string line_str(line);
      free(line);

      if (!finish) {
        clean_command(line_str);

        std::ostringstream os;
        process_command(line_str, os);
        std::cout << os.str();
      }
    }
  }

  std::cout << "Finishing..." << std::endl;
}
// LCOV_EXCL_STOP

void
Terminal::clean_command(std::string & command)
{
  // remove continuous spaces
  size_t pos;
  while ((pos = command.find("  ")) != command.npos) {
    command.erase(pos, 1);
  }

  // remove from spaces
  while (*command.begin() == ' ') {
    command.erase(0, 1);
  }

  // remove trailing spaces
  while (command[command.size() - 1] == ' ') {
    command.pop_back();
  }
}


void
Terminal::process_command(std::string & command, std::ostringstream & os)
{
  std::vector<std::string> tokens = tokenize(command);

  if (tokens.empty()) {
    return;
  }

  if (tokens[0] == "add") {
    pop_front(tokens);
    process_add(tokens, os);
  } else if (tokens[0] == "remove") {
    pop_front(tokens);
    process_remove(tokens, os);
  } else if (tokens[0] == "get") {
    pop_front(tokens);
    process_get(tokens, os);
  } else if (tokens[0] == "print") {
    pop_front(tokens);
    process_print(tokens, os);
  } else {
    os << "Command not found" << std::endl;
  }
}

void
Terminal::process_add(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    if (command[0] == "node") {
      pop_front(command);
      process_add_node(command, os);
    } else if (command[0] == "edge") {
      pop_front(command);
      process_add_edge(command, os);
    } else {
      os << "\tUsage: \n\t\tadd [node|edge]..." <<
        std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tadd [node|edge]..." <<
      std::endl;
  }
}

void
Terminal::process_add_node(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 2) {
    auto node = ros2_knowledge_graph::new_node(command[0], command[1]);
    graph_->update_node(node);
  } else {
    os << "\tUsage: \n\t\tadd node [name] [type]" <<
      std::endl;
  }
}

std::optional<ros2_knowledge_graph_msgs::msg::Edge>
Terminal::get_edge(
  const std::string & source, const std::string & target,
  const std::string & type, const std::string & content)
{
  ros2_knowledge_graph_msgs::msg::Edge ret;
  if (type == "bool") {
    ret = ros2_knowledge_graph::new_edge<bool>(source, target, content == "true");
  } else if (type == "int") {
    auto edge = ros2_knowledge_graph::new_edge<int>(source, target, atoi(content.c_str()));
    graph_->update_edge(edge);
  } else if (type == "float") {
    ret = ros2_knowledge_graph::new_edge<float>(source, target, atof(content.c_str()));
  } else if (type == "double") {
    ret = ros2_knowledge_graph::new_edge<double>(source, target, atof(content.c_str()));
  } else if (type == "string") {
    ret = ros2_knowledge_graph::new_edge<std::string>(source, target, content);
  } else if (type == "tf" || type == "tf_static") {
    auto vals = tokenize(content, ":");
    geometry_msgs::msg::TransformStamped tf1;
    if (vals.size() == 6) {
      tf1.header.frame_id = source;
      tf1.child_frame_id = target;
      tf1.header.stamp = now();
      tf1.transform.translation.x = atof(vals[0].c_str());
      tf1.transform.translation.y = atof(vals[1].c_str());
      tf1.transform.translation.z = atof(vals[2].c_str());

      tf2::Quaternion q;
      q.setRPY(atof(vals[3].c_str()), atof(vals[4].c_str()), atof(vals[5].c_str()));

      tf1.transform.rotation.x = q.x();
      tf1.transform.rotation.y = q.y();
      tf1.transform.rotation.z = q.z();
      tf1.transform.rotation.w = q.w();

      ret = ros2_knowledge_graph::new_edge<geometry_msgs::msg::TransformStamped>(
        source, target, tf1, type == "tf_static");
    } else {
      std::cerr << "Usage: x:y:z:r:p:y" << std::endl;
      return {};
    }
  } else {
    std::cerr << "ToDo: implement more types [" << type << "]" << std::endl;
    return {};
  }

  return ret;
}

void
Terminal::process_add_edge(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() >= 4) {
    while (command.size() > 4) {
      command[command.size() - 2] = command[command.size() - 2] + " " + command.back();
      command.pop_back();
    }

    const std::string & source = command[0];
    const std::string & target = command[1];
    const std::string & type = command[2];
    const std::string & data = command[3];

    auto edge = get_edge(source, target, type, data);
    if (edge.has_value()) {
      graph_->update_edge(edge.value());
    } else {
      os << "Could not add the edge [" << command[2] << "] " << command[0] <<
        " -> " << command[1] << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tadd edge [source] [target] [type] [content]" <<
      std::endl;
  }
}

void
Terminal::process_remove(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    if (command[0] == "node") {
      pop_front(command);
      process_remove_node(command, os);
    } else if (command[0] == "edge") {
      pop_front(command);
      process_remove_edge(command, os);
    } else {
      os << "\tUsage: \n\t\tremove [node|edge]..." <<
        std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tremove [node|edge]..." <<
      std::endl;
  }
}

void
Terminal::process_remove_node(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 1) {
    if (!graph_->remove_node(command[0])) {
      os << "Could not remove the node [" << command[0] << "]" << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tremove node [name] [type]" <<
      std::endl;
  }
}

void
Terminal::process_remove_edge(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() >= 4) {
    while (command.size() > 4) {
      command[command.size() - 2] = command[command.size() - 2] + " " + command.back();
      command.pop_back();
    }

    const std::string & source = command[0];
    const std::string & target = command[1];
    const std::string & type = command[2];
    const std::string & data = command[3];

    auto edge = get_edge(source, target, type, data);
    if (edge.has_value()) {
      graph_->remove_edge(edge.value());
    } else {
      os << "Could not remove the edge [" << command[2] << "] " <<
        command[0] << " -> " << command[1] << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tremove edge [source] [target] [type] [content]" <<
      std::endl;
  }
}

void
Terminal::process_get(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() > 0) {
    if (command[0] == "node") {
      pop_front(command);
      process_get_node(command, os);
    } else if (command[0] == "edge") {
      pop_front(command);
      process_get_edge(command, os);
    } else if (command[0] == "nodes") {
      pop_front(command);
      process_get_nodes(command, os);
    } else if (command[0] == "edges") {
      pop_front(command);
      process_get_edges(command, os);
    } else {
      os << "\tUsage: \n\t\tremove [node|edge]..." <<
        std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tremove [node|edge]..." <<
      std::endl;
  }
}

void
Terminal::process_get_node(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 1) {
    const auto & node = graph_->get_node(command[0]);

    if (node.has_value()) {
      os << ros2_knowledge_graph::to_string(node.value()) << std::endl;
    } else {
      os << "node [" << command[0] << "] not found" << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget node [node_name]" <<
      std::endl;
  }
}

void
Terminal::process_get_edge(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 3) {
    auto edges =
      graph_->get_edges(
      command[0], command[1],
      ros2_knowledge_graph::type_from_string(command[2]));

    os << "Edges: " << edges.size() << std::endl;
    for (const auto & edge : edges) {
      os << ros2_knowledge_graph::to_string(edge) << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget edge [source] [target] [type]" <<
      std::endl;
  }
}

void
Terminal::process_get_nodes(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 0) {
    const auto & nodes = graph_->get_nodes();

    os << "Nodes: " << nodes.size() << std::endl;
    for (const auto & node : nodes) {
      os << ros2_knowledge_graph::to_string(node) << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget nodes" <<
      std::endl;
  }
}

void
Terminal::process_get_edges(std::vector<std::string> & command, std::ostringstream & os)
{
  if (command.size() == 0) {
    const auto & edges = graph_->get_edges();

    os << "Connections: " << edges.size() << std::endl;
    for (const auto & edge : edges) {
      os << ros2_knowledge_graph::to_string(edge) << std::endl;
    }
  } else {
    os << "\tUsage: \n\t\tget edges" <<
      std::endl;
  }
}

void
Terminal::process_print(std::vector<std::string> & command, std::ostringstream & os)
{
  const auto & nodes = graph_->get_nodes();

  os << "Nodes: " << nodes.size() << std::endl;
  os << "==============" << std::endl;
  for (const auto & node : nodes) {
    os << "\t" << ros2_knowledge_graph::to_string(node) << std::endl;
  }

  const auto & edges = graph_->get_edges();

  os << std::endl;
  os << "Connections: " << edges.size() << std::endl;
  os << "==============" << std::endl;
  for (const auto & edge : edges) {
    os << "\t" << ros2_knowledge_graph::to_string(edge) << std::endl;
  }
}

}  // namespace ros2_knowledge_graph_terminal
