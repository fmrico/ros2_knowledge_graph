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


#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "ros2_knowledge_graph_terminal/Terminal.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto terminal_node = std::make_shared<ros2_knowledge_graph_terminal::Terminal>();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(terminal_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  terminal_node->run_console();

  finish = true;
  t.join();

  rclcpp::shutdown();
  return 0;
}
