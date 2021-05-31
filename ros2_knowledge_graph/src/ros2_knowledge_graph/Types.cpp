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

#include <string>
#include <vector>
#include <algorithm>
#include <cctype>

#include "ros2_knowledge_graph/Types.hpp"

namespace ros2_knowledge_graph
{

std::vector<std::string> tokenize(const std::string & text, const std::string & delim)
{
  std::vector<std::string> ret;
  size_t start = 0, end = 0;

  while (end != std::string::npos) {
    end = text.find(delim, start);
    ret.push_back(text.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
    start = ((end > (std::string::npos - 1)) ? std::string::npos : (end + delim.length() - 1) + 1);
  }
  return ret;
}
bool operator==(const Node & op1, const Node & op2)
{
  return op1.name == op2.name && op1.type == op2.type;
}

bool operator==(const Edge & op1, const Edge & op2)
{
  return op1.content == op2.content && op1.type == op2.type;
}

std::string to_property(std::vector<double> values)
{
  if (values.empty()) {
    return "";
  }

  std::string ret = std::to_string(values[0]);
  for (int i = 1; i < values.size(); i++) {
    ret = ret + ", " + std::to_string(values[i]);
  }

  return ret;
}

std::string to_property(bool value)
{
  if (value) {
    return "True";
  } else {
    return "False";
  }
}

std::string to_property(int value)
{
  return std::to_string(value);
}

std::string to_property(double value)
{
  return std::to_string(value);
}

std::vector<double> property_as_vector(const std::string & property)
{
  std::vector<double> ret;
  auto tokens = tokenize(property, ",");
  for (const auto & token : tokens) {
    ret.push_back(atof(token.c_str()));
  }
  return ret;
}

bool property_as_bool(const std::string & property)
{
  std::string low;
  std::transform(property.begin(), property.end(), low.begin(),
    [](unsigned char c){ return std::tolower(c); });

  return low == "true";
}

int property_as_int(const std::string & property)
{
  return atoi(property.c_str());
}

double property_as_double(const std::string & property)
{
  return atof(property.c_str());
}


}  // namespace ros2_knowledge_graph
