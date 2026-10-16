//
// Created by koonyu on 2022/5/4.
//

#pragma once
#include <regex>
#include <string>
#include <vector>
#include <fstream>

namespace rm_windmill_vision
{
inline void readTxt(std::vector<float>& data, const std::string& location)
{
  std::ifstream file(location);
  std::string content((std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));

  std::smatch result;
  std::regex reg("(\\d{1,})");

  while (std::regex_search(content, result, reg))
  {
    data.push_back(std::stof(result.str()));
    content = result.suffix();
  }

  file.close();
}
}  // namespace rm_windmill_vision