//
// Created by kaxns on 11/18/21.
//

#include <string>
#include "windmill_dev/windmill_recognizer.h"
#include "windmill_dev/windmill_analyzer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, std::string("windmill_vision"));
  ros::NodeHandle n("~");
  // ...
  return 0;
}