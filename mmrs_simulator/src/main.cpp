// The MIT License (MIT)

// Copyright (c) 2019 Piotr Dulewicz

//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

/**
 * @file main.cpp
 * @author Piotr Dulewicz (piotr.dulewicz@pwr.edu.pl)
 * @brief mmrs_simulator package
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "ros/ros.h"
#include "../include/mmrs_simulator/task.h"
#include "../nlohmann/json.hpp"

#include <fstream>
#include <iostream>

using nlohmann::json;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "simulator");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(mmrs::kDefaultSimulationRateHz);

  std::ifstream i("/home/piotrek/file.json");
  json j;
  i >> j;
  mmrs::Task task = j.get<mmrs::Task>();
  std::cout << j.dump(4) << std::endl;

  int step_counter = 0;

  while (ros::ok())
  {
    // execute simulation steps until all vehicles are done
    if (task.SimulationStep())
    {
      ROS_INFO("Task completed!");
      break;
    }
    ROS_INFO("Simulation step number %d", step_counter);
    step_counter++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
