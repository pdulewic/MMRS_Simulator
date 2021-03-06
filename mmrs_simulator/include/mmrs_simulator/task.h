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
 * @file task.h
 * @author Piotr Dulewicz (piotr.dulewicz@pwr.edu.pl)
 * @brief mmrs_simulator package
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <vector>
#include <string>
#include <utility>

#include "../../nlohmann/json.hpp"
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "vehicle.h"
#include "path.h"

using nlohmann::json;

namespace mmrs
{

const double kDefaultSimulationRateHz = 10.0;

/**
 * @brief class representing whole task
 * 
 * In this simulation, each vehicle must pass from the beginning to 
 * the end of its path. Class represents vehicles associated with their 
 * paths, and calculates subsequent simulation steps. 
 */
class Task
{
  int number_of_vehicles_;
  /**
   * @brief vehicles with their associated paths
   * 
   */
  std::vector<std::pair<mmrs::Vehicle, mmrs::Path>> paths_;
  /**
   * @brief number of simulation steps in one second
   * 
   */
  double rate_hz_;
  /**
   * @brief Used for creating movement permission subscriber
   */
  ros::NodeHandle node_handle_;
  ros::Subscriber permission_subscriber_; 

  /**
   * @brief moves forward single vehicle
   * 
   * @param instance - pair of vehicle and its path
   * @return true - vehicle completed its task after this step
   * @return false - vehicle didn't completed its task after this step
   */
  bool UpdateVehicle(std::pair<mmrs::Vehicle, mmrs::Path> &instance);

  /**
   * @brief callback for granting movement permission event
   * 
   * Activates when the controller assigns movement permission to a vehicle. 
   * 
   * @param msg - ROS message with vehicles ID
   */
  void PermissionCallback(const std_msgs::Int16::ConstPtr &msg);

public:
  /**
   * @brief Construct a new Task object
   * 
   * @param filename - .json file from which a task can be loaded
   * @param rate_hz - rate_hz_ initializer
   */
  Task(std::string filename = "", double rate_hz = kDefaultSimulationRateHz);
  /**
   * @brief executes one simulation step
   * 
   * @return true - whole task has been completed
   * @return false - task is not completed yet
   */
  bool SimulationStep();
  friend void mmrs::to_json(json& j, const Task& t);
  friend void mmrs::from_json(const json& j, Task& t);
};

// special functions descriebed in
// https://github.com/nlohmann/json#arbitrary-types-conversions
void to_json(json& j, const Task& t);
void from_json(const json& j, Task& t);



} // namespace mmrs