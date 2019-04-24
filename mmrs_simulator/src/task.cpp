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

/* code */


/**
 * @file task.cpp
 * @author Piotr Dulewicz (piotr.dulewicz@pwr.edu.pl)
 * @brief mmrs_simulator package
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "../include/mmrs_simulator/task.h"

bool Task::UpdateVehicle(std::pair<Vehicle, Path> &instance)
{
  instance.first.Advance(1.0 / rate_hz_);
  instance.second.UpdateStage(instance.first);
  return instance.second.CheckIfCompleted(instance.first);
}

Task::Task(std::string filename, double rate_hz): rate_hz_{rate_hz}
{
  if (filename.empty())
  {
    //hardcoded initialization, only for testing
    number_of_vehicles_ = 5;
    Vehicle default_vehicle = Vehicle{};
    for (int i = 0; i < number_of_vehicles_; ++i)
    {
      //vehicles_.push_back(default_vehicle);
      paths_.push_back(std::make_pair(default_vehicle,
                                      Path{
                                        default_vehicle, {5.6, 8.1, 6.4, 9.2, 7.7}
                                      }));
    }
  }
}

bool Task::SimulationStep()
{
  bool is_task_completed = true;
  for (auto &vehicle_and_path : paths_)
  {
    if (!UpdateVehicle(vehicle_and_path))
    {
      is_task_completed = false;
    }
  }
  return is_task_completed;
}