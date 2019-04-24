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
 * @file path.cpp
 * @author Piotr Dulewicz (piotr.dulewicz@pwr.edu.pl)
 * @brief mmrs_simulator package
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <iostream> //temporary, should be replaced by ROS_INFO later

#include "../include/mmrs_simulator/path.h"

Path::Path(const Vehicle &vehicle, std::initializer_list<double> stages) : stages_{stages}
{
  // make sure that path has at least 2 sectors
  if (stages_.size() < 2)
  {
    return;
  }

  // auxiliary variables
  double radius = vehicle.GetRadius();
  double max_vel = vehicle.GetMaxVelocity();
  double acc = vehicle.GetAcceleration();
  double braking_distance = (max_vel * max_vel) / (2 * acc);
  double critical_distance = radius + braking_distance;

  // calculate special points
  for (auto it = stages_.begin(); it != stages_.end() - 1; ++it)
  {
    special_points_.push_back(SpecialPoint{SpecialPoint::CRITICAL_POINT,
                                           *it - critical_distance});
    special_points_.push_back(SpecialPoint{SpecialPoint::RELEASE_POINT,
                                           *it + radius});
  }
  // skipping last element of stages_, because there are no special
  // points around it
}

void Path::UpdateStage(Vehicle &vehicle) const
{
  while (vehicle.GetCurrentPosition() > stages_[vehicle.GetCurrentStage()])
  {
    vehicle.ProceedToNextStage();
  }
}

void Path::CheckSpecialPoints(Vehicle &vehicle)
{
  if (special_points_.empty())
  {
    return;
  }
  while (vehicle.GetCurrentPosition() > special_points_[0].location)
  {
    switch (special_points_[0].type)
    {
    case SpecialPoint::CRITICAL_POINT:
      std::cout << "Critical point reached!" << std::endl;
      break;
    case SpecialPoint::RELEASE_POINT:
      std::cout << "Release point reached!" << std::endl;
      break;
    default:
      break;
    }
  }
  special_points_.pop_front();
}

bool Path::CheckIfCompleted(Vehicle &vehicle) const
{
  if(vehicle.GetCurrentPosition() >= stages_.back())
  {
    vehicle.Stop();
    return true;
  }
  return false;
}