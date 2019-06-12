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
 * @file path.h
 * @author Piotr Dulewicz (piotr.dulewicz@pwr.edu.pl)
 * @brief mmrs_simulator package
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <vector>
#include <deque>
#include <initializer_list>

#include "ros/ros.h"
#include "special_point.h"
#include "vehicle.h"
#include "sector.h"

namespace mmrs
{

/**
 * @brief class representing path of a vehicle
 * 
 * The path is represented in a simplified way, in isolation from the
 * geometry. It's divided into stages. The stages_ vector contains 
 * endpoints of each stage. 
 */

class Path
{
  /**
   * @brief vector containing locations of endpoints of each stage
   * 
   * The 1-st stage (with index 0) starts in point 0.0 and ends in
   * stages_[0], so it's length is equal to stages_[0]. i-th stage 
   * ends in stages_[i-1] point. Length of any i-th stage can be 
   * calculated as stages_[i] - stages_[i-1].
   */
  std::vector<mmrs::Sector> stages_;
  /**
   * @brief container of special points
   * 
   * Contains locations of critical and release points on path
   */
  std::deque<mmrs::SpecialPoint> special_points_;

  /**
   * @brief Used for creating special point publishers
   * 
   * This NodeHandle allows us to create publishers for "critical_points"
   * and "release_points" topics. 
   */
  ros::NodeHandle node_handle_;

  // special point publishers
  ros::Publisher critical_point_publisher_;
  ros::Publisher release_point_publisher_;

public:
  /**
   * @brief Construct a new Path object
   * 
   * @param vehicle - source of vehicle properties (radius, max velocity,
   * etc.) used for calculating special points
   * @param stages - locations of endpoints of each stage
   */
  Path(const mmrs::Vehicle &vehicle, std::initializer_list<mmrs::Sector> stages);
  /**
   * @brief checks if the vehicle has reached its next special point
   * 
   * After reaching a special point, the appropriate action is taken. For
   * critical point it's broadcasting the 'cp' event. Transition point means
   * entering a new stage by the vehicle. Release point causes both sending
   * 'rp' event and updating vehicle state (leaving corresponding stage).
   * 
   * @param vehicle - vehicle to be checked
   */
  void CheckSpecialPoints(mmrs::Vehicle &vehicle);
  /**
   * @brief checks if the vehicle has completed its path
   * 
   * @param vehicle - vehicle to be checked
   * @return true - task completed
   * @return false - task not completed yet
   */
  bool CheckIfCompleted(mmrs::Vehicle &vehicle) const;
};

} // namespace mmrs
