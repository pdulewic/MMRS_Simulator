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
 * @file vehicle.h
 * @author Piotr Dulewicz (piotr.dulewicz@pwr.edu.pl)
 * @brief mmrs_simulator package
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once

#include <deque>

namespace mmrs
{

const double kDefaultRadius = 0.5;       // meters
const double kDefaultMaxVelocity = 0.65; // meters per second
const double kDefaultAcceleration = 0.2; // meters per square seconds
const double kDefaultDeceleration = 0.6; // meters per square seconds

/**
 * @brief Class representing single vehicle with basic dynamics
 * 
 */
class Vehicle
{
  //inline static unsigned id_counter_ = 0;

  // constant parameters
  const double radius_m_;
  const double max_velocity_ms_;
  const double acceleration_ms2_;
  const double deceleration_ms2_;
  const int id_;

  // state of the vehicle
  double current_position_m_;
  double current_velocity_ms_;
  /**
   * @brief currently occupied stages
   * 
   */
  std::deque<int> current_stages_;
  bool is_moving_;
  /**
   * @brief counts movement permissions granted by the controller
   * 
   */
  int permission_counter_;

public:
  Vehicle(int id);
  double GetRadius() const { return radius_m_; }
  double GetMaxVelocity() const { return max_velocity_ms_; }
  double GetAcceleration() const { return acceleration_ms2_; }
  double GetDeceleration() const { return deceleration_ms2_; }
  double GetCurrentPosition() const { return current_position_m_; }
  int GetID() const { return id_; }

  /**
   * @brief drives forward for time_s seconds
   * 
   * Updates the velocity and calculates the distance traveled in 
   * the next time_s seconds considering vehicles dynamics. 
   * 
   * @param time_s - duration of the movement
   */
  void Advance(double time_s);
  void EnterNextStage() { current_stages_.push_back(current_stages_.back() + 1); }
  void LeavePreviousStage() { current_stages_.pop_front(); }
  void Stop() { is_moving_ = false; }
};

} // namespace mmrs