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
 * @file vehicle.h
 * @author Piotr Dulewicz (piotr.dulewicz@pwr.edu.pl)
 * @brief mmrs_simulator package
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#pragma once


const double kDefaultRadius = 0.5;  // meters
const double kDefaultMaxVelocity = 0.65;   // meters per second
const double kDefaultAcceleration = 0.2;   // meters per square seconds
const double kDefaultDeceleration = 0.6;   // meters per square seconds

/**
 * @brief Class representing single vehicle with basic dynamics
 * 
 */
class Vehicle
{
  // constant parameters
  const double radius_m_;
  const double max_velocity_ms_;
  const double acceleration_ms2_;
  const double deceleration_ms2_;

  // state of the vehicle
  double current_position_m_;
  double current_velocity_ms_;
  int current_stage_;
  bool is_moving_;

public:
  Vehicle();
  double GetRadius() const {return radius_m_;}
  double GetMaxVelocity() const {return max_velocity_ms_;}
  double GetAcceleration() const {return acceleration_ms2_;}
  double GetDeceleration() const {return deceleration_ms2_;}
  double GetCurrentStage() const {return current_stage_;}
  double GetCurrentPosition() const {return current_position_m_;}

  /**
   * @brief drives forward for time_s seconds
   * 
   * Updates the velocity and calculates the distance traveled in 
   * the next time_s seconds. 
   * 
   * @param time_s - duration of the move
   */
  void Advance(double time_s);
  void ProceedToNextStage() {current_stage_++;}
  void Stop() {is_moving_ = false;}
};