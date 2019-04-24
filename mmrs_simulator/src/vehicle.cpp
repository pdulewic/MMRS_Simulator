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
 * @file vehicle.cpp
 * @author Piotr Dulewicz (piotr.dulewicz@pwr.edu.pl)
 * @brief mmrs_simulator package
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "../include/mmrs_simulator/vehicle.h"

Vehicle::Vehicle() : radius_m_{kDefaultRadius},
                     max_velocity_ms_{kDefaultMaxVelocity},
                     acceleration_ms2_{kDefaultAcceleration},
                     deceleration_ms2_{kDefaultDeceleration},
                     current_position_m_{0.0},
                     current_velocity_ms_{0.0},
                     current_stage_{0},
                     is_moving_{true}
{
}

void Vehicle::Advance(double time_s)
{
  if (is_moving_)
  {
    // update velocity
    if (current_velocity_ms_ < max_velocity_ms_)
    {
      current_velocity_ms_ += time_s * acceleration_ms2_;
      if (current_velocity_ms_ > max_velocity_ms_)
      {
        current_velocity_ms_ = max_velocity_ms_;
      }
    }
  }
  else
  {
    //update velocity
    if (current_velocity_ms_ > 0.0)
    {
      current_velocity_ms_ -= time_s * deceleration_ms2_;
      if (current_velocity_ms_ < 0.0)
      {
        current_velocity_ms_ = 0.0;
      }
    }
  }
  // calculate shift distance
  current_position_m_ += time_s * current_velocity_ms_;
}