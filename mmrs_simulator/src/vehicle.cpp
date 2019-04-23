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