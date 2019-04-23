#pragma once


const double kDefaultRadius = 0.5;  // meters
const double kDefaultMaxVelocity = 0.65;   // meters per second
const double kDefaultAcceleration = 0.2;   // meters per square seconds
const double kDefaultDeceleration = 0.6;   // meters per square seconds

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

  void Advance(double time_s);
  void ProceedToNextStage() {current_stage_++;}
  void Stop() {is_moving_ = false;}
};