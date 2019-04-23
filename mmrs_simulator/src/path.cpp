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