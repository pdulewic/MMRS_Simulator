#pragma once

#include <vector>
#include <string>
#include <utility>

#include "ros/ros.h"
#include "vehicle.h"
#include "path.h"

const double kDefaultSimulationRateHz = 10.0;

class Task
{
  int number_of_vehicles_;
  std::vector<std::pair<Vehicle, Path>> paths_;
  double rate_hz_;

  bool UpdateVehicle(std::pair<Vehicle, Path> &instance);

public:
  Task(std::string filename = "", double rate_hz = kDefaultSimulationRateHz);
  bool SimulationStep();
};