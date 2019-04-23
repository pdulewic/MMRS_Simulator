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