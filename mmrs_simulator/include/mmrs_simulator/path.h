#pragma once

/**
 * @file path.h
 * @author Piotr Dulewicz (piotr.dulewicz@pwr.edu.pl)
 * @brief mmrs_simulator package
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <vector>
#include <deque>
#include <initializer_list>

#include "special_point.h"
#include "vehicle.h"

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
   * stages_[0], so it's length is equal to stages_[0]. Length of any
   * other, i-th stage can be calculated as stages_[i] - stages_[i-1].
   */
  std::vector<double> stages_;
  /**
   * @brief container of special points
   * 
   * Contains locations of critical and release points on path
   */
  std::deque<SpecialPoint> special_points_;

public:
  /**
   * @brief Construct a new Path object
   * 
   * @param vehicle - source of vehicle properties (radius, max velocity,
   * etc.) used for calculating special points
   * @param stages - locations of endpoints of each stage
   */
  Path(const Vehicle &vehicle, std::initializer_list<double> stages);
  /**
   * @brief checks if the vehicle has reached its next stage
   * 
   * @param vehicle - vehicle to be checked
   */
  void UpdateStage(Vehicle &vehicle) const;
  /**
   * @brief checks if the vehicle has reached its next special point
   * 
   * @param vehicle - vehicle to be checked
   */
  void CheckSpecialPoints(Vehicle &vehicle);
  /**
   * @brief checks if the vehicle has completed its path
   * 
   * @param vehicle - vehicle to be checked
   * @return true - task completed
   * @return false - task not completed yet
   */
  bool CheckIfCompleted(Vehicle &vehicle) const;
};