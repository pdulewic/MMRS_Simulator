#pragma once

struct SpecialPoint
{
  enum PointType {CRITICAL_POINT, RELEASE_POINT};
  
  PointType type;
  double location;
};