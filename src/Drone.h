#ifndef DRONE_H_
#define DRONE_H_

#include "DroneInfo.h"
#include "Coordinate3D.h"

class Drone {
 public:
  Drone(Coordinate3D* target, DroneInfo* position);
  DroneInfo* info;
  Coordinate3D* waypoint;
  Coordinate3D* target;
  void CalculateNewWaypoint(DroneInfo* leadDrone);
};

#endif
