#ifndef DRONE_H_
#define DRONE_H_

#include "droneinfo.h"
#include "vector3d.h"

class Drone {
 public:
  Drone(const Vector3D &target, const DroneInfo &position);

  void CalculateWaypoint(const DroneInfo &leadDrone);
  void Move(double speed);
  double ComputeDistanceToTarget(const std::string &Net, const Vector3D &target);

  Vector3D getWaypoint() { return waypoint; }
  Vector3D getTarget() { return target; }
  DroneInfo getInfo() { return info; }
 private:
  DroneInfo info; //current status
  Vector3D waypoint;
  Vector3D target;
};

#endif
