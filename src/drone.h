#ifndef DRONE_H_
#define DRONE_H_

#include "droneinfo.h"
#include "vector3d.h"

class Drone {
 public:
  Drone(const Vector3D& target, const DroneInfo& position);

  void CalculateWaypoint(const DroneInfo& leadDrone);
  void Move(double speed);
  static double ComputeDistanceToTarget(const std::string& Net,
                                        const Vector3D& target);

  static void MoveDronesVFormation(
      std::string Net[], const Vector3D& formationVector, std::string leadDrone,
      double formationDistance, const Vector3D& target, const int num_drones);
  static void AvoidCollision(std::string Net[],
                             const std::vector<Vector3D>& changeVector,
                             const int num_drones);
  static void NetOut(std::string Net[], int State, const Vector3D& target,
                     int num_drones, int formationNumber);

  Vector3D getWaypoint() { return waypoint; }
  Vector3D getTarget() { return target; }
  DroneInfo getInfo() { return info; }

 private:
  DroneInfo info;  // current status
  Vector3D waypoint;
  Vector3D target;
};

#endif
