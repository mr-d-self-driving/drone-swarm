#ifndef DRONEINFO_H_
#define DRONEINFO_H_

#include <string>
#include "vector3d.h"

class DroneInfo {
 public:
  DroneInfo() {}
  DroneInfo(std::string Net, bool lead);

  inline bool isAlive() { return (battery > 0); } // Returns true if battery > 0, otherwise returns false
  std::string toString();

  inline void setLocation(const Vector3D &location) { this->location = location; }
  inline void setLocation(float x, float y, float z) { location = Vector3D(x, y, z); }
  inline void setBattery(float batteryLevel)
                            { battery = batteryLevel; }
  inline double getBattery() const { return battery; }
  inline Vector3D getLocation() const { return location; }
  inline bool isLead() { return isLeadDrone; }

 private:
  Vector3D location; // Represents the drones location
  double battery; // Drones battery life remaining
  bool isLeadDrone;
  int ID; // ID number of the drone
};

#endif
