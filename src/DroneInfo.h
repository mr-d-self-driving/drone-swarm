#ifndef DRONEINFO_H_
#define DRONEINFO_H_

#include "Coordinate3D.h"
#include <string>

class DroneInfo {
 public:
  DroneInfo(std::string Net, bool lead);
 ~DroneInfo();

  bool isAlive(); // Returns true if battery > 0, otherwise returns false
  bool isLead();
  void SetLocation(float x, float y, float z);
  std::string ToString();

  inline void SetLocation(float x, float y, float z)
                            { location = new Coordinate3D(x, y, z); }
  inline void SetBattery(float batteryLevel)
                            { battery = batteryLevel; }
  inline double GetBattery() { return battery; }
  inline Coordinate3D* GetLocation() { return location; }

 private:
  Coordinate3D* location; // Represents the drones location
  double battery; // Drones battery life remaining
  bool LeadDrone;
  int ID; // ID number of the drone
};

#endif
