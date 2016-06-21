#include <sstream>
#include <string>
#include "droneinfo.h"
#include "vector3d.h"

// ID:getX:getY:getZ:Battery
DroneInfo::DroneInfo(std::string Net, bool lead) {
  std::istringstream stream(Net);
  int id;
  double x, y, z, bat;
  stream >> id >> x >> y >> z >> bat;
  this->ID = id;
  this->location = Vector3D(x, y, z);
  this->battery = bat;
  this->isLeadDrone = lead;
}

std::string DroneInfo::toString() {
  std::ostringstream stream;
  stream << ID << " " << location.getX() << " " << location.getY() << " "
         << location.getZ() << " " << battery;
  return stream.str();
}
