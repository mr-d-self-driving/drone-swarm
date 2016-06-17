#include "DroneInfo.h"
#include <sstream>
#include <string>

// ID:X:Y:Z:Battery
DroneInfo::DroneInfo(std::string Net, bool lead) {
  std::istringstream stream(Net);
  int id;
  double x, y, z, bat;
  stream >> id >> x >> y >> z >> bat;
  this->ID = id;
  this->location = new Coordinate3D(x, y, z);
  this->battery = bat;
  this->LeadDrone = lead;
}

DroneInfo::~DroneInfo() { delete location; }

bool DroneInfo::isAlive() { return (battery > 0); }

bool DroneInfo::isLead() { return LeadDrone; }

std::string DroneInfo::ToString() {
  std::ostringstream stream;
  stream << ID << " " << location->X() << " " << location->Y() << " "
         << location->Z() << " " << battery;
  return stream.str();
}
