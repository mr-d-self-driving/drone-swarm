#include "DroneInfo.h"
#include <sstream>
#include <string>

using std::istringstream;
using std::ostringstream;
using std::endl;

bool DroneInfo::isAlive()
{
    return (battery > 0);
}

//ID:X:Y:Z:Battery
DroneInfo::DroneInfo(string Net, bool lead)
{
    istringstream stream(Net);
    int id;
    double x, y, z, bat;
    stream >> id >> x >> y >> z >> bat;
    this->ID = id;
    this->location = new Coordinate3D(x, y, z);
    this->battery = bat;
    this->LeadDrone = lead;
}

DroneInfo::~DroneInfo()
{
    delete location;
}

Coordinate3D* DroneInfo::GetLocation()
{
    return location;
}

void DroneInfo::SetLocation(float x, float y, float z)
{
    location = new Coordinate3D(x, y, z);
}

double DroneInfo::GetBattery()
{
    return battery;
}

void DroneInfo::SetBattery(float batteryLevel)
{
    battery = batteryLevel;
}

bool DroneInfo::isLead()
{
    return LeadDrone;
}

string DroneInfo::ToString()
{
    ostringstream stream;
    stream << ID << " " << location->X() << " " << location->Y() << " " << location->Z() << " " << battery;
    return stream.str();
}
