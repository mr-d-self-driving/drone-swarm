#pragma once
#include "DroneInfo.h"
#include "Coordinate3D.h"

class Drone
{
public:
    DroneInfo* info;
    Coordinate3D* waypoint;
    Coordinate3D* target;
    bool isLead;
    void CalculateNewWaypoint();
    Drone(Coordinate3D* target, DroneInfo* position);
};
