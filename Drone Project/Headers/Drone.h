#pragma once
#include "DroneInfo.h"
#include "Coordinate3D.h"

class Drone
{
public:
    DroneInfo* info;
    Coordinate3D* waypoint;
    Coordinate3D* target;
    void CalculateNewWaypoint(DroneInfo* leadDrone);
    Drone(Coordinate3D* target, DroneInfo* position);
};
