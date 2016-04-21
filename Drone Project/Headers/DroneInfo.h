#pragma once
#include "Coordinate3D.h"
#include <string>

using std::string;

class DroneInfo
{
private:
    //Represents the drones location
    Coordinate3D* location;
    //Drones battery life remaining
    double battery;

public:

    //ID number of the drone
    int ID;

    //Returns true if battery > 0, otherwise returns false
    bool isAlive();

    DroneInfo(string Net);

    ~DroneInfo();

    Coordinate3D* GetLocation();

    void SetLocation(float x, float y, float z);

    double GetBattery();

    void SetBattery(float batteryLevel);
};
