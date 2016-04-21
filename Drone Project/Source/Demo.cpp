//Demo code for writing to & from files goes here
#include "Demo.h"
#include <fstream>
#include <iostream>
#include "Coordinate3D.h"

using std::ofstream;
using std::cout;
using std::endl;
using std::string;

ofstream PositionFile, RecPacketFile, SentPacketFile;

//WritePosition()
void Demo::WritePosition(Coordinate3D* position )
{
    PositionFile << "Position: X: " << position->X() << "Y: " << position->Y() << "Z: " << position->Z() << endl;
}

//WriteReceivedPacket()
void Demo::WriteReceivedPacket(string packet)
{
    RecPacketFile << packet << endl;
}

//WriteSentPacket()
void Demo::WriteSentPacket(string packet)
{
    SentPacketFile << packet << endl;
}

//Initialize()
//This will create the 3 text files,
void Demo::Initialize()
{
    PositionFile.open("Position.txt");
    RecPacketFile.open("PacketReceived.txt");
    SentPacketFile.open("PacketSent.txt");

    if (!(PositionFile.is_open() || RecPacketFile.is_open() || SentPacketFile.is_open()))
        cout << "Error opening file" << endl;
}

//Close files
void Demo::CloseFiles()
{
    PositionFile.close();
    RecPacketFile.close();
    SentPacketFile.close();
}

void Demo::Move(Drone *drone, double speed)
{
    Coordinate3D *currentLocation = drone->info->GetLocation();
    Vector3D *movementVector = *(drone->waypoint) - *currentLocation;
    movementVector = movementVector->UnitVector();
    movementVector = *(movementVector) * speed;
    drone->info->SetLocation(currentLocation->X() + movementVector->X(), currentLocation->Y() + movementVector->Y(), currentLocation->Z() + movementVector->Z());
}
