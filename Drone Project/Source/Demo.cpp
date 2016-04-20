//Demo code for writing to & from files goes here
#include <fstream>
#include <iostream>
#include "Coordinate3D.h"

using std::ofstream;
using std::cout;
using std::endl;
using std::string;

ofstream PositionFile, RecPacketFile, SentPacketFile;

//WritePosition()
void WritePosition(Coordinate3D position )
{
    PositionFile << "Position: X: " << position.X() << "Y: " << position.Y() << "Z: " << position.Z() << endl;
}

//WriteReceivedPacket()
void WriteReceivedPacket(string packet)
{
    RecPacketFile << packet << endl;
}

//WriteSentPacket()
void WriteSentPacket(string packet)
{
    SentPacketFile << packet << endl;
}

//Initialize()
//This will create the 3 text files,
void Initialize()
{
    PositionFile.open("Position.txt");
    RecPacketFile.open("PacketReceived.txt");
    SentPacketFile.open("PacketSent.txt");

    if (!(PositionFile.is_open() || RecPacketFile.is_open() || SentPacketFile.is_open()))
        cout << "Error opening file" << endl;
}

//Close files
void CloseFiles()
{
    PositionFile.close();
    RecPacketFile.close();
    SentPacketFile.close();
}
