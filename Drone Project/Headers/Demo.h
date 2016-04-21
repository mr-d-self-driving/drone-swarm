#pragma once
#include "Coordinate3D.h"
#include <string>

using std::string;

//Demo code for writing to & from files goes here
class Demo
{
public:
static void WritePosition(Coordinate3D* position);

static void WriteReceivedPacket(string packet);

static void WriteSentPacket(string packet);

//Initialize()
//This will create the 3 text files,
static void Initialize();

//Close the files
static void CloseFiles();
};
