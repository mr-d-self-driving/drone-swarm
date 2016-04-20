//Demo code for writing to & from files goes here
#include <fstream>
#include <iostream>

using std::ofstream;
using std::cout;
ofstream PositionFile, RecPacketFile, SentPacketFile;

//WritePosition()

//WriteRecievedPacket()

//WriteSentPacket()

//Initialize()
//This will create the 3 text files,
void Initialize()
{
    PositionFile.open("Position.txt");
  if (PositionFile.is_open())
  {
    PositionFile << "This is a line.\n";
    PositionFile << "This is another line.\n";
    PositionFile.close();
  }
  else cout << "Unable to open file";
}
