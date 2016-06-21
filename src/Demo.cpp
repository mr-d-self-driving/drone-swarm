// Demo code for writing to & from files goes here
#include <iostream>
#include <fstream>
#include "demo.h"

// WritePosition()
void Demo::WritePosition(std::ofstream *file, const Vector3D &position) {
  *file << "Position: X: " << position.getX() << "Y: " << position.getY()
               << "Z: " << position.getZ() << std::endl;
}

// WriteReceivedPacket()
void Demo::WritePacket(std::ofstream *file, const std::string &packet) {
  *file << packet << std::endl;
}

// Initialize()
// This will create the 3 text files,
void Demo::Initialize() {
  position_file.open("Position.txt");
  position_file_two.open("Position2.txt");
  rec_packet_file.open("PacketReceived.txt");
  sent_packet_file.open("PacketSent.txt");

  if (!(position_file.is_open() || position_file_two.is_open() ||
        rec_packet_file.is_open() || sent_packet_file.is_open())) {
          std::cout << "Error opening file" << std::endl;
        }
}

// Close files
void Demo::CloseFiles() {
  position_file.close();
  position_file_two.close();
  rec_packet_file.close();
  sent_packet_file.close();
}
