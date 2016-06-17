// Demo code for writing to & from files goes here
#include "Demo.h"
#include <fstream>
#include <iostream>
#include "Coordinate3D.h"

static std::ofstream Demo::PositionFile;

// WritePosition()
void Demo::WritePosition(Coordinate3D *position) {
  position_file << "Position: X: " << position->X() << "Y: " << position->Y()
               << "Z: " << position->Z() << std::endl;
}

void Demo::WritePositionTwo(Coordinate3D *position) {
  position_fileTwo << "Position: X: " << position->X() << "Y: " << position->Y()
                  << "Z: " << position->Z() << std::endl;
}

// WriteReceivedPacket()
void Demo::WriteReceivedPacket(std::string packet) {
  rec_packet_file << packet << std::endl;
}

// WriteSentPacket()
void Demo::WriteSentPacket(std::string packet) {
  sent_packet_file << packet << std::endl;
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

void Demo::Move(Drone *drone, double speed) {
  Coordinate3D *currentLocation = drone->info->GetLocation();
  Vector3D *movementVector = *(drone->waypoint) - *currentLocation;
  movementVector = movementVector->UnitVector();
  movementVector = *(movementVector)*speed;
  drone->info->SetLocation(currentLocation->X() + movementVector->X(),
                           currentLocation->Y() + movementVector->Y(),
                           currentLocation->Z() + movementVector->Z());
}
