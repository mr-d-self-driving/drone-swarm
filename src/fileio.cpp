// FileIO code for writing to & from files goes here
#include <iostream>
#include <fstream>
#include "fileio.h"

std::ofstream FileIO::position_file;
std::ofstream FileIO::position_file_two;
std::ofstream FileIO::rec_packet_file;
std::ofstream FileIO::sent_packet_file;

// WritePosition()
void FileIO::WritePosition(std::ofstream *file, const Vector3D &position) {
  *file << "Position: X: " << position.getX() << "Y: " << position.getY()
               << "Z: " << position.getZ() << std::endl;
}

// WriteReceivedPacket()
void FileIO::WritePacket(std::ofstream *file, const std::string &packet) {
  *file << packet << std::endl;
}

// Initialize()
// This will create the 3 text files,
void FileIO::Initialize() {
  position_file.open("position.txt");
  position_file_two.open("position2.txt");
  rec_packet_file.open("packet_received.txt");
  sent_packet_file.open("packet_sent.txt");

  if (!(position_file.is_open() || position_file_two.is_open() ||
        rec_packet_file.is_open() || sent_packet_file.is_open())) {
          std::cout << "Error opening file" << std::endl;
        }
}

// Close files
void FileIO::CloseFiles() {
  position_file.close();
  position_file_two.close();
  rec_packet_file.close();
  sent_packet_file.close();
}

// for now format files such that the first line is your ip. later on, once BATMAN is fully implemented,
// we can have a list of all the ip's and self is automatically removed
void FileIO::ReadIP(std::string *self, std::vector<std::string> *remotes) {
  std::ifstream file("ip.txt");
  std::string str;

  std::getline(file, *self);

  while (std::getline(file, str)) {
    //if(str.find_first_not_of("\t\n ") != std::string::npos) {
  //    std::cout << "found a non whitespace line" << std::endl;
      remotes->push_back(str);
//    }
  }
}
