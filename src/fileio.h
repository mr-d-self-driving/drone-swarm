#ifndef FILEIO_H_
#define FILEIO_H_

#include <string>
#include "drone.h"
#include "vector3d.h"

// FileIO code for writing to & from files goes here
class FileIO {
 public:
  static void ReadIP(std::string *self, std::vector<std::string> *remotes);
  static void WritePosition(std::ofstream *file, const Vector3D &position);
  static void WritePacket(std::ofstream *file, const std::string &packet);

  static void Initialize(); // This will create the 3 text files,
  static void CloseFiles(); // Close the files

 //private:
  static std::ofstream position_file;
  static std::ofstream position_file_two;
  static std::ofstream rec_packet_file;
  static std::ofstream sent_packet_file;
};

#endif