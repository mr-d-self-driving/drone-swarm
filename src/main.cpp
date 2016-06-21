#include <unistd.h>
#include <iostream>
#include "vector3d.h"
#include "droneinfo.h"
#include "drone.h"
#include "demo.h"
#include "socket.h"

int main() {
  std::string SelfIP = "192.168.1.50";
  std::string PartnerIP = "192.168.1.150";

  //create the socket
  if (Socket::Init(SelfIP.c_str()) != 0) {
    std::cout << "Socket initialization failed." << std::endl;
  }

  //create the socket address
  Socket::SetSockaddrIn(&Socket::remoteAddr, PartnerIP.c_str(), Socket::PORTNUM);

  // Creates the output files
  Demo::Initialize();

  /* Temporary until MavLink can get the GPS coords*/
  Vector3D self(50, 50, 100);
  Vector3D target(100, 100, 100);

  // Network lead drone
  DroneInfo demoDroneInfo("1 50 50 100 100", true);
  DroneInfo leadDrone(demoDroneInfo);

  Drone demoDrone(target, demoDroneInfo);
  demoDrone.CalculateWaypoint(leadDrone);

  //test for 10 iterations
  for (int i = 0; i < 10; i++) {
    //update self data
    demoDrone.Move(5.0f); //move the drone
    demoDrone.CalculateWaypoint(leadDrone); //calculate the drone's next waypoint

    // save and send current info
    std::string info = demoDrone.getInfo().toString(); //string to write and send
    Demo::WritePacket(&Demo::sent_packet_file, info); //write the drone's position to the file
    Socket::SendMessage(info.c_str()); //send message to remotes

    sleep(0.2);

    // receive info and write to file
    std::string message = Socket::ReceiveMessage();
    Demo::WritePacket(&Demo::rec_packet_file, message);

    if (!demoDroneInfo.isLead()) { // if this drone is not the lead drone
      leadDrone = DroneInfo(message, true); //set the lead drone to be the drone that send the message
    }
  }
  return 0;
}
