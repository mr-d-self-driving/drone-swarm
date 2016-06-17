#include <unistd.h>
#include <iostream>
#include "Coordinate3D.h"
#include "Vector3D.h"
#include "CCode.h"
#include "DroneInfo.h"
#include "Drone.h"
#include "Demo.h"

int main() {
  std::string SelfIP = "192.168.1.50";
  std::string PartnerIP = "192.168.1.150";

  if (Init(SelfIP.c_str()) != 1)
    std::cout << "Socket initialization failed." << std::endl;

  SetSendTo(PartnerIP.c_str());

  // Creates the output files
  Demo::Initialize();

  Coordinate3D *self, *target;
  self = new Coordinate3D(50, 50, 100);
  target = new Coordinate3D(100, 100, 100);

  // Network lead drone
  DroneInfo *demoDroneInfo = new DroneInfo("1 50 50 100 100", true);
  DroneInfo *leadDrone = demoDroneInfo;

  /*
  //Network not lead
  DroneInfo *demoDroneInfo = new DroneInfo("1 40 40 100 100", false);
  DroneInfo *leadDrone = new DroneInfo("1 50 50 100 100", true);
  */

  // DroneInfo *demoDroneInfoTwo = new DroneInfo("2 40 40 100 100", false);

  Drone *demoDrone = new Drone(target, demoDroneInfo);
  // Drone *demoDroneTwo = new Drone(target, demoDroneInfoTwo);
  demoDrone->CalculateNewWaypoint(leadDrone);
  // demoDroneTwo->CalculateNewWaypoint();
  for (int i = 0; i < 10; i++) {
    // cout << "Iteration: " << i << endl;
    Demo::Move(demoDrone, 5.0f);
    demoDrone->CalculateNewWaypoint(leadDrone);
    Demo::WriteSentPacket(demoDrone->info->ToString());
    std::string messageOut = demoDrone->info->ToString();
    // cout << "Message Out: " << messageOut << endl;
    char packetOut[messageOut.size()];
    strcpy(packetOut, messageOut.c_str());
    SendMessage(packetOut);
    // cout << "Message sent." << endl;
    sleep(0.2);
    std::string message(RecieveMessage());
    // cout << "Message recieved: " << message << endl;
    Demo::WriteReceivedPacket(message);
    if (!demoDroneInfo->isLead()) leadDrone = new DroneInfo(message, true);
    Demo::WritePosition(demoDrone->info->GetLocation());
  }
  return 0;
#else   // DEMO
  std::cout << "Not demo" << std::endl;
  return 0;
#endif  // DEMO
}
