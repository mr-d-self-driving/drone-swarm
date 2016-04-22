#include "Coordinate3D.h"
#include "Vector3D.h"
#include <iostream>
#include "CCode.h"
#include "DroneInfo.h"
#include "Drone.h"
#include <unistd.h>

//Enable the demo code
#define DEMO 1

#ifdef DEMO
#include "Demo.h"
#endif // DEMO

using std::cout;
using std::endl;
using std::string;

int main()
{

    #ifdef DEMO
    cout << "Demo detected" << endl;

    string SelfIP, PartnerIP;
    SelfIP = "";
    PartnerIP = "";

    if (Init(SelfIP.c_str()) != 1)
        cout << "Socket initialization failed." << endl;

    SetSendTo(PartnerIP.c_str());

    //Creates the output files
    Demo::Initialize();

    Coordinate3D *self, *target;
	self = new Coordinate3D(50, 50 , 100);
	target = new Coordinate3D(100, 100, 100);

	//Network lead drone
    DroneInfo *demoDroneInfo = new DroneInfo("1 50 50 100 100", true);
    DroneInfo *leadDrone = demoDroneInfo;

    /*
    //Network not lead
    DroneInfo *demoDroneInfo = new DroneInfo("1 40 40 100 100", false);
    DroneInfo *leadDrone = new DroneInfo("1 50 50 100 100", true);

    */
    //DroneInfo *demoDroneInfoTwo = new DroneInfo("2 40 40 100 100", false);

    Drone *demoDrone = new Drone(target, demoDroneInfo);
    //Drone *demoDroneTwo = new Drone(target, demoDroneInfoTwo);
    demoDrone->CalculateNewWaypoint(leadDrone);
    //demoDroneTwo->CalculateNewWaypoint();
    for (int i = 0; i < 10; i++)
    {
        Demo::Move(demoDrone, 5.0f);
        demoDrone->CalculateNewWaypoint(leadDrone);
        cout << demoDroneInfo->ToString();
        Demo::WriteSentPacket(demoDrone->info->ToString());
        string messageOut = demoDrone->info->ToString();
        char packetOut [messageOut.size()];
        strcpy(packetOut, messageOut.c_str());
        SendMessage(packetOut);
        sleep(1);
        string message(RecieveMessage());
        Demo::WriteReceivedPacket(message);
        if (!demoDroneInfo->isLead())
            leadDrone = new DroneInfo(message, true);
        //Demo::Move(demoDroneTwo, 5.0f);
        //demoDroneTwo->CalculateNewWaypoint();
        Demo::WritePosition(demoDrone->info->GetLocation());
        //Demo::WritePositionTwo(demoDroneTwo->info->GetLocation());
    }

    return 0;
    #else // DEMO
    cout << "Not demo" << endl;
    return 0;
	#endif // DEMO
}
