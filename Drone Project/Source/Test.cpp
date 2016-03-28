#include <iostream>
#include "CCode.h"


using std::cout;
using std::endl;
using std::cin;
using std::string;


int NetworkTest()
{
	string hostIP, remoteIP;
	cout << "Enter host ip address" << endl;
	cin >> hostIP;

	cout << "Enter destination ip address" << endl;
	cin >> remoteIP;

	const char *hostIPCString, *remoteIPCString;

	hostIPCString = hostIP.c_str();
	remoteIPCString = remoteIP.c_str();

	if (Init(hostIPCString) < 1)
	{
		cout << "Init failed" << endl;
		return -1;
	}

	if (SetSendTo(remoteIPCString) < 1)
	{
		cout << "Set remote failed" << endl;
		return -1;
	}

	cout << "Init complete" << endl;

	unsigned char message[2048];
	bool exit = false;
	while(!exit)
	{
		cout << "Menu:" << endl << "1) Send message" << endl << "2) Recieve message" << endl << "3) Exit" << endl;
		int menuSelection;
		cin >> menuSelection;
		switch(menuSelection)
		{
			case 1:
				cout << "Enter message" << endl;
				cin >> message;
				SendMessage(message);
				break;
			case 2:
				//message = RecieveMessage();
				cout << "Message: " << message << endl;
				break;
			case 3:
				exit = true;
				break;
		}
	}
	return 0;
}
