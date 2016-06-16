#include <iostream>
#include "CCode.h"

int NetworkTest() {
  std::string hostIP, remoteIP;
  std::cout << "Enter host ip address" << std::endl;
  std::cin >> hostIP;

  std::cout << "Enter destination ip address" << std::endl;
  std::cin >> remoteIP;

  const char *hostIPCString, *remoteIPCString;

  hostIPCString = hostIP.c_str();
  remoteIPCString = remoteIP.c_str();

  if (Init(hostIPCString) < 1) {
    std::cout << "Init failed" << std::endl;
    return -1;
  }

  if (SetSendTo(remoteIPCString) < 1) {
    std::cout << "Set remote failed" << std::endl;
    return -1;
  }

  std::cout << "Init complete" << std::endl;

  unsigned char message[2048];
  bool exit = false;
  while (!exit) {
    std::cout << "Menu:" << std::endl
              << "1) Send message" << std::endl
              << "2) Recieve message" << std::endl
              << "3) Exit" << std::endl;
    int menuSelection;
    std::cin >> menuSelection;
    switch (menuSelection) {
      case 1:
        std::cout << "Enter message" << std::endl;
        std::cin >> message;
        SendMessage(message);
        break;
      case 2:
        // message = RecieveMessage();
        std::cout << "Message: " << message << std::endl;
        break;
      case 3:
        exit = true;
        break;
    }
  }
  return 0;
}
