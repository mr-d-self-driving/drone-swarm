#pragma once
#ifndef CCODE_H_
#define CCODE_H_

//#ifdef cplusplus_
extern "C" {
//#endif

#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>  /* for fprintf */
#include <string.h> /* for memcpy */
#include <arpa/inet.h>

#define PORTNUM 1200
#define BUFFERSIZE 2048

// Descriptor for our socket
int ourSocketDescriptor;

/**
With the IP set command I don't think we need to create sockaddr_in objects for
every drone.
It might cut down on overhead though to create one for each at startup and have
a "SendToAll" type function.
**/

// IPv4 info struct, stores IP addresses
struct sockaddr_in myAddr;  // The info for this host, use for initialization

struct sockaddr_in remoteAddr;  // Stores info for other unit

// Needed parameter for system calls
socklen_t addrlen = sizeof(myAddr);

// messageBufferfer to store data, change from character to something else
// later, message sent is a void *
char messageBuffer[BUFFERSIZE];

// Might not be needed, only for testing completeness of data, we could make
// sure sent messages are always constant lengths, pad with 0 if needed?
int bytesRecieved;

// Sets up the socket with the given IP address and hard coded port. Returns 1
// on success and 0 on error.
int Init(const char *IPAddr) {
  // Create the socket, AF_INET = IPv4 and SOCK_DGRAM = UDP. The final parameter
  // is for setting flags.
  if ((ourSocketDescriptor = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("cannot create socket\n");
    return 0;
  }

  // bind the socket to given IP address and a specific port

  memset((char *)&myAddr, 0, sizeof(myAddr));  // Clear the memory, just in case
  myAddr.sin_family = AF_INET;  // IPv4
  inet_pton(AF_INET, IPAddr, &(myAddr.sin_addr));  // Set IP and makes sure it
                                                   // has the network
                                                   // endian-ness
  myAddr.sin_port = htons(PORTNUM);  // Set Port number, htons() sets
                                     // endian-ness

  // Bind the socket created earlier to the given myAddr info.
  if (bind(ourSocketDescriptor, (struct sockaddr *)&myAddr, sizeof(myAddr)) <
      0) {
    perror("bind failed");
    return 0;
  }

  return 1;
}

// Sets the info for the destination of sent messages
int SetSendTo(const char *IPAddr) {
  int status = 0;
  // Fill in IP address and port
  memset((char *)&remoteAddr, 0,
         sizeof(remoteAddr));  // Clear the memory, just in case
  remoteAddr.sin_family = AF_INET;  // IPv4
  status = inet_pton(AF_INET, IPAddr,
                     &(remoteAddr.sin_addr));  // Set IP and makes sure it has
                                               // the network endian-ness
  remoteAddr.sin_port =
      htons(PORTNUM);  // Set Port number, htons() sets endian-ness
  return status;
}

// Sends a character message to remoteAddr. Returns 0 on failure, 1 on success.
int SendMessage(char *message) {
  // sendto(int socketDescriptor, void * message, length of message, flag
  //	sockaddr * addressToSendTo, sizeof sockaddr)
  // sockaddr is a generic version for ipv4 and ipv6 so we can cast our struct
  // to it
  if (sendto(ourSocketDescriptor, message, strlen((char *)message), 0,
             (struct sockaddr *)&remoteAddr, sizeof(remoteAddr)) < 0) {
    perror("sendto failed");
    return 0;
  }
  return 1;
}

// Recieves a message sent to the address in myAddr. Returns NULL on failure.
char *RecieveMessage() {
  // Holds the data on the message sender
  struct sockaddr_in remoteAddr;

  // recieve message from socket
  // Parameters: socket, buffer, buffer length, flags, struct to store info on
  // sender, size of that struct
  bytesRecieved =
      recvfrom(ourSocketDescriptor, messageBuffer, BUFFERSIZE, MSG_DONTWAIT,
               (struct sockaddr *)&remoteAddr, &addrlen);
  // printf("received %d bytes\n", bytesRecieved);
  if (bytesRecieved > 0) {
    messageBuffer[bytesRecieved] = 0;
    return messageBuffer;
  }
  return NULL;
}

//#ifdef cplusplus_
}
//#endif

#endif // CCODE_H_
