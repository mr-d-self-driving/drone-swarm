#ifndef SOCKET_H_
#define SOCKET_h_

namespace Socket {

constexpr int PORTNUM = 1200;
constexpr int BUFFERSIZE = 2048;

int SetSockaddrIn(struct sockaddr_in *addr, const char *ip, uint16_t port);
int Init(const char *IPAddr);
int SendMessage(const char *message);
char *ReceiveMessage();

// IPv4 info struct, stores IP addresses
// The info for this host, use for initialization
struct sockaddr_in myAddr;
struct sockaddr_in remoteAddr;  // Stores info for other unit

// Might not be needed, only for testing completeness of data, we could make
// sure sent messages are always constant lengths, pad with 0 if needed?
int bytesRecieved;
int ourSocketDescriptor; // Descriptor for our socket

// messageBufferfer to store data, change from character to something else
// later, message sent is a void *
char messageBuffer[BUFFERSIZE];
};

#endif // SOCKET_H_
