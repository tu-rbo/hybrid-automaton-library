/*
 *\brief TCPcomm class
 *\detailed The TCPcomm class implements communication over a network based
 *          on the TCP protocol 
 *$Author: dubik $
 *$Date: 2007/04/17 17:31:28 $
 *$Revision: 1.2 $
 */

#ifndef __TCPCOMM__
#define __TCPCOMM__

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#ifdef __QNX__
#include <sys/netmgr.h>
#include <sys/iomsg.h>
#include <sys/neutrino.h>
#endif

#ifdef _WIN32
#pragma comment (lib, "ws2_32.lib")
# include <windows.h>
# include <winsock.h>
#include <string> 
#else
#include <stdint.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <pthread.h>
#endif

typedef int SOCKET_DESC;
typedef struct{
	SOCKET_DESC id;
	struct sockaddr_in saServer;
}SOCKET_HANDLE_STRUCT;
typedef SOCKET_HANDLE_STRUCT *SOCKET_HANDLE;

class TCPcomm{
public:
	TCPcomm();
	SOCKET_HANDLE OpenTCPConnection(const char *ServerName, short nPort);
	int GetServerIP(const char *ServerName, struct hostent** lpHostEntry);
	int SendData( SOCKET_HANDLE socket,const char* data, int data_length); 
	int RecvData( SOCKET_HANDLE socket, char* buffer, int buffer_size);
	int CloseConnection(SOCKET_HANDLE socket_handle);
	SOCKET_HANDLE OpenListeningSocket (const char *ServerHostname, int ServerPortNo);
	SOCKET_HANDLE WaitForClient (SOCKET_HANDLE listening_socket);

private:
	#ifdef __unix__
    pthread_mutex_t writeMutex; //provides write lock
	pthread_mutex_t readMutex; //provides read lock
	#endif
	#ifdef WIN32
		HANDLE writeMutex, readMutex;
	#endif
	
	void PrintNetworkError(char* err);
	int CloseSocket(SOCKET_HANDLE socket_handle);
	SOCKET_HANDLE CreateSocket(short nPort, struct hostent* lpHostEntry);
	int ConnectSocket(SOCKET_HANDLE socket);
};
#endif

