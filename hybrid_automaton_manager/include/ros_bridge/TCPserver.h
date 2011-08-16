/*
 *\brief TCPserver class
 *\detailed The TCPserver class implements communication over a network based on the TCP
 *          protocol for a server (unidirectional)
 *$Author: dubik $
 *$Date: 2007/04/17 17:31:29 $
 *$Revision: 1.2 $
 */

#ifndef __TCPSERVER__
#define __TCPSERVER__

#include "TCPcomm.h"
#include <string>
#include <stdio.h>
#include <iostream>
#include <vector>

class TCPserver{
  TCPcomm net;
  SOCKET_HANDLE server_socket;
  SOCKET_HANDLE client_socket;
public:
  TCPserver(const char * IP,int port); 	//c'tor
  ~TCPserver();			//d'tor
  void waitForClient();
  int readMessage(char ** data, int * id, int * priority, int * type, int * size);

private:
  int readByte(char * buffer);	//read 8 chars into buffer
};
#endif

