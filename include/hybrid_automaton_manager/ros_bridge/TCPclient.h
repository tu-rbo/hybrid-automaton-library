/*
 *\brief TCPclient class
 *\detailed The TCPclient class implements communication over a network based on the TCP
 *          protocol for a client (unidirectional)
 *$Author: dubik $
 *$Date: 2007/04/17 17:31:28 $
 *$Revision: 1.2 $
 */

#ifndef __TCPCLIENT__
#define __TCPCLIENT__

#include "TCPcomm.h"
#include <string>
#include <vector>
#include <iostream>

class TCPclient{
  TCPcomm net;
  SOCKET_HANDLE socket;
public:
  TCPclient(const char * IP,int port);	//c'tor 
  ~TCPclient();						//d'tor
  int send(int ID, int priority, int type, int size, const char * data);

private:
  int rawSend(char * msg, int size);       //sends a 8*size bytes message
  int isInRange( int val, int low, int high);

};
#endif

