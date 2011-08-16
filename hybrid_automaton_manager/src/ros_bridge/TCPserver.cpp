#include "ros_bridge\TCPserver.h"
using namespace std;

TCPserver::TCPserver(const char * IP,int port){
    server_socket=NULL;

	#ifdef WIN32
	WORD wVersionRequested;
    WSADATA wsaData;

      wVersionRequested = MAKEWORD(2, 0);              // Request WinSock v2.0
      if (WSAStartup(wVersionRequested, &wsaData) != 0) {  // Load WinSock DLL
        cout << "Unable to load WinSock DLL" << endl;
      }
	#endif

    server_socket=net.OpenListeningSocket(IP,port);
    printf("created listening socket %d\n",server_socket->id);
  }


  TCPserver::~TCPserver(){
     net.CloseConnection(server_socket);
    // net.CloseConnection(client_socket);
   }

  void TCPserver::waitForClient(){
    client_socket=net.WaitForClient(server_socket);
    printf("Got a connection to local socket %d\n",client_socket->id);
  }
  int TCPserver::readByte(char * buffer){
    if(client_socket==NULL || server_socket==NULL) return 1;
    if(net.RecvData(client_socket,buffer,8)!=-6){
	return 0;
    }else{
	return 1;
    }
  }

  int TCPserver::readMessage(char ** data, int * id, int * priority, int * type, int * size){
    char header[5];

    //verify connectivity
    if(client_socket==NULL || server_socket==NULL) return 1;

    //verify null data pointer
    //if(data!=NULL) delete(data);

    //read header
    int i=0;
    while(i<5){
      if(net.RecvData(client_socket,&header[i],1)!=-6){
        i++;
      }
      else {
	client_socket = NULL;
	printf("TCPserver::client_socket is closed\n");
	return 1;
      }
    }

    //decipher header
    (*id)=(int)header[0];
    (*priority)=(int)(header[1]&0x0f);
    (*type)=(int)(header[2]&0x3f);
    (*type)=((*type)<<4);
    (*type)+=(int)((header[1]&0xf0)>>4);
	(*size) = ((int)(header[4]))<<10;
	(*size) += (((int)(header[3]))&0xff)<<2;
	(*size) += (int)((header[2]&0xc0)>>6);
	
    //read data
//cout<<"DEBUG size = "<<(*size)<<endl;
    *data=new char[*size];
    if(net.RecvData(client_socket,*data,(*size))==-6){
      client_socket = NULL; 
      printf("TCPserver::client_socket is closed\n");
      return 1;
    }
    return 0;
 }

