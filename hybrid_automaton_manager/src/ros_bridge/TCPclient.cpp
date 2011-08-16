#include "TCPclient.h"
using namespace std;

 TCPclient::TCPclient(const char * IP,int port){ 
   socket=NULL;
 
  #ifdef WIN32
	WORD wVersionRequested;
      WSADATA wsaData;

      wVersionRequested = MAKEWORD(2, 0);              // Request WinSock v2.0
      if (WSAStartup(wVersionRequested, &wsaData) != 0) {  // Load WinSock DLL
        cout << "Unable to load WinSock DLL" << endl;
      }
  #endif
    
  socket=net.OpenTCPConnection(IP,port);
  if(socket==NULL){
	  printf("error in opening socket\n");
  }
 }

  TCPclient::~TCPclient(){
	  net.CloseConnection(socket);
  }

  int TCPclient::rawSend(char * msg, int size){
    if(socket==NULL) return 1;
    if (net.SendData(socket,msg,size)==-6) {
      socket = NULL;
      printf("TCPclient::rawSend socket is closed\n");
      return 1;
    }
    return 0;
  }
  
  int TCPclient::isInRange( int val, int low, int high){
    if(val<=high && val>=low) return 1;
    return 0; 
  }

int TCPclient::send(int ID, int priority, int type, int size, const char * data){
    if(socket==NULL) return 1;

    //test for message correctness
    if(!(isInRange(ID,0,255) && isInRange(priority,0,15) && isInRange(type,0,1023) && isInRange(size,0,262143))) return 1;

    //allocate a message
    char * message=new char[size+5];

    //create message header
    message[0]=(char)ID;
    message[1]=(char)(priority+((type&0x0000000f)<<4));
    message[2]=(char)(((type&0x000003f0)>>4)+((size&0x00000003)<<6));
    message[3]=(char)(size>>2);
    message[4]=(char)(size>>10);

    //concentate header and data
    for(int i=0;i<(int)size;i++){
      message[i+5]=data[i];
    }

    if (rawSend(message,size+5)) {
      delete(message);
      return 1;
    }

    delete(message);

    return 0;
  }
