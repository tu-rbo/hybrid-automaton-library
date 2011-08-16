#include "ros_bridge\TCPcomm.h"
using namespace std;

#ifdef WIN32
#define ENOTSOCK WSAENOTSOCK
#define EADDRNOTAVAIL WSAEADDRNOTAVAIL
#define EADDRINUSE WSAEADDRINUSE
#define close(socket) closesocket(socket);
typedef int socklen_t; 
#endif

TCPcomm::TCPcomm(){
  #ifdef __unix__
	pthread_mutex_init(&writeMutex, NULL);
	pthread_mutex_init(&readMutex, NULL);
  #endif

  #ifdef WIN32
  writeMutex = CreateMutex(0, FALSE, 0);
  if (!(writeMutex))
  {
	  std::cout << "Could not create write mutex\n";
  }
  readMutex = CreateMutex(0, FALSE, 0);
  if (!(readMutex))
  {
	  std::cout << "Could not create read mutex\n";
  }
  #endif
}

SOCKET_HANDLE TCPcomm::OpenTCPConnection(const char *ServerName, short nPort){
	int ret=0;    //stores return values
	SOCKET_HANDLE socket;
	struct hostent* lpHostEntry;

	ret = GetServerIP(ServerName, &lpHostEntry);
	if(ret) return NULL;
	socket = CreateSocket(nPort, lpHostEntry);	
	if(socket == NULL) return socket;
	
	ret = ConnectSocket(socket);
	while(ret){
	  #ifdef __QNX__
	  // close socket and starting from the beginning
	  //printf("ConnectSocket error code %d\n",errno);
	  CloseSocket(socket);
	  CreateSocket(nPort, lpHostEntry);
	  ret = ConnectSocket(socket);
	  #else
		ret=ConnectSocket(socket);
	  #endif

	  #ifdef WIN32
		Sleep(10);
	  #else
		usleep(10000);
	  #endif
	}
	return socket;
}

int TCPcomm::SendData( SOCKET_HANDLE socket,const char* data, int data_length){
	int res=0;

	#ifdef __unix__
   	pthread_mutex_lock(&writeMutex);
	#endif
	#ifdef WIN32
	DWORD rc;
	if ((rc = WaitForSingleObject(writeMutex, INFINITE)) == WAIT_FAILED)
	{
		std::cout<<"Cannot lock the controller process (TCPcomm::SendData)"<<std::endl;
	}
	#endif

	res = send(socket->id, data, data_length, 0);

	#ifdef __unix__
	pthread_mutex_unlock(&writeMutex);
	#endif
	#ifdef WIN32
	if (! ReleaseMutex(writeMutex))
	{
		rc = GetLastError();
		//cout << "Cannot unlock the controller process\n";
	}
	#endif
	

	if( res != data_length){		
		if(res == -1)
			PrintNetworkError("Error in sending data");
		else
			PrintNetworkError("Error while sending data to the server, not all bytes were sent\n");
		CloseSocket(socket);
		return -6;
	}
	return res;
}

int TCPcomm::RecvData( SOCKET_HANDLE socket, char* buffer, int buffer_size){
  int res=0;
  int total_bytes=0, bytes_left=buffer_size;
 
  while(total_bytes < buffer_size){
    #ifdef __unix__
    pthread_mutex_lock(&readMutex);
    #endif
   
    #ifdef WIN32
    DWORD rc;
    if((rc = WaitForSingleObject(readMutex, INFINITE)) == WAIT_FAILED){}
    #endif

    res = recv(socket->id, buffer, bytes_left, 0);

    #ifdef __unix__
    pthread_mutex_unlock(&readMutex);
    #endif
  
    #ifdef WIN32
    if(!ReleaseMutex(readMutex)){
      rc = GetLastError();
    }
    #endif
		
    if(res == -1){
      //PrintNetworkError("RecvData: Error while receiving data from the server\n");
      CloseSocket(socket);
      return -6;
    }
		
    if(res == 0){
      //PrintNetworkError("RecvData: No data can be recieved from server. Connection has been gracefully closed\n");
      CloseSocket(socket);
      return -6;
    }

    total_bytes += res;
    buffer += res;
    bytes_left -= res;
  }
  return total_bytes;
}


int TCPcomm::CloseConnection(SOCKET_HANDLE socket_handle){
	int ret=0;
	char errorInfo[1024];
	int socket = socket_handle->id;

	ret = CloseSocket(socket_handle);	
	if(ret == -1)
	{
		#ifdef _WIN32
		sprintf_s(errorInfo,"Could not close socket %d\n",socket); // safer version sprintf_s
		#else
		sprintf_s(errorInfo,"Could not close socket %d\n",socket);
		#endif
		PrintNetworkError(errorInfo);
		return ret;
	}
	return ret;
}

SOCKET_HANDLE TCPcomm::OpenListeningSocket (const char *ServerHostname, int ServerPortNo){
	int len;
	SOCKET_HANDLE listening_socket;
	struct hostent *hp;
	int status;
	char errorInfo[1024];
	
	status = GetServerIP(ServerHostname, &hp);
	if(status){
		#ifdef _WIN32
		sprintf_s(errorInfo,"OpenListeningSocket: Cannot get server IP for %s!\n", ServerHostname); // safer version sprintf_s
                #else
		sprintf_s(errorInfo,"OpenListeningSocket: Cannot get server IP for %s!\n", ServerHostname);
                #endif
	        PrintNetworkError(errorInfo);
		return NULL;
	}
	// Create socket that will accept connections
	listening_socket = CreateSocket((short) ServerPortNo, hp);
		
	if (listening_socket == NULL){
		#ifdef _WIN32
		sprintf_s(errorInfo,"OpenListeningSocket: Cannot create socket!\n"); // safer version sprintf_s
                #else
		sprintf_s(errorInfo,"OpenListeningSocket: Cannot create socket!\n");
                #endif
		PrintNetworkError(errorInfo);
	  return NULL;
	}
	
		len = sizeof (struct sockaddr_in);
		status = bind (listening_socket->id, (struct sockaddr *) (&(listening_socket->saServer)), len);	
  
	if (status < 0){	
		#ifdef _WIN32
		sprintf_s(errorInfo,"OpenListeningSocket: Cannot bind to server port, the error is: \n");  // safer version sprintf_s
                #else
		sprintf_s(errorInfo,"OpenListeningSocket: Cannot bind to server port, the error is: \n");
                #endif
		PrintNetworkError(errorInfo);
		switch (errno){
		  case EBADF:
			#ifdef _WIN32
			sprintf_s(errorInfo,"EBADF\n");  // safer version sprintf_s
			#else
			sprintf_s(errorInfo,"EBADF\n");
			#endif
		    break;
		  case ENOTSOCK:
			#ifdef _WIN32
			sprintf_s(errorInfo,"ENOTSOCK\n"); // safer version sprintf_s
			#else
			sprintf_s(errorInfo,"ENOTSOCK\n");
			#endif
		    break;
		  case EADDRNOTAVAIL:
			#ifdef _WIN32
			sprintf_s(errorInfo,"EADDRNOTAVAIL\n"); // safer version sprintf_s
                        #else
			sprintf_s(errorInfo,"EADDRNOTAVAIL\n");
                        #endif
		    break;
		  case EADDRINUSE:
			#ifdef _WIN32
			sprintf_s(errorInfo,"EADDRINUSE\n"); // safer version sprintf_s
                        #else
			sprintf_s(errorInfo,"EADDRINUSE\n");
                        #endif
		    break;
		  case EINVAL:
			#ifdef _WIN32
			sprintf_s(errorInfo,"EINVAL\n"); // safer version sprintf_s
                        #else
			sprintf_s(errorInfo,"EINVAL\n");
                        #endif
		    break;
		  case EACCES:
			#ifdef _WIN32
			sprintf_s(errorInfo,"EACCES\n"); // safer version sprintf_s
                        #else
			sprintf_s(errorInfo,"EACCES\n");
                        #endif
		    break;
		  case EFAULT:
			#ifdef _WIN32
			sprintf_s(errorInfo,"EFAULT\n"); // safer version sprintf_s
                        #else
			sprintf_s(errorInfo,"EFAULT\n");
                        #endif
		    break;
		  default:
		    break;
		  }
		PrintNetworkError(errorInfo);
		CloseSocket(listening_socket);
		return NULL;
	}

	// Specify a willingness to accept up to 1 connection on the socket
	status = listen(listening_socket->id, 1);
	
	if (status < 0){
		#ifdef _WIN32
		sprintf_s(errorInfo,"OpenListeningSocket: Cannot listen on port!\n"); // safer version sprintf_s
		#else
		sprintf_s(errorInfo,"OpenListeningSocket: Cannot listen on port!\n");
		#endif
		PrintNetworkError(errorInfo);
		CloseSocket(listening_socket);
		return NULL;
	}
	
	return listening_socket;	    
}

/*
 * Receives a listening socket and block waits for client connection on it
 * Should get the same sockaddr_in *ServerAddr  that was used when creating the listening socket
 */
SOCKET_HANDLE TCPcomm::WaitForClient (SOCKET_HANDLE listening_socket){
	socklen_t len;
	SOCKET_HANDLE accepted_socket;
	char errorInfo[1024];

	accepted_socket = new SOCKET_HANDLE_STRUCT;	
	if(accepted_socket == NULL){
		#ifdef _WIN32
	 	sprintf_s(errorInfo,"WaitForClient: err allocating accepted_socket handle\n"); // safer version sprintf_s
		#else
		sprintf_s(errorInfo,"WaitForClient: err allocating accepted_socket handle\n");
		#endif
		PrintNetworkError(errorInfo);
		return NULL;
	}

	//Wait for client to connect: accept call blocks until an attempt to connect
	len=sizeof(struct sockaddr_in);
	accepted_socket->id=(SOCKET_DESC)accept(listening_socket->id,(struct sockaddr*)(&(listening_socket->saServer)),&len);

	if(accepted_socket->id < 0){
		#ifdef _WIN32
		sprintf_s(errorInfo,"WaitForClient: err when accepting client connection\n"); // safer version sprintf_s
		#else
		sprintf_s(errorInfo,"WaitForClient: err when accepting client connection\n");
		#endif
		PrintNetworkError(errorInfo);
		CloseSocket(accepted_socket);
		return NULL;
	}else{
		return accepted_socket;
	}
}
// ----------------------------------INNER FUNCTIONS--------------------------------
/* 
 * Prints the given error message to stdout
 */
void TCPcomm::PrintNetworkError(char* err)
{
	printf(err);
	printf("errno[%d]:%s",errno,strerror(errno));	
}


/* 
 * Closes the given socket.
 * Returns 0 on success, SOCKET_ERROR on error
 */
int TCPcomm::CloseSocket(SOCKET_HANDLE socket_handle)
{
    int socket = socket_handle->id;
    return close(socket);//returns 0 on success, -1 on failure

}


/* 
 * Lookup server ip from its name, put it in lpHostEntry
 * Return: 0 on success, -2 on failure
 */

int TCPcomm::GetServerIP(const char *ServerName, struct hostent** lpHostEntry){
	*lpHostEntry = gethostbyname((char *)(std::string(ServerName).c_str()));
    if (*lpHostEntry == NULL)
    {	
        PrintNetworkError("Could not retrieve server ip. Error in gethostbyname()");
        return -2;
    }else{
		return 0;
	}
}


/* 
 * Creates a socket using the given port
 * Return: socket identifier on success, NULL on failure
 */
SOCKET_HANDLE TCPcomm::CreateSocket(short nPort, struct hostent* lpHostEntry){
  SOCKET_HANDLE theSocket;
  struct sockaddr_in* saServer;

  theSocket=new SOCKET_HANDLE_STRUCT;
  if(theSocket==NULL){
    return NULL;
  }
  theSocket->id = 0;
  saServer = &(theSocket->saServer);

  //Create a TCP/IP stream socket
  theSocket->id = (SOCKET_DESC)socket(AF_INET,SOCK_STREAM,0);
  //#ifdef __QNX__
  int opt = 1;
  if (setsockopt (theSocket->id, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof (opt)) < 0) {
    printf("setting socket option failed in TCPclient\n");
  }
  //#endif

  if(theSocket->id == -1){
    PrintNetworkError("CreateSocket: cannot create socket. Error in socket()");
    free(theSocket);
    return NULL;
  }

  //Fill in the address structure
  saServer->sin_family=AF_INET;
  saServer->sin_addr=*((struct in_addr*)(lpHostEntry->h_addr));//Server address
  saServer->sin_port   = htons(nPort);	// Port number
  return theSocket;
}

/* 
 * Opens a connection to a server using theSocket
 * Return: 0 on success, -4 on failure
 */
int TCPcomm::ConnectSocket(SOCKET_HANDLE socket){
	struct sockaddr_in* saServer = &(socket->saServer);		
	int nRet = connect(socket->id,(struct sockaddr *)saServer,sizeof(struct sockaddr));
	if (nRet == -1){
	  //cout << "error code: " << errno << " len: " << sizeof(*saServer) << " vs " << sizeof(struct sockaddr) << endl;
	  return -4;
	}
	return 0;
}

