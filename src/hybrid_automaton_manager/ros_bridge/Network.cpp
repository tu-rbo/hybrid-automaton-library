#include "Network.h"

Network::Network(const char * serverIP, int serverPort, const char * clientIP, int clientPort,NetworkUpdatable *listener){
	_listener=listener;
	networkStatus=true; 
#ifdef _WIN32
	Qmutex = CreateMutex(0, FALSE, 0);
	if (!(Qmutex)){
		throw(std::string("Mutex was not created (Qmutex)"));
	}
	listenerMutex = CreateMutex(0, FALSE, 0);
	if (!(listenerMutex)){
		throw(std::string("Mutex was not created (listenerMutex)"));
	}
#else
	pthread_mutex_init(&(Qmutex), NULL);
	pthread_mutex_init(&(listenerMutex), NULL);
#endif

	server=new TCPserver(serverIP,serverPort);
	client=new TCPclient(clientIP,clientPort);
	server->waitForClient();
	runReadThread();
	//runWriteThread();
	//launch a thread that listens and store in the Q (with mutex)
	//launch a thread that reads from the Q and sends

	netName = std::string(clientIP);
}

Network::~Network(void){
	networkStatus=false;
	//printf("in network destruction code -0\n");
	//making sure that all the threads exited correctly
	for(int i=0;i<3;i++){
#ifdef _WIN32
		DWORD rc;
		if ((rc = WaitForSingleObject(Qmutex, INFINITE)) == WAIT_FAILED){
			std::cout<<"Cannot lock the controller process (Network::~Network)"<<std::endl;
		}
#else
		pthread_mutex_lock(&Qmutex);
#endif

#ifdef _WIN32
		if(!ReleaseMutex(Qmutex)){
			rc = GetLastError();
			std::cout<<"Cannot lock the controller process (Network::~Network)"<<std::endl;
		}
#else
		pthread_mutex_unlock(&Qmutex);
#endif
	}
	//printf("in network destruction code -1\n");
	delete(server);
	//printf("in network destruction code -2\n");
	delete(client);
	//printf("in network destruction code -3\n");

	//wait for TCP connection shutdown
#ifdef _WIN32
	Sleep(1000);
#else
	sleep(1);
#endif
}

bool Network::readSocket(std::string & s){
	int id,priority,type,size;
	char *data;
	if (server->readMessage(&data,&id,&priority,&type,&size))
		return false;
	s=std::string(data, size);
	return true;
}

bool Network::writeSocket(std::string & s){
	const char * data=s.c_str();
	int size=(int)s.size();
	if (!networkStatus) return false;
	if (client->send(1,2,3,size,data)) {
		printf("Network class is not able to send data\n");
		return false;
	}
	return true;
}

void Network::subscribeObject(rlab::NetworkData *obj){
	std::vector<rlab::NetworkData*>::iterator it;
	for(it = registeredObjects.begin(); it != registeredObjects.end(); it++){
		if((*it)->getType() == obj->getType()){
			throw std::string("type already registered"); 
		}
	}
	registeredObjects.push_back(obj);
}

void Network::runReadThread(){
	//creating network handling thread & mutex
#ifdef _WIN32
	//HANDLE hThread;
	DWORD dwGenericThread;
	HANDLE hThread=CreateThread(NULL,0,read,(void*)this,0,&dwGenericThread);
	if(hThread == NULL){
		DWORD dwError = GetLastError();
		std::cout<<"SCM:Error in Creating thread"<<dwError<<std::endl ;
		return;
	}
	WaitForSingleObject(hThread,0);//INFINITE);

#else
	pthread_t networkThread;
	int rc;
	rc=pthread_create(&networkThread,NULL,readThread,this);
	if(rc){
		printf("error %d in creating new read thread\n",rc);
		return;
	}
	pthread_detach(networkThread);
#endif
}

void Network::runWriteThread(){
	//creating network handling thread & mutex
#ifdef _WIN32
	//HANDLE hThread;
	DWORD dwGenericThread;
	HANDLE hThread=CreateThread(NULL,0,write,(void*)this,0,&dwGenericThread);
	if(hThread == NULL){
		DWORD dwError = GetLastError();
		std::cout<<"SCM:Error in Creating thread"<<dwError<<std::endl ;
		return;
	}
	WaitForSingleObject(hThread,0);//INFINITE);
#else
	pthread_t networkThread;
	int rc;
	//usleep(10);
	rc=pthread_create(&networkThread,NULL,writeThread,this);
	if(rc){
		printf("error %d in creating new write thread\n",rc);
		return;
	}
	pthread_detach(networkThread);
#endif
}


void Network::update(rlab::NetworkData* obj){
#ifdef _WIN32
	DWORD rc;
	if ((rc = WaitForSingleObject(listenerMutex, INFINITE)) == WAIT_FAILED){
		std::cout<<"Cannot lock the controller process (Network::update)"<<std::endl;
	}
#else
	pthread_mutex_lock(&listenerMutex);
#endif

	_listener->update(obj);

#ifdef _WIN32
	if(!ReleaseMutex(listenerMutex)){
		rc = GetLastError();
		std::cout<<"Cannot lock the controller process (Network::update)"<<std::endl;
	}
#else
	pthread_mutex_unlock(&listenerMutex);
#endif
}

