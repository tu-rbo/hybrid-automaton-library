/*
 *\brief Network class
 *\detailed The Network class is responsible for managing the communication over the network
 *          in a bi-directional fashion
 *$Author: dubik $
 *$Date: 2008/01/12 07:37:52 $
 *$Revision: 1.13 $
 */

#ifndef __NETWORK__
#define __NETWORK__

#include <string>
#include <queue>
#include <iostream>
#include "TCPserver.h"
#include "TCPclient.h"

#include "NetworkData.h"

#include "NetworkUpdatable.h"

#ifdef _WIN32
#include <Windows.h>
#include <process.h>
#else
#include <pthread.h>
#endif

extern "C" void* writeThread(void *obj);
extern "C" void* readThread(void *obj);

class Network{
private:
	std::string netName;
	TCPserver *server;
	TCPclient *client;
	NetworkUpdatable *_listener;
	bool networkStatus;

#ifdef _WIN32
	HANDLE Qmutex;
	HANDLE listenerMutex;
#else
	pthread_mutex_t Qmutex;
	pthread_mutex_t listenerMutex;
#endif

	std::queue<std::string> msgQ;
	std::vector<rlab::NetworkData*> registeredObjects;

	template<class T>
	std::string serialize(T & t){
		std::string s; 
		//t>>s;
		s = t.serialize();
		return s;
	}

	template<class T>
	inline T & deserialize(std::string & s){
		T *t=new T();
		(*t)<<s;
		return *t;  
	}

	bool readSocket(std::string & s);	 //reads a string from the socket
	bool writeSocket(std::string & s);  //sends a string to the socket
	void runReadThread();				//runs in a thread, listens to incoming traffic and adds it to the queue
	void runWriteThread();
	void parse(void * updateableObject); //parse and send update to the interested object
	void update(rlab::NetworkData *obj);

public:
	Network(const char * serverIP, int serverPort, const char * clientIP, int clientPort, NetworkUpdatable *listener);
	~Network(void);

	std::string getNetworkName() {return netName;}
	void setNetworkName(std::string name) {netName = name;}

	void subscribeObject(rlab::NetworkData *obj);

	template<class T>
	friend Network& operator << (Network& net, T& t){
		std::string s;
		s=net.serialize(t);
		net.writeSocket(s);
#ifndef _WIN32__ // wierd compilation error under windows; CE: i introduced it again (originally was _WIN32), because msvc required a return value
		return net;
#endif
	}

	template<class T>
	friend Network& operator >> (Network& net, T& t){
		std::string s;
		if(!net.msgQ.empty()){
#ifdef _WIN32
			DWORD rc;
			if ((rc = WaitForSingleObject(net.Qmutex, INFINITE)) == WAIT_FAILED){
				std::cout<<"Cannot lock the controller process (Network::>>)"<<std::endl;
			}
#else
			pthread_mutex_lock(&(net.Qmutex));
#endif

			s=net.msgQ.front();
			net.msgQ.pop();

#ifdef _WIN32
			if(!ReleaseMutex(net.Qmutex)){
				rc = GetLastError();
				std::cout<<"Cannot lock the controller process (Network::>>)"<<std::endl; 
			}
#else
			pthread_mutex_unlock(&(net.Qmutex));
#endif
			t=net.deserialize <T> (s); //try for each registered component, if exception caught proceed to next 
		}else{
			throw std::string("no data in network queue");
		}		
		return net;
	}

#ifdef _WIN32
	friend DWORD WINAPI read(void *obj){ 
#else
		friend void *readThread(void *obj){
#endif
#ifdef __QNX__
			sched_param schedparam;
			int policy = SCHED_RR;
			pthread_getschedparam(0, &policy, &schedparam);
			schedparam.sched_priority = 10; // make it consistent with network driver
			pthread_setschedparam(0, policy, &schedparam);
			//printf("current read thread priority: %d\n", schedparam.sched_priority);
#endif

			Network * net=(Network*)obj;
			std::string msg;

			while(net->networkStatus){
				if (!net->readSocket(msg)) {
					printf("Network is down\n");
					net->networkStatus = false;
					break;
				}

#ifdef _WIN32
				DWORD rc;
				if ((rc = WaitForSingleObject(net->Qmutex, INFINITE)) == WAIT_FAILED){
					std::cout<<"Cannot lock the controller process (Network::read)"<<std::endl;
					DWORD dw = GetLastError();
					std::cout<<"details: "<<dw<<std::endl;
				}
#else
				pthread_mutex_lock(&net->Qmutex);
#endif

				net->msgQ.push(msg);

#ifdef _WIN32
				if(!ReleaseMutex(net->Qmutex)){
					rc = GetLastError();
					std::cout<<"Cannot lock the controller process (Network::read)"<<std::endl;
				}
#else
				pthread_mutex_unlock(&net->Qmutex);
#endif

				net->runWriteThread();

			}
			printf("Existing Network readThread\n");
#ifdef WIN32
			_endthread();
#else
			pthread_exit(0);
#endif
			return NULL;
		}

#ifdef _WIN32
		friend DWORD WINAPI write(void *obj){
#else
			friend void *writeThread(void *obj){
#endif
#ifdef __QNX__
				sched_param schedparam;
				int policy = SCHED_RR;
				pthread_getschedparam(0, &policy, &schedparam);
				schedparam.sched_priority = 10; // make it consistent with network driver
				pthread_setschedparam(0, policy, &schedparam);
				//printf("current write thread priority: %d\n", schedparam.sched_priority);
#endif

				Network * net=(Network*)obj;
				std::string msg;

#ifdef _WIN32
				DWORD rc;
				if ((rc = WaitForSingleObject(net->Qmutex, INFINITE)) == WAIT_FAILED){
					std::cout<<"Cannot lock the controller process (Network::write)"<<std::endl;
				}
#else
				pthread_mutex_lock(&net->Qmutex);
#endif

				bool match=false;

				if(net->networkStatus && !(net->msgQ.empty())){

					msg=net->msgQ.front();
					std::vector<rlab::NetworkData*>::iterator it;
					for(it=net->registeredObjects.begin();it!=net->registeredObjects.end();it++){
//						std::cout << msg << std::endl;
						if((*it)->canDeserialize(msg)){
							//found match => release lock on Queue
							net->msgQ.pop();

#ifdef _WIN32
							if(!ReleaseMutex(net->Qmutex)){
								rc = GetLastError();
								std::cout<<"Cannot lock the controller process (Network::write)"<<std::endl;
							}
#else
							pthread_mutex_unlock(&net->Qmutex);
#endif

							//deserialize and call update
							net->update((*it)->deserialize(msg));
							match=true;
						}
						if(match) break;
					}
					if(!match){
						//release the mutex (no parser found)
						std::cout<<"In write thread: rcvd an object that cannot be parsed!"<<std::endl;
#ifdef _WIN32
						if(!ReleaseMutex(net->Qmutex)){
							rc = GetLastError();
							std::cout<<"Cannot lock the controller process (Network::write)"<<std::endl;
						}
#else
						pthread_mutex_unlock(&net->Qmutex);
#endif
					}
				}else{
					//no msg to handle => release lock on Queue
#ifdef _WIN32
					if(!ReleaseMutex(net->Qmutex)){
						rc = GetLastError();
						std::cout<<"Cannot lock the controller process (Network::write)"<<std::endl;
					}
#else
					pthread_mutex_unlock(&net->Qmutex);
#endif
				}

				//return (thread dies)
#ifdef WIN32
				_endthread();
#else
				pthread_exit(0);
#endif
				return NULL;
			}

		};

#endif

