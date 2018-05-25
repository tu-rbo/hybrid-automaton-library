#pragma once

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#include <process.h>
#else
#include <pthread.h>
#endif

#include <list>

class printLock
{

public:

	printLock(void)
	{
#ifndef __GNUC__
        _mutex = CreateMutex(NULL, FALSE, NULL);
#else
        pthread_mutex_init(&_mutex, NULL);
#endif
	}

	~printLock(void)
	{
#ifndef __GNUC__
        CloseHandle(_mutex);
#else
        pthread_mutex_destroy(&_mutex);
#endif
	}

	void lock()
	{
#ifndef __GNUC__
        WaitForSingleObject(_mutex, INFINITE);
#else
        pthread_mutex_lock(&_mutex);
#endif
	}

	void unlock()
	{
#ifndef __GNUC__
        ReleaseMutex(_mutex);
#else
        pthread_mutex_unlock(&_mutex);
#endif
	}
		
private:
#ifndef __GNUC__
    HANDLE _mutex;
#else
    pthread_mutex_t _mutex;
#endif

};

class NonblockingPrinting
{
	private:
	    //The printing thread
		#ifdef _WIN32
			HANDLE _printingThread;
		#else
			pthread_t _printingThread;
		#endif

		bool _printingThreadActive;

	protected:
		std::list<std::string> _printQueue;
		printLock _printLock;
		

    public:
        static NonblockingPrinting& getInstance()
        {
            static NonblockingPrinting    instance; // Guaranteed to be destroyed.
                                  // Instantiated on first use.
            return instance;
        }

		void appendMessage(std::string msg){
			_printLock.lock();
			_printQueue.push_back(msg);
			_printLock.unlock();
		}
    private:
		NonblockingPrinting(): _printingThreadActive(false) {
			createThread();
		}                   

        NonblockingPrinting(NonblockingPrinting const&);              
        void operator=(NonblockingPrinting const&); 

		void NonblockingPrinting::createThread(void) {
			//creating network handling thread
			_printingThreadActive = true;
			#ifdef _WIN32
				DWORD dwGenericThread;
				_printingThread = CreateThread(NULL,0,printThread,(void*)this,0,&dwGenericThread);

			#else
				pthread_t _printThread;
				int rc;
				//usleep(10);
				rc=pthread_create(&_printThread,NULL,printThread,this);
				if(rc){
					throw Network2Exception("[Network2::createThread] Error in creating thread");
				}

				//Arne: I think we do not need this? - we call join in the destructor
				//pthread_detach(_networkThread);
			#endif
		}

    public:

		#ifdef _WIN32
			friend DWORD WINAPI printThread(void *obj)
		#else
			friend void *printThread(void *obj)
		#endif
			{
				NonblockingPrinting* printer = (NonblockingPrinting*)obj;
				while(true){
					Sleep(100);
					if(printer->_printQueue.size()){
					  
					  while (!printer->_printQueue.empty())
					  {
					    
						printer->_printLock.lock();
						std::string str = printer->_printQueue.front();
						printer->_printQueue.pop_front();
						printer->_printLock.unlock();
						std::cout  << str <<"\n";
						
					  }
					  std::cout<<std::flush;
					}
				}
			}


};

inline void safePrint(std::string msg){
	//const std::string red("\033[0;31m"); 
	//const std::string reset("\033[0m");
	NonblockingPrinting::getInstance().appendMessage(msg);
}
