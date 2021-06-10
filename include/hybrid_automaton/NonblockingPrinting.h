#pragma once

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#include <process.h>
#else
#include <pthread.h>
#endif

#include <list>
#include <thread>
#include <chrono>

#define PRINTLVL_DEBUG 0
#define PRINTLVL_INFO 1
#define PRINTLVL_WARN 2
#define PRINTLVL_ERROR 3

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
			// the handle to the console (need this for colored output)
			HANDLE hConsole;
		#else
			pthread_t _printingThread;
		#endif

		bool _printingThreadActive;

	protected:
		std::list<std::pair<std::string, int> > _printQueue;

		printLock _printLock;
		

    public:
        static NonblockingPrinting& getInstance()
        {
            static NonblockingPrinting    instance; // Guaranteed to be destroyed.
                                  // Instantiated on first use.
            return instance;
        }

		void appendMessage(std::string msg_text, int msg_level = PRINTLVL_DEBUG){
			std::pair<std::string, int> msg (msg_text, msg_level);
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

		void createThread(void) {
			//creating network handling thread
			_printingThreadActive = true;
			#ifdef _WIN32
				DWORD dwGenericThread;
				_printingThread = CreateThread(NULL,0,printThread,(void*)this,0,&dwGenericThread);

			#else
				pthread_t _printThread;
				int rc;
				//usleep(10);
				rc=pthread_create(&_printThread,NULL,&printThread,this);
				if(rc){
					throw "Error in creating thread";
				}

				//Arne: I think we do not need this? - we call join in the destructor
				//pthread_detach(_networkThread);
			#endif
		}

    public:

		#ifdef _WIN32
			friend DWORD WINAPI printThread(void *obj)
		#else
			static void *printThread(void *obj)
		#endif
			{
				NonblockingPrinting* printer = (NonblockingPrinting*)obj;

				#ifdef _WIN32
					HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
				#endif

				while(true){
					// check for stuff to be printed ten times a second

					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					  
					while (!printer->_printQueue.empty())
					{
					    
						// get the message to be printed out of the print queue and lock the mutex for as short as possible
						printer->_printLock.lock();
						std::pair<std::string, int> msg = printer->_printQueue.front();
						printer->_printQueue.pop_front();
						printer->_printLock.unlock();

						// choose cerr or cout, depending on the print level
						std::ostream& stream = (msg.second == PRINTLVL_WARN || msg.second == PRINTLVL_ERROR)? std::cerr : std::cout;

						// set the color
						#ifdef _WIN32
						// text coloring on windows as according to
						// https://stackoverflow.com/questions/4053837/colorizing-text-in-the-console-with-c
						switch(msg.second){
							case PRINTLVL_DEBUG:
								break;
							case PRINTLVL_INFO:
								break;
							case PRINTLVL_WARN:
								SetConsoleTextAttribute(hConsole, 14);
								break;
							case PRINTLVL_ERROR:
								SetConsoleTextAttribute(hConsole, 12);
								break;
						}
						#else
							switch(msg.second){
								case PRINTLVL_DEBUG:
									break;
								case PRINTLVL_INFO:
									break;
								case PRINTLVL_WARN:
									stream<<"\033[33m";
									break;
								case PRINTLVL_ERROR:
									stream<<"\033[31m";
									break;
							}
						#endif

						// actually print the message
						stream  << msg.first <<"\n";
							
						
						// reset printing to the standard color
						#ifdef _WIN32
							SetConsoleTextAttribute(hConsole, 15);
						#else
							stream<<"\033[0m";
						#endif
					}
					// after going through the loop of messages, flush both streams once
					std::cerr<<std::flush;
					std::cout<<std::flush;
					
				}
			}


};

inline void safePrint(std::string msg_text, int msg_level = PRINTLVL_DEBUG){
	//const std::string red("\033[0;31m"); 
	//const std::string reset("\033[0m");
	NonblockingPrinting::getInstance().appendMessage(msg_text, msg_level);
}
