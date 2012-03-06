/*
* RTBlackBoard.h
*
*  Created on: Mar 19, 2010
*      Author: clemens, dermax
*/

#ifndef RTBLACKBOARD_H_
#define RTBLACKBOARD_H_

//#include <cstdlib>
//#include <unistd.h>
//#include <math.h>
#include <map>
#include <exception>
#include <vector>

#include <Windows.h>
//#include <process.h>

#include "Network.h"
#include "msgs\NetworkData.h"
#include "NetworkUpdatable.h"

#include "rMath/rMath.h"

/*!
*  \brief  Thru the Blackboard all Parts of the HybridAutomaton communicate with each other and the ros cloud.
*/

class RTBlackBoard : public NetworkUpdatable {

private:
	bool usesNetwork;
	std::auto_ptr<Network> net;

	HANDLE writeToNetworkThreadHandle;
	HANDLE isAboutToBeDestructedEvent;

	/**
	* The map contains all topic value pairs.
	*
	*/
	typedef std::map< std::string, rlab::NetworkData* > DataMap;
	DataMap networkInputBuffer;
	DataMap inputMap;
	DataMap outputMap;

	HANDLE copyInputBufferMutex;
	HANDLE accessOutputMapMutex;

	/*
	* how often step() is called (not used)
	*/
	unsigned long ticks;
	unsigned int ros_publish_every_n_ticks;

	/*
	* is true when we have created an instance of the Blackboard
	*/
	static bool instanceFlag;
	/*
	* stores a pointer to the only blackboard that exists
	*/
	static RTBlackBoard *single;

	/**
	* it's a singleton
	*/
	RTBlackBoard();
	RTBlackBoard(const std::string& rlab_host, int rlab_port, const std::string& ros_host, int ros_port);

public:
	/*!
	* \brief Get the pointer to the only instance of the bb.
	* @return a pointer to the BlackBoard object
	*/
	static RTBlackBoard* getInstance();
	static RTBlackBoard* getInstance(const std::string& rlab_host, int rlab_port, const std::string& ros_host, int ros_port);

	~RTBlackBoard();
	/*!
	* \brief returns the ros::Message at topic topc in val.
	* no check if it is actually Tmsg in the bb
	* Tmsg is the type of message you want to get
	* Example <std_msgs::FLoat64>getRosMsg() is equilavant to get()
	*/
	rlab::NetworkData* getData(const std::string &topic);

	/*!\brief returns true if the topic is in the BlackBoard
	* @return "topic is in the bb"
	*/
	bool exists(const std::string& topic) const;
	bool isUpdated(const std::string& topic) const;

	int size();

	void subscribeToROSMessage(const std::string& topic);

	/*!
	* \brief set a value in the BlackBoard.
	* @param  topic   the topic to set
	* @param  value   the value
	*/
	void setFloat64(const std::string& topic, double value);
	void setString(const std::string& topic, const std::string& value);
	void setFloat64MultiArray(const std::string& topic, const std::vector<double>& value);
	void setJointState(const std::string& topic, const std::vector<double>& position, const std::vector<double>& velocity, const std::vector<double>& effort);
	void setTransform(const std::string& topic, rMath::HTransform& transform, const std::string& parent);

	/*!
	* \brief increment tick counter and starts the ros communication on every 10 call.
	*/
	void step();

	void update(rlab::NetworkData *s);

private:
	void update(rlab::NetworkData *s, DataMap& map);

	void setFloat64(const std::string& topic, double value, DataMap& map);
	void setString(const std::string& topic, const std::string& value, DataMap& map);
	void setFloat64MultiArray(const std::string& topic, const std::vector<double>& value, DataMap& map);
	void setJointState(const std::string& topic, const std::vector<double>& position, const std::vector<double>& velocity, const std::vector<double>& effort, DataMap& map);
	void setTransform(const std::string& topic, rMath::HTransform& transform, const std::string& parent, DataMap& map);


	friend unsigned __stdcall writeToNetwork(void *obj)
	{
		RTBlackBoard* black_board = static_cast<RTBlackBoard*>(obj);

		while (black_board->outputMap.empty()) {
			Sleep(200);
		}

		DataMap::iterator it = black_board->outputMap.begin();
		
		// write it infinitely to the network
		while (WaitForSingleObject(black_board->isAboutToBeDestructedEvent, 0) != WAIT_OBJECT_0) {
			// copy the stuff from the output buffer
			// TODO: use separate mutexes for every data item
			DWORD rc = WaitForSingleObject(black_board->accessOutputMapMutex, 0);
			if (rc == WAIT_FAILED) {
				std::cerr << "Cannot lock accessOutputMapMutex (RTBlackBoard::writeToNetwork)" << std::endl;
			}

			if (rc != WAIT_TIMEOUT)
				(*(black_board->net)) << *(it->second);

			if (!ReleaseMutex(black_board->accessOutputMapMutex)) {
				std::cerr << "Cannot release accessOutputMapMutex (RTBlackBoard::writeToNetwork)" << std::endl;
			}

			Sleep(20);

			if ((++it) == black_board->outputMap.end()) {
				it = black_board->outputMap.begin();
			}
		}

		//std::cerr << "Main Thread is about to be destructed!" << std::endl;

		_endthreadex(0);
		return 0;
	}
};

#endif /* BLACKBOARD_H_ */
