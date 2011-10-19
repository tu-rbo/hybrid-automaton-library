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


/*!
*  \brief  Thru the Blackboard all Parts of the HybridAutomaton communicate with each other and the ros cloud.
*/

class RTBlackBoard : public NetworkUpdatable {

private:
	Network net;

	/**
	* The map contains all topic value pairs.
	*
	*/
	typedef std::map< std::string, rlab::NetworkData* > DataMap;
	DataMap inputBuffer;
	DataMap inputMap;
	DataMap outputMap;

	HANDLE copyInputBufferMutex;

	/*
	* how often step() is called (not used)
	*/
	unsigned long ticks;
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
	~RTBlackBoard();

	bool ros_thread_finished;
public:
	/*!
	* \brief Get the pointer to the only instance of the bb.
	* @return a pointer to the BlackBoard object
	*/
	static RTBlackBoard* getInstance();
	static RTBlackBoard* getInstance(const std::string& rlab_host, int rlab_port, const std::string& ros_host, int ros_port);

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
	* \brief set a double value in the BlackBoard
	*/
	void setFloat64(const std::string &topic, double val);

	/*!
	* \brief set a value in the BlackBoard.
	* @param  topic   the topic to set
	* @param  val    the value
	*/
	void setFloat64MultiArray(const std::string& topic, const std::vector<double>& val);

	void setJointState(const std::string& topic, const std::vector<double>& position, const std::vector<double>& velocity, const std::vector<double>& effort);

	/*!
	* \brief increment tick counter and starts the ros communication on every 10 call.
	*/
	void step();

	void update(rlab::NetworkData *s);

private:
	void update(rlab::NetworkData *s, DataMap& map);

	void setFloat64(const std::string& topic, double val, DataMap& map);
	void setFloat64MultiArray(const std::string& topic, const std::vector<double>& val, DataMap& map);
	void setJointState(const std::string& topic, const std::vector<double>& position, const std::vector<double>& velocity, const std::vector<double>& effort, DataMap& map);

	//template <typename T1, typename T2>
	//void set(const std::string& topic, T& value, DataMap& map) {
	//	DataMap::iterator iter = map.find(topic);
	//	if(iter == map.end()) {
	//		// the topic is empty so we need to create a new object and copy the data
	//		map[topic] = new rlab::Float64(topic, val);
	//	}
	//	else {
	//	// copy the new data
	//	rlab::Float64* data = static_cast<rlab::Float64*>(iter->second);

	//	if(data)
	//		data->set(val);
	//	else
	//		throw "topic has different type than expected";
	//}

};

#endif /* BLACKBOARD_H_ */
