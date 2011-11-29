#include "RTBlackBoard.h"

//#include <iostream>
//#include <cstdlib>
//#include <unistd.h>
//#include <stdlib.h>
//#include <stdio.h>
//#include <math.h>
#include <errno.h>

#include "msgs\Float64.h"
#include "msgs\Float64MultiArray.h"
#include "msgs\String.h"
#include "msgs\JointState.h"
#include "msgs\RequestROSSubscription.h"

/*
* for the singleton stuff
*/
bool RTBlackBoard::instanceFlag = false;
RTBlackBoard* RTBlackBoard::single = NULL;
/*
* static variables
*/
//pthread_cond_t  BlackBoard::ros_go = PTHREAD_COND_INITIALIZER;
//pthread_mutex_t  BlackBoard::ros_signal_mutex = PTHREAD_MUTEX_INITIALIZER;
//pthread_mutex_t  BlackBoard::ros_bb_mutex = PTHREAD_MUTEX_INITIALIZER;

/*
* returns the one and only blackboard
*/
RTBlackBoard* RTBlackBoard::getInstance()
{
	if(! instanceFlag)
	{
		single = new RTBlackBoard();
		instanceFlag = true;
		return single;
	}
	else
	{
		return single;
	}
}

RTBlackBoard* RTBlackBoard::getInstance(const std::string& rlab_host, int rlab_port, const std::string& ros_host, int ros_port)
{
	if(! instanceFlag)
	{
		single = new RTBlackBoard(rlab_host, rlab_port, ros_host, ros_port);
		instanceFlag = true;
		return single;
	}
	else
	{
		return single;
	}
}

/*
* constructor
*/
RTBlackBoard::RTBlackBoard() :
	net("130.149.238.179", 1888, "130.149.238.184", 1999, this)
{
	// TODO: Order matters => because of string comparision(!)
	net.subscribeObject(new rlab::Float64MultiArray());
	net.subscribeObject(new rlab::Float64());
	net.subscribeObject(new rlab::String());
	net.subscribeObject(new rlab::JointState());

	// initialize mutex to copy input buffer
	copyInputBufferMutex = CreateMutex(0, FALSE, 0);
	if( !copyInputBufferMutex ) {
		throw(std::string("Mutex was not created (copyInputBufferMutex)"));
	}

	// set ticks to zero
	this->ticks = 0;
	this->ros_publish_every_n_ticks = 10;
}

RTBlackBoard::RTBlackBoard(const std::string& rlab_host, int rlab_port, const std::string& ros_host, int ros_port) :
net(rlab_host.c_str(), rlab_port, ros_host.c_str(), ros_port, this)
{
	// TODO: Order matters => because of string comparision(!)
	net.subscribeObject(new rlab::Float64MultiArray());
	net.subscribeObject(new rlab::Float64());
	net.subscribeObject(new rlab::String());
	net.subscribeObject(new rlab::JointState());

	// initialize mutex to copy input buffer
	copyInputBufferMutex = CreateMutex(0, FALSE, 0);
	if( !copyInputBufferMutex ) {
		throw(std::string("Mutex was not created (copyInputBufferMutex)"));
	}

	// set ticks to zero
	this->ticks = 0;
	this->ros_publish_every_n_ticks = 10;
}

/*
* Destructor
*/
RTBlackBoard::~RTBlackBoard()
{
	// go thru all data in the datamap and deletes the ros::Messages
	//pthread_cond_signal(&ros_go);
	/*for (Tdatamap::iterator it=this->datamap.begin() ; it != this->datamap.end(); it++ )
	{
		ros::Message *msg = (*it).second;
		if(msg!=NULL) delete msg;
	}*/

	// clear the instance flag
	instanceFlag = false;
	//pthread_join(thread_handle_ros, NULL);
}

//double RTBlackBoard::get(const std::string &topic)
//{
//  // check if topic is in the map
//  //if(datamap.find(topic)==datamap.end())
//  Tdatamap::iterator iter=datamap.find(topic);
//  if(iter==datamap.end())
//  {
//    throw "topic not in map";
//  }
//  // get the message out of the map, no check if it is the right message type
//  //std_msgs::Float64 *msg = (std_msgs::Float64*)datamap[topic];
//  std_msgs::Float64 *msg = (std_msgs::Float64*)iter->second;
//  return msg->data;
//}

rlab::NetworkData* RTBlackBoard::getData(const std::string& topic) {
	// check
	DataMap::iterator iter = inputMap.find(topic);
	if(iter == inputMap.end()) {
		throw "topic not in map";
	}
	// return data
	return iter->second;
}

int RTBlackBoard::size() {
	return this->inputMap.size() + this->outputMap.size();
}

//std::vector<double> RTBlackBoard::getVector(const std::string &topic)
//{
//  // check if topic is in the map
//  //if(datamap.find(topic)==datamap.end())
//  Tdatamap::iterator iter=datamap.find(topic);
//  if(iter==datamap.end())
//  {
//    throw "topic not in map";
//  }
//  // get the message out of the map, no check if it is the right message type
//  //std_msgs::Float64MultiArray *msg = (std_msgs::Float64MultiArray*)datamap[topic];
//  std_msgs::Float64MultiArray *msg = (std_msgs::Float64MultiArray*)iter->second;
//  return msg->data;
//}
/*
void RTBlackBoard::set(rlab::NetworkData* data, DataMap& map) {
	if( map.find(data->getTopic() == map.end() ) {
		if( data.hasType("Float") )
		map[topic] = new 
	}
}
*/

void RTBlackBoard::setFloat64(const std::string& topic, double val ) {
	setFloat64(topic, val, outputMap);
}

void RTBlackBoard::setFloat64(const std::string& topic, double val, DataMap &map) {
	DataMap::iterator iter = map.find(topic);
	if(iter == map.end()) {
		// the topic is empty so we need to create a new object and copy the data
		map[topic] = new rlab::Float64(topic, val);
	}
	else {
		// copy the new data
		rlab::Float64* data = static_cast<rlab::Float64*>(iter->second);

		if(data)
			data->set(val);
		else
			throw "topic has different type than expected";
	}
}

void RTBlackBoard::setFloat64MultiArray(const std::string& topic, const std::vector<double>& val) {
	setFloat64MultiArray(topic, val, outputMap);
}

void RTBlackBoard::setFloat64MultiArray(const std::string& topic, const std::vector<double>& val, DataMap &map) {
	DataMap::iterator iter = map.find(topic);
	if(iter == map.end()) {
		// if not in map create a new object with data from val
		map[topic] = new rlab::Float64MultiArray(topic, val);
	}
	else {
		// cast to message type
		rlab::Float64MultiArray* data = static_cast<rlab::Float64MultiArray*>(iter->second);
		
		if(data)
			data->set(val);
		else
			throw "topic has different type than expected";
	}
}

void RTBlackBoard::setJointState(const std::string& topic, const std::vector<double>& position, const std::vector<double>& velocity, const std::vector<double>& effort) {
	setJointState(topic, position, velocity, effort, outputMap);
}

void RTBlackBoard::setJointState(const std::string& topic, const std::vector<double>& position, const std::vector<double>& velocity, const std::vector<double>& effort, DataMap &map) {
	DataMap::iterator iter = map.find(topic);
	if(iter == map.end()) {
		// if not in map create a new object with data from val
		map[topic] = new rlab::JointState(topic, position, velocity, effort);
	}
	else {
		// cast to message type
		rlab::JointState* data = static_cast<rlab::JointState*>(iter->second);
		
		if(data) {
			data->setPosition(position);
			data->setVelocity(velocity);
			data->setEffort(effort);
		}
		else
			throw "topic has different type than expected";
	}
}

//void RTBlackBoard::set(const std::string &topic, const std::vector<double> &val)
//{
//  //if(datamap.find(topic)==datamap.end())
//  Tdatamap::iterator iter=datamap.find(topic);
//  if(iter==datamap.end())
//  {
//    std_msgs::Float64MultiArray *msg = new std_msgs::Float64MultiArray();
//    msg->data = val;
//    datamap[topic] = msg;
//  } else
//  {
//    //std_msgs::Float64MultiArray *msg = (std_msgs::Float64MultiArray*)datamap[topic];
//    std_msgs::Float64MultiArray *msg = (std_msgs::Float64MultiArray*)iter->second;
//    msg->data = val;
//  }
//}

bool RTBlackBoard::exists(const std::string& topic) const {
	// return true if topic is in the map
	return inputMap.find(topic) != inputMap.end();
}

bool RTBlackBoard::isUpdated(const std::string& topic) const {
	DataMap::const_iterator it = inputMap.find(topic);
	return (it != inputMap.end()) && it->second->isUpdated();
}

void RTBlackBoard::subscribeToROSMessage(const std::string& topic) {
	rlab::RequestROSSubscription req(topic);
	net << req;
}

//void RTBlackBoard::erase(const std::string &topic)
//{
//  // removes a topic from all the maps
//  removeSubscribe(topic);
//  removePublish(topic);
//  delete datamap[topic];
//  datamap.erase(topic);
//}

//void RTBlackBoard::removeSubscribe(const std::string &topic)
//{
//  if(sub.find(topic)!=sub.end()) // is the topic in the sub map?
//  {
//    // get the entry in the subscriber topic
//    Tsubmap  *submap = &sub[topic];
//    // delete subscriber object
//    boost::get<0>(*submap).reset();
//    // the message will be deleted by the subscriber object
//    // erase the pointer from the list
//    sub.erase(topic);
//  }
//}
//
//// add a double topic to the publish list
//void RTBlackBoard::addPublish(const std::string &topic)
//{
//  addPublish<std_msgs::Float64>(topic);
//}
//
//void RTBlackBoard::addSubscribe (const std::string &topic)
//{
//  addSubscribe <std_msgs::Float64>(topic);
//}
//
//void RTBlackBoard::removePublish(const std::string &topic)
//{
//  if(pub.find(topic)!=pub.end()) // is the topic in the sub map?
//  {
//    // erase the pointer from the list
//    pub.erase(topic);
//  }
//}
//
//void RTBlackBoard::publishToRos()
//{
//  // go thru all publish msg
//  int index = 0;
//  //for (Tpub::iterator it=pub.begin() ; it != pub.end(); it++ )
//
//  for ( ; publish_it != pub.end() && index < 10 ; ++publish_it )
//  {
////    index++;
//    (*publish_it).second.publish();
//  }
//}

void RTBlackBoard::step() {
	ticks++;

	if (ticks % ros_publish_every_n_ticks == 0) {
		// publish all the data of the outputMap
		for( DataMap::iterator it = this->outputMap.begin(); it != this->outputMap.end(); ++it ) {
			net << *(it->second);
		}
	}

	// set all updated flags to false in the input map
	for( DataMap::iterator it = this->inputMap.begin(); it != this->inputMap.end(); ++it ) {
		it->second->setIsUpdated(false);
	}

	// copy all the data from the inputBuffer to the inputMap
	DWORD rc = WaitForSingleObject(copyInputBufferMutex, 0);
	if( rc == WAIT_FAILED ) {
		std::cerr << "Cannot lock copyInputBufferMutex (RTBlackBoard::step)" << std::endl;
		return;
	}
	for( DataMap::iterator it = this->inputBuffer.begin(); it != this->inputBuffer.end(); ++it ) {
		update(it->second, inputMap);
	}
	// clear input buffer
	// TODO: i'm pretty sure we're in LEEKAPALOOZA here!
	/*for( DataMap::iterator it = this->inputBuffer.begin(); it != this->inputBuffer.end(); ++it ) {
		delete it->second;
	}*/
	inputBuffer.clear();
	if( !ReleaseMutex(copyInputBufferMutex) ) {
		std::cerr << "Cannot release copyInputBufferMutex (RTBlackBoard::step)" << std::endl;
	}
}

void RTBlackBoard::update(rlab::NetworkData *s) {
	DataMap::iterator it = inputBuffer.find(s->getTopic());
	if( it != inputBuffer.end() ) {
		// prevent input buffer from being copied at the same moment
		DWORD rc = WaitForSingleObject(copyInputBufferMutex, INFINITE);
		if( rc == WAIT_FAILED ) {
			std::cerr << "Cannot lock copyInputBufferMutex (RTBlackBoard::update)" << std::endl;
		}
		
		// TODO: check for type?
		it->second->update(s);

		if( !ReleaseMutex(copyInputBufferMutex) ) {
			std::cerr << "Cannot release copyInputBufferMutex (RTBlackBoard::update)" << std::endl;
		}

		delete s;
	}
	else {
		inputBuffer.insert( std::pair<std::string, rlab::NetworkData*>(s->getTopic(), s) );
	}
}

void RTBlackBoard::update(rlab::NetworkData *s, DataMap& map) {
	DataMap::iterator it = map.find(s->getTopic());
	if( it != map.end() ) {
		// TODO: check for type?
		it->second->update(s);
		//delete s;
	}
	else {
		map.insert( std::pair<std::string, rlab::NetworkData*>(s->getTopic(), s) );
		s->setIsUpdated(true);
	}
}