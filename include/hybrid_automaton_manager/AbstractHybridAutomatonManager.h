/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#ifndef __ABSTRACTHYBRIDAUTOMATONMANAGER_H__
#define __ABSTRACTHYBRIDAUTOMATONMANAGER_H__

#include "rControlAlgorithm/rControlAlgorithm.h"
#include "rxControlSDK/rxControlSDK.h"

#include "ros_bridge/include/RTBlackBoard.h"
#include "hybrid_automaton/include/hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton\include\hybrid_automaton\XMLDeserializer.h"

#define _USE_RCONTROLALGORITHM_EX_

struct DeserializingThreadArguments {
	std::string _string;
	rxSystem* _sys;
	double _dT;
	bool _noQueue;
	std::deque<HybridAutomaton*>* _deserialized_hybrid_automatons;
	HANDLE* _deserialize_mutex;
};

class REXPORT AbstractHybridAutomatonManager : public rControlAlgorithm
{
public:
	AbstractHybridAutomatonManager(rDC rdc);
	virtual ~AbstractHybridAutomatonManager();

	virtual void init(int mode = 0);
	virtual void update(const rTime& t);
	virtual void setNominalSystem(const TCHAR* path, const TCHAR* aml, const HTransform& T0, const dVector& q0);
	virtual void setPeriod(const rTime& dT);
	virtual int command(const short& cmd, const int& arg = 0);
	virtual void datanames(vector<string_type>& names, int channel = -1);
	virtual void collect(vector<double>& data, int channel = -1);
	virtual void onSetInterestFrame(const TCHAR* name, const HTransform& T);

	virtual void activateBlackboard(std::string &rlab_host, int rlab_port, std::string &ros_host, int ros_port);
	virtual void setHybridAutomaton(HybridAutomaton*  _new_hybrid_automaton);
	virtual void setHybridAutomaton(const std::string&  _new_hybrid_automaton_str);

	RTBlackBoard* getBlackboard(){return this->_blackboard;};
	void setNoQueue(bool noQueue){_noQueue = noQueue;};

protected:
	void _estimate();
	void _readDevices();
	void _writeDevices();
	void _reflect();
	void _compute(const rTime& t);

	/**
	* Check if a new HA was written on the Blackboard and creates a new thread to deserialize it.
	*/
	virtual void updateHybridAutomaton();
	virtual void updateMotionBehaviour(const rTime& t) = 0;
	virtual void updateBlackboard();

	/**
	* The latest hybrid automaton
	*/
	HybridAutomaton*	_hybrid_automaton;
	
	/**
	* The rxSystem under control
	*/
	rxSystem*			_sys;

	// the force/torque sensor
	rxDevice*			_ft_sensor;

	/**
	* The ROBOT device
	*/
	rHANDLE				_robot;

	/**
	* The currently activeted motion behaviour
	*/
	MotionBehaviour*	_activeMotionBehaviour;

	/**
	* The path to the AML file
	*/
	string_type			_path;

	/**
	* The AML file of the current robot
	*/
	string_type			_aml;

	/**
	* Start Transformation and Configuration
	*/
	HTransform			_T0;
	dVector				_q0;

	/**
	* The control rate
	*/
	double				_dT;

	/**
	* Parameters for plotting
	*/
	dVector				_q;
	dVector				_qOld;
	dVector				_qdot;
	dVector				_torque;

	rxFilteredDerivative*	 _qdot_filter;

	/**
	* An RTBlackBoard instance. The manager receives Hybrid automata from this blackboard.
	*/
	RTBlackBoard*		_blackboard;

	int					_dof;
	bool				_servo_on;

	/**
	* If this is set to true, hybrid automata are not buffered -
	* Only the latest received automaton is executed.
	*/
	bool				_noQueue;

	/**
	* The queue of reserialized hybird automata. Is executed one by one
	*/
	std::deque<HybridAutomaton*> _deserialized_hybrid_automatons;

	/**
	* A mutex to synchronize the deserializing threads
	*/
	HANDLE				_deserialize_mutex;

};
#endif