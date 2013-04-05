/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#ifndef __HYBRIDAUTOMATONMANAGER_H__
#define __HYBRIDAUTOMATONMANAGER_H__

#include "rControlAlgorithm/rControlAlgorithm.h"
#include "rxControlSDK/rxControlSDK.h"

#include "ros_bridge/include/RTBlackBoard.h"
#include "hybrid_automaton/include/hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/include/hybrid_automaton/LocalDecisionCriterion.h" 

#define _USE_RCONTROLALGORITHM_EX_

typedef struct DeserializingThreadArguments {
	std::string _string;
	rxSystem* _sys;
	double _dT;
	std::deque<HybridAutomaton*>* _deserialized_hybrid_automatons;
	HANDLE* _deserialize_mutex;
};

class REXPORT HybridAutomatonManager : public rControlAlgorithm
{
public:
	HybridAutomatonManager(rDC rdc);
	~HybridAutomatonManager();

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
	virtual void setHybridAutomaton(std::string  _new_hybrid_automaton_str);
	virtual void setCollisionInterface(CollisionInterface* collision_interface);
	virtual void setLocalDecisionCriterion(LocalDecisionCriterion* criterion); 

	RTBlackBoard* getBlackboard(){return this->_blackboard;};

protected:
	virtual void _estimate();
	virtual void _readDevices();
	virtual void _writeDevices();
	virtual void _reflect();
	virtual void _compute(const rTime& t);

	/**
	* Check if a new HA was written on the Blackboard and creates a new thread to deserialize it.
	*/
	void updateHybridAutomaton();
	void updateBlackboard();

	/**
	* The latest hybrid automaton
	*/
	HybridAutomaton*	_hybrid_automaton;
	
	/**
	* The rxSystem under control
	*/
	rxSystem*			_sys;

	/**
	* The ROBOT device
	*/
	rHANDLE				_robot;

	/**
	* The currently activeted motion behaviour
	*/
	MotionBehaviour*	_activeMotionBehaviour;

	/**
	* The motion behaviour that is used when no hybrid automaton is present
	* Consists of a joint controller that holds the current position.
	*/
	MotionBehaviour*	_defaultMotionBehavior;

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

	/**
	* An RTBlackBoard instance. The manager receives Hybrid automata from this blackboard.
	*/
	RTBlackBoard*		_blackboard;

	int					_dof;
	bool				_servo_on;

	/**
	* The queue of reserialized hybird automata. Is executed one by one
	*/
	std::deque<HybridAutomaton*> _deserialized_hybrid_automatons;

	/**
	* A mutex to synchronize the deserializing threads
	*/
	HANDLE				_deserialize_mutex;

	/**
	* A criterion that tells which motionBehaviour to choose if there are multiple options.
	*/
	LocalDecisionCriterion*	_criterion;
};
#endif