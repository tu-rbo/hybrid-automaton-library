/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#ifndef __HYBRIDAUTOMATONMANAGER_H__
#define __HYBRIDAUTOMATONMANAGER_H__

#include <list>
#include "rControlAlgorithm/rControlAlgorithm.h"
#include "rxControlSDK/rxControlSDK.h"

#include "RTBlackBoard.h"

#include "CSpaceMilestone.h"
#include "OpSpaceMilestone.h"
#include "MotionBehaviour.h"
#include "Milestone.h"
#include "HybridAutomaton.h"

#define _USE_RCONTROLALGORITHM_EX_

//#define DRAW_HYBRID_AUTOMATON
//#define DRAW_LOCAL_DECISION
#include "rCustomDraw.h"

typedef struct DeserializingThreadArguments {
	std::string _string;
	rxSystem* _robot;
	double _dT;
	std::deque<HybridAutomaton*>* _deserialized_hybrid_automatons;
	HANDLE* _deserialize_mutex;
};

enum drawLists
{
	eRAYS,
	ePATH,
	eCYLINDERS,
	eCUSTOM1,
	eCUSTOM2
};

enum drawListsM
{
	eLD_MILESTONES,
	ePATH_MILESTONES	
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
	virtual void setHybridAutomaton(std::string  _new_hybrid_automaton_str, CollisionInterface* collision_interface);
	virtual void setCollisionInterface(CollisionInterface* collision_interface);
	virtual void setLocalDecisionCriterion(LocalDecisionCriterion* criterion); // TODO: in future this could be defined in the milestones and serialised!
	//virtual void setPhysicsWorld(rxWorld* physics_world); // for debug draw...
	virtual bool isBlackboardActive() const;
	RTBlackBoard* getBlackboard(){return this->_blackboard;};


	//Drawing//////////////////////////////////////////////////////////
	std::vector<rCustomDrawInfo> getDrawObjects(drawLists index);
	std::vector<dVector> getDrawMilestones(drawListsM index);

	bool getRequestDraw(drawLists index) {return _request_draw[index];};
	void setRequestDraw(drawLists index, bool draw) {_request_draw[index] = draw;};

	bool getRequestDrawM(drawListsM index) {return _request_draw_m[index];};
	void setRequestDrawM(drawListsM index, bool draw) {_request_draw_m[index] = draw;};

	//TODO: WTF?
	double							max_ori_error[3];
	double							max_ori_error_norm;
	double							minObstDistanceBase;
	double							minObstDistanceEE;
	rxSystem*			_robot;

private:
	virtual void _estimate();
	virtual void _readDevices();
	virtual void _writeDevices();
	virtual void _reflect();
	virtual void _compute(const rTime& t);
	void drawLine(const rMath::Displacement &start, const rMath::Displacement &end, drawLists index, const rColor &color = RED, const char lineWidth = 1);
	void drawMilestones(const OpSpaceMilestone* ms, drawListsM index);

	/**
	* Check if a new HA was written on the Blackboard and creates a new thread to deserialize it.
	*/
	void updateHybridAutomaton();
	void updateBlackboard();

	HybridAutomaton*	_hybrid_automaton;

	
	rHANDLE				_robotDevice;

	MotionBehaviour*	_activeMotionBehavior;
	MotionBehaviour*	_defaultMotionBehavior;

	string_type			_path;
	string_type			_aml;

	HTransform			_T0;

	dVector				_q0;
	double				_dT;
	dVector				_q;
	dVector				_qdot;
	dVector				_torque;

	RTBlackBoard*		_blackboard;
	std::vector<double>	_q_BB;
	std::vector<double>	_qdot_BB;
	std::vector<double>	_torque_BB;

	int					_dof;
	bool				_servo_on;

	std::deque<HybridAutomaton*> _deserialized_hybrid_automatons;
	HANDLE				_deserialize_mutex;

	//rxWorld*			_physics_world;
	LocalDecisionCriterion*	_criterion;
	double				_t_old;

	std::vector<rCustomDrawInfo>	_draw_objects[5];
	bool							_request_draw[5];

	std::vector<dVector>			_draw_objects_m[2];
	bool							_request_draw_m[2];
};
#endif