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
#include "MotionBehaviour.h"
#include "Milestone.h"
#include "HybridAutomaton.h"

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

	

private:
	virtual void _estimate();
	virtual void _readDevices();
	virtual void _writeDevices();
	virtual void _reflect();
	virtual void _compute(const rTime& t);

	void updateHybridAutomaton();

private:

	HybridAutomaton						_plan;

	std::string							_HS;

	rxSystem*							_robot;

	rHANDLE								_robotDevice;

	RTBlackBoard*						_blackboard;

	MotionBehaviour*					_activeMotionBehavior;

	string_type							_path;
	
	string_type							_aml;

	HTransform							_T0;

	dVector								_q0;

	double								_dT;

	dVector								_q;

	dVector								_qdot;

	dVector								_torque;

	std::vector<double>					_q_BB;

	std::vector<double>					_qdot_BB;

	std::vector<double>					_torque_BB;

	int									_dof;

	::std::vector< MotionBehaviour* >   _navigationFunction;

	bool								_newHAArrived;

};
#endif