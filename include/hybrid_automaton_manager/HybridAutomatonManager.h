/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#ifndef __HYBRIDAUTOMATONMANAGER_H__
#define __HYBRIDAUTOMATONMANAGER_H__

//If switched to 1, AMCL localization is used via blackboard
#define USE_LOCALIZATION 0

#include "hybrid_automaton/include/hybrid_automaton_manager/AbstractHybridAutomatonManager.h"

#include "hybrid_automaton/include/hybrid_automaton/LocalDecisionCriterion.h" 

#define _USE_RCONTROLALGORITHM_EX_

class REXPORT HybridAutomatonManager : public AbstractHybridAutomatonManager
{
public:
	HybridAutomatonManager(rDC rdc);
	virtual ~HybridAutomatonManager();

	virtual int command(const short& cmd, const int& arg = 0);

	virtual void setCollisionInterface(CollisionInterface* collision_interface);
	virtual void setLocalDecisionCriterion(LocalDecisionCriterion* criterion); 
	virtual void activateBlackboard(std::string &rlab_host, int rlab_port, std::string &ros_host, int ros_port);

protected:
	virtual void _reflect();

	/**
	* Check if a new HA was written on the Blackboard and creates a new thread to deserialize it.
	*/
	virtual void updateMotionBehaviour(const rTime& t);
	virtual void updateBlackboard();

	void gravityCompensation();

	/**
	* true if the HAM should execute hybrid automata
	*/
	bool				_active;

	/**
	* A criterion that tells which motionBehaviour to choose if there are multiple options.
	*/
	LocalDecisionCriterion*	_criterion;

#ifdef USE_LOCALIZATION
	HTransform _localizedFrame;
	dVector _qLoc;
#endif
};
#endif