/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#ifndef __SEQUENTIALHYBRIDAUTOMATONMANAGER_H__
#define __SEQUENTIALHYBRIDAUTOMATONMANAGER_H__

#include "hybrid_automaton/include/hybrid_automaton_manager/AbstractHybridAutomatonManager.h"

//#define _USE_RCONTROLALGORITHM_EX_

#ifdef _USE_RCONTROLALGORITHM_EX_
class REXPORT SequentialHybridAutomatonManager : public AbstractHybridAutomatonManager
#else
class REXPORT SequentialHybridAutomatonManager : public AbstractHybridAutomatonManager
#endif
{
public:
	SequentialHybridAutomatonManager(rDC rdc);
	virtual ~SequentialHybridAutomatonManager();

private:
	virtual void updateMotionBehaviour(const rTime& t);
	virtual void updateBlackboard();
};
#endif