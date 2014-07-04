/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#include "SequentialHybridAutomatonManager.h"
#include "SequentialHybridAutomatonManagerCmd.h"

SequentialHybridAutomatonManager::SequentialHybridAutomatonManager(rDC rdc) 
#ifdef _USE_RCONTROLALGORITHM_EX_
:AbstractHybridAutomatonManager(rdc)
#else
:AbstractHybridAutomatonManager(rdc)
#endif
{
}

SequentialHybridAutomatonManager::~SequentialHybridAutomatonManager()
{
}


void SequentialHybridAutomatonManager::updateMotionBehaviour(const rTime& t)
{
	Milestone* childMs = (Milestone*)(_activeMotionBehaviour->getChild());

	if( !_deserialized_hybrid_automatons.empty() )
	{
		if (WaitForSingleObject(_deserialize_mutex, 0) != WAIT_FAILED && !_deserialized_hybrid_automatons.empty())
		{
			delete _hybrid_automaton;
			_hybrid_automaton = _deserialized_hybrid_automatons.front();
			_deserialized_hybrid_automatons.pop_front();
			
			std::cout << "New Hybrid Automaton" << std::endl;
			_activeMotionBehaviour->deactivate();
			_activeMotionBehaviour = (MotionBehaviour*) _hybrid_automaton->outgoingEdges(_hybrid_automaton->getStartNode())[0];
			_activeMotionBehaviour->activate();
			ReleaseMutex(_deserialize_mutex);
		}
	}
	else if (_hybrid_automaton && childMs->hasConverged(_sys))
	{
		std::cout << "[HybridAutomatonManager::_compute] INFO: Milestone " << childMs->getName() << " converged." << std::endl;
		std::vector<const Edge*> edges = _hybrid_automaton->outgoingEdges(childMs);
		if (!edges.empty())
		{
			_activeMotionBehaviour->deactivate();
			_activeMotionBehaviour = (MotionBehaviour*) edges[0];
			_activeMotionBehaviour->activate();
			std::cout << "[HybridAutomatonManager::_compute] INFO: Changed motion behavior. Next goal: " << ((Milestone*)(_activeMotionBehaviour->getChild()))->getName() << std::endl;
		}
	}
}


void SequentialHybridAutomatonManager::updateBlackboard()
{
	if(!_blackboard)
		return;

	if (_currentMotionBehavior && _currentMotionBehavior->getChild())
		_blackboard->setBool("idle", ((Milestone*)_currentMotionBehavior->getChild())->hasConverged(_sys));
	else
		_blackboard->setBool("idle", false);

	AbstractHybridAutomatonManager::updateBlackboard();
}

rControlAlgorithm* CreateControlAlgorithm(rDC& rdc)
{
	return new SequentialHybridAutomatonManager(rdc);
}
