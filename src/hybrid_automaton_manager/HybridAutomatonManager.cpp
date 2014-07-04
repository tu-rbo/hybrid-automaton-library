/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
*
* This library is commercial and cannot be redistributed, and/or modified
* WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
*/
#include "HybridAutomatonManager.h"
#include "HybridAutomatonManagerCmd.h"

#include "XMLDeserializer.h"
#include "msgs\String.h"
#include "msgs\Transform.h"

#include "controllers\include\TPImpedanceControlSet.h"

#include <process.h>

HybridAutomatonManager::HybridAutomatonManager(rDC rdc) 
:AbstractHybridAutomatonManager(rdc)
, _criterion(new LocalDecisionCriterion())
, _active(false)
{
#ifdef USE_LOCALIZATION
	_qLoc.resize(10);
#endif
}

HybridAutomatonManager::~HybridAutomatonManager()
{
    if(_criterion)
		delete _criterion;
}

void HybridAutomatonManager::gravityCompensation()
{
	TPImpedanceControlSet* nullControl = new TPImpedanceControlSet(this->_sys, this->_dT);
	nullControl->setGravity(0, 0, -GRAV_ACC);
	
	_activeMotionBehaviour = new MotionBehaviour(new Milestone(), new Milestone(),nullControl, 0.0);
	_activeMotionBehaviour->activate();
}

void HybridAutomatonManager::activateBlackboard(std::string &rlab_host, int rlab_port, std::string &ros_host, int ros_port)
{
	_blackboard = RTBlackBoard::getInstance(rlab_host, rlab_port, ros_host, ros_port);
	
#ifdef USE_LOCALIZATION

	//Wait until localization messages are updated
	while(!_blackboard->exists("/odom"))
	{
		std::cout<<"waiting for blackboard"<<std::endl;
		_blackboard->subscribeToTransform("/odom", "/map");
		updateBlackboard();
		Sleep(500);
	}
#endif

}

void HybridAutomatonManager::setCollisionInterface(CollisionInterface* collision_interface)
{
	CollisionInterface::instance = collision_interface;
}

void HybridAutomatonManager::setLocalDecisionCriterion(LocalDecisionCriterion* criterion)
{
	delete _criterion;
	_criterion=criterion;
}

void HybridAutomatonManager::updateBlackboard()
{
	if(!_blackboard)
		return;

#ifdef USE_LOCALIZATION
	std::vector<double> b_position;
	std::vector<double> b_velocity;
	std::vector<double> b_effort;
	for(int i=7; i<10; i++)
	{
		b_position.push_back(_q[i]);
		b_velocity.push_back(_qdot[i]);
		b_effort.push_back(0.0);
	}
	_blackboard->setJointState("/xr4000_jointstate", b_position, b_velocity, b_effort);

	if(_blackboard->isUpdated("/odom"))
	{
		// to odom

		rlab::Transform* localT = dynamic_cast<rlab::Transform*>(_blackboard->getData("/odom"));
		
		HTransform locFrame;
		locFrame.r = Displacement(localT->getPosition()[0],localT->getPosition()[1],localT->getPosition()[2]);
		locFrame.R = Rotation(localT->getOrientation()[0],localT->getOrientation()[1],localT->getOrientation()[2],localT->getOrientation()[3]);		
			
		double maxLocalizationUpdate = 0.01;
		double maxLocalizationUpdateTheta = 0.01;

		double updateX = localT->getPosition()[0]-_localizedFrame.r(0);
		double updateY = localT->getPosition()[1]-_localizedFrame.r(1);
		double updateTheta = acos(locFrame.R(0,0)) - acos(_localizedFrame.R(0,0));

		if( fabs(updateX) < 0.2 && fabs(updateY) < 0.2 && fabs(updateTheta) < 0.2 )
		{
			if(updateX > 0.0)
				updateX = max(updateX, maxLocalizationUpdate);
			else
				updateX = min(updateX, -maxLocalizationUpdate);

			if(updateY > 0.0)
				updateY = max(updateY, maxLocalizationUpdate);
			else
				updateY = min(updateY, -maxLocalizationUpdate);

			if(updateTheta > 0.0)
				updateTheta = max(updateTheta, maxLocalizationUpdateTheta);
			else
				updateTheta = min(updateTheta, -maxLocalizationUpdateTheta);

			_localizedFrame.Reset();
			_localizedFrame.r[0] = updateX;
			_localizedFrame.r[1] = updateY;
			_localizedFrame.R.Set(updateTheta, 0.0, 0.0, ZYX);
		}
		else
		{
			//We have a large localization update - something went wrong
			std::cout<<"Localization error exceeded 20cm! - update IGNORED!!!"<<std::endl;
		}
	}
#endif
	
	AbstractHybridAutomatonManager::updateBlackboard();
}

void HybridAutomatonManager::_reflect()
{
#ifdef USE_LOCALIZATION
	HTransform odoT;
	odoT.r = Displacement(_q[7], _q[8], 0.0);
	odoT.R = Rotation(_q[9],0.0,0.0);
	HTransform newT = _localizedFrame*odoT;
	
	_qLoc = _q;
	_qLoc[7]=newT.r(0);
	_qLoc[8]=newT.r(1);
	double angLoc;
	Vector3D dir;
	newT.R.GetEquivalentAngleAxis(dir,angLoc);
	
	if(dir[2]<0)
	{
		//can this happen?
		std::cout<<"axis switched"<<std::endl;
		angLoc *= -1.0;
	}

	_qLoc[9]=angLoc;

	_sys->q(_qLoc);

#else	
	_sys->q(_q);
#endif
	_sys->qdot(_qdot);
}

void HybridAutomatonManager::updateMotionBehaviour(const rTime& t)
{
	if(!_active)
		return;

	//Now switch the active motion behaviour if possible
    Milestone* childMs=(Milestone*)(_activeMotionBehaviour->getChild());
	Milestone* queryMs; //The milestone we choose our next motion from

	
	//Flag tells if the graph structure changed.	
	bool behaviourChange = true;

	if (!_deserialized_hybrid_automatons.empty() && WaitForSingleObject(_deserialize_mutex, 0) != WAIT_FAILED)
	{
		delete _hybrid_automaton;
		_hybrid_automaton = _deserialized_hybrid_automatons.front();

		_deserialized_hybrid_automatons.pop_front();
		
		//std::cout << "[HybridAutomatonManager::_compute] INFO: Switching Hybrid Automaton" << std::endl;

		ReleaseMutex(_deserialize_mutex);

		queryMs = _hybrid_automaton->getStartNode(); //Take the first behaviour from the new roadmap
		behaviourChange = true;
	}
	else if(childMs->hasConverged(_sys)) //The current behaviour converged
	{
		std::cout<<"[HybridAutomatonManager::_compute] INFO: Milestone "<<childMs->getName()<<" converged."<<std::endl;
		queryMs = childMs; //Converged, switch to next milestone
		behaviourChange = true;

		if(childMs->getName() == "goal")
		{
			_active = false;
			return;
		}
	}
	else	
	{
		queryMs = (Milestone*)_activeMotionBehaviour->getParent(); //Switch before reaching the milestone.
		behaviourChange = false;		
	}

	// make new local decision:
	if(_hybrid_automaton)
	{
		MotionBehaviour* nextMotion; 
		nextMotion = _criterion->getNextMotionBehaviour(queryMs,_hybrid_automaton, behaviourChange, t, _sys->q());

		//Switch motion behaviour
		if(nextMotion && nextMotion != _activeMotionBehaviour)
		{
			if(nextMotion->isUpdateAllowed() && _activeMotionBehaviour->isUpdateAllowed())
			{
				dVector delta = nextMotion->getGoal() - _activeMotionBehaviour->getGoal();

				//Try to update currently running behaviour instead of replacing it
				if(delta.norm() > 0.01)
				{
					_activeMotionBehaviour->updateControllers(nextMotion);
					Milestone* next=(Milestone*)(nextMotion->getChild());
					std::cout << "[HybridAutomatonManager::_compute] INFO: Updated goal towards "<< next->getName() << " ,x="<<next->getConfiguration()[7]<<", y="<<next->getConfiguration()[8]<<std::endl;
				}
			}
			else
			{
				//update not possible (controller types changed) - exchange motion behaviour
				_activeMotionBehaviour->deactivate();
				_activeMotionBehaviour = nextMotion;
				_activeMotionBehaviour->activate();
				std::cout << "[HybridAutomatonManager::_compute] INFO: Changed control set. Next goal: "<< ((Milestone*)(nextMotion->getChild()))->getName() << std::endl;			
			}
		}
	}
}

int HybridAutomatonManager::command(const short& cmd, const int& arg)
{
	AbstractHybridAutomatonManager::command(cmd, arg);

	switch (cmd)
	{
		case ACTIVATE:
		{
			_active = !_active;
		}
		break;
	}

	return 0;
}

rControlAlgorithm* CreateControlAlgorithm(rDC& rdc)
{
	return new HybridAutomatonManager(rdc);
}
