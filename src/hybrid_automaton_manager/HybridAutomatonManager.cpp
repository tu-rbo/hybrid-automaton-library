/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#include "HybridAutomatonManager.h"
#include "HybridAutomatonManagerCmd.h"

#include "msgs\String.h"

std::vector<double> convert(const dVector& in)
{
	std::vector<double> out(in.size());
	for(int i = 0; i < in.size(); ++i)
		out[i] = in[i];
	return out;
}

HybridAutomatonManager::HybridAutomatonManager(rDC rdc) 
:rControlAlgorithm(rdc)
, _robot(NULL)
, _blackboard(NULL)
, _activeMotionBehavior(NULL)
, _defaultMotionBehavior(NULL)
, _dof()
, _HS()
, _newHAArrived(false)
{
}

HybridAutomatonManager::~HybridAutomatonManager()
{
	delete _activeMotionBehavior;

	//delete _defaultMotionBehavior;

	for( ::std::vector< MotionBehaviour* >::const_iterator it = _navigationFunction.begin(); it != _navigationFunction.end(); ++it)
		delete *it;
	_navigationFunction.clear();

	FREE_SYSTEMS();
}



void HybridAutomatonManager::init(int mode)
{

	_robot =  LOAD_SYSTEM(_path, _aml, _T0, _q0);
	assert(_robot);

	_dof = _robot->jdof();

	//Initialize variables
	_q.resize(_robot->jointDOF() + _robot->earthDOF() + _robot->constraintDOF());
	_qdot.resize(_robot->jointDOF() + _robot->earthDOF() + _robot->constraintDOF());
	_torque.resize(_robot->jointDOF() + _robot->earthDOF() + _robot->constraintDOF());

	_q_BB.resize(_dof);		
	_qdot_BB.resize(_dof);
	_torque_BB.resize(_dof);

	_q.zero();
	_torque.zero();
	_qdot.zero();
	
	//Find the robot device. This devicce is used to read position and write torques
	//to the robot
	_robotDevice = findDevice(_T("ROBOT"));
	RASSERT(_robotDevice != INVALID_RHANDLE);

	_blackboard = RTBlackBoard::getInstance();

	_defaultMotionBehavior = new MotionBehaviour(new Milestone(), new Milestone(),_robot);
	_activeMotionBehavior = _defaultMotionBehavior;
}


void HybridAutomatonManager::update(const rTime& t)
{
	this->updateHybridAutomaton();

	rControlAlgorithm::update(t);

	this->updateBlackboard();

}

void HybridAutomatonManager::updateBlackboard()
{
	_blackboard->setFloat64MultiArray("angle", _q_BB);

	_blackboard->setFloat64MultiArray("velocity", _qdot_BB);

	_blackboard->setFloat64MultiArray("torque" , _torque_BB);

	_blackboard->step();	
}

void HybridAutomatonManager::updateHybridAutomaton()
{
	if (_blackboard->isUpdated("update_hybrid_automaton"))
	{
		rlab::String* s = dynamic_cast<rlab::String*>(_blackboard->getData("update_hybrid_automaton"));
		_HS = s->get();
		//::std::cout << _HS << ::std::endl;
		try
		{
			_plan.fromStringXML(_HS,_robot);
			//::std::cout << _plan.toStringXML() << ::std::endl;
		}
		catch(::std::string e)
		{
			::std::cout << e << ::std::endl;
		}

		//_navigationFunction.clear();
		_activeMotionBehavior = NULL;

		//Milestone* tmpMilestone = _plan.getStartNode();
	
		//for( int i = 1; i < _plan.getNodeNumber(); ++i)
		//{
		//	_navigationFunction.push_back(_plan.outgoingEdges(*tmpMilestone)[0]);
		//	//::std::cout << _plan.outgoingEdges(*tmpMilestone)[0]->toStringXML() << ::std::endl;
		//	//::std::cout << _navigationFunction[0]->toStringXML() << ::std::endl;
		//	tmpMilestone = _navigationFunction.back()->getChild();
		//}

		//delete tmpMilestone;

		_newHAArrived = true;
	}
}

void HybridAutomatonManager::setNominalSystem(const TCHAR* path, const TCHAR* aml, const HTransform& T0, const dVector& q0)
{
	this->_path = path;

	this->_aml = aml;

	this->_T0 = T0;
	
	this->_q0 = q0;
}

void HybridAutomatonManager::setPeriod(const rTime& dT)
{
	this->_dT = dT;
}


void HybridAutomatonManager::_readDevices()
{
	double* q = new double[_dof];
	double* qdot = new double[_dof];

	int read = readDeviceValue(_robotDevice, q, _dof * sizeof(double), 0);
	RASSERT(read > 0);

	read = readDeviceValue(_robotDevice, qdot, _dof * sizeof(double), 1);
	RASSERT(read > 0);

	for(int i = 0; i < _dof; ++i)
	{
		_q[i] = q[i];
		_qdot[i] = qdot[i];
	}

	_qdot_BB = convert(_qdot);
	_q_BB = convert(_q);

	delete q;
	delete qdot;
}


void HybridAutomatonManager::_writeDevices()
{
	double* torque = new double[_dof];

	for(int i = 0; i < _dof; ++i)
		torque[i] = _torque[i];

	int written = writeDeviceValue(_robotDevice, torque, _dof * sizeof(double));
	RASSERT(written > 0);

	_torque_BB = convert(_torque);

	delete torque;

}


void HybridAutomatonManager::_reflect()
{
	_robot->q(_q);

	_robot->qdot(_qdot);
}


void HybridAutomatonManager::_compute(const double& t)
{

	if( _newHAArrived )
	{
		Milestone* tmpMilestone = _plan.getStartNode();
		_activeMotionBehavior = _plan.outgoingEdges(*tmpMilestone)[0];
		_newHAArrived = false;
		_activeMotionBehavior->activate();
	}
	else if(_activeMotionBehavior->hasConverged() ){
		if(_plan.getEdgeNumber() > 0){
			std::cout << "Switching controller" << std::endl;
			_activeMotionBehavior = _plan.outgoingEdges(*(_activeMotionBehavior->getChild()))[0];
			::std::cout << _activeMotionBehavior->toStringXML() << ::std::endl;
		}else{
			std::cout << ".";
			_activeMotionBehavior = _defaultMotionBehavior;
		}
		_activeMotionBehavior->activate();
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	//NOTE (Roberto): The old version creates a vector of MBs, the _navigationFunction, when a new plan is received
	//We then iterate over this vector.
	//It is much more flexible if we extract the next MB to execute directly from the HS
	//NOTE2: We take simply the first outgoing MB from the reached MS, but we could decide between several outgoing edges if they exist
	/*if( _newHAArrived 	|| _activeMotionBehavior->hasConverged())
	{
		if(_navigationFunction.size() < 1)
		{
			std::cout << "Using default controller" << std::endl;
			_activeMotionBehavior = _defaultMotionBehavior;
		}
		else
		{
			std::cout << "Switching controlller" << std::endl;
			_activeMotionBehavior = _navigationFunction.front();
			_navigationFunction.erase(_navigationFunction.begin());
			::std::cout << _activeMotionBehavior->toStringXML() << ::std::endl;
		}		
		_newHAArrived = false;
		_activeMotionBehavior->activate();
	}*/
	
	_torque = _activeMotionBehavior->update(t);	
}



void HybridAutomatonManager::_estimate()
{

}

int HybridAutomatonManager::command(const short& cmd, const int& arg)
{
	switch (cmd)
	{
	case EXECUTE_PLAN:
		{
		}
		break;
	case PAUSE:
		{

		}
		break;
	case RESUME:
		{

		}
		break;
	default:
		break;
	}

	return 0;
}


void HybridAutomatonManager::datanames (vector<string_type>& names, int channel)
{
}

void HybridAutomatonManager::collect(vector<double>& data, int channel)
{
	if (channel == 1)
	{
		for(int i = 0; i < _dof; ++i)
			data.push_back(_torque[i]);
	}
	else if (channel == 2)
	{
		for(int i = 0; i < _dof; ++i)
			data.push_back(_q[i]);
	}
	else if (channel == 3)
	{
		for(int i = 0; i < _dof; ++i)
			data.push_back(_qdot[i]);
	}
}

void HybridAutomatonManager::onSetInterestFrame(const TCHAR* name, const HTransform& T)
{
}

rControlAlgorithm* CreateControlAlgorithm(rDC& rdc)
{
	return new HybridAutomatonManager(rdc);
}
