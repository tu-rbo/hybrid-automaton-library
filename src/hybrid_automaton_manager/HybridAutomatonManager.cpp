/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
*
* This library is commercial and cannot be redistributed, and/or modified
* WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
*/
#include "HybridAutomatonManager.h"
#include "HybridAutomatonManagerCmd.h"

#include "msgs\String.h"

#include <process.h>

//#define NOT_IN_RT

unsigned __stdcall deserializeHybridAutomaton(void *udata)
{
	DeserializingThreadArguments* thread_args = static_cast<DeserializingThreadArguments*>(udata);
	HybridAutomaton* ha = new HybridAutomaton();

	try {
		ha->fromStringXML(thread_args->_string, thread_args->_robot, thread_args->_dT);
	}
	catch(std::string e)
	{
		std::cerr << "ERROR while deserializing hybrid automaton!" << std::endl;
		std::cerr << e << std::endl;

		delete ha;
		delete thread_args;
		return 0;
	}

	WaitForSingleObject(*(thread_args->_deserialize_mutex), INFINITE);
	thread_args->_deserialized_hybrid_automatons->push_back(ha);
	ReleaseMutex(*(thread_args->_deserialize_mutex));
	
	delete thread_args;

	return 0;
}

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
, _dof(0)
, _servo_on(false)
, _hybrid_automaton(NULL)
{
	_deserialize_mutex = CreateMutex(0, FALSE, 0);
	if( !_deserialize_mutex ) {
		throw(std::string("[HybridAutomatonManager::HybridAutomatonManager(rDC rdc)] Mutex was not created (_deserialize_mutex)!"));
	}
}

HybridAutomatonManager::~HybridAutomatonManager()
{
	delete _activeMotionBehavior;

	//delete _defaultMotionBehavior;

	FREE_SYSTEMS();
}

void HybridAutomatonManager::init(int mode)
{
	//std::wcout << _path << std::endl;
	//std::wcout << _aml << std::endl;

	_robot =  LOAD_SYSTEM(_path, _aml, _T0, _q0);
	assert(_robot);

	_dof = _robot->jointDOF() + _robot->earthDOF() + _robot->constraintDOF();

	//Initialize variables
	_q.resize(_dof);
	_qdot.resize(_dof);
	_torque.resize(_dof);

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

	_blackboard = RTBlackBoard::getInstance("130.149.238.179", 1888, "130.149.238.184", 1999);

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
	_blackboard->setJointState("joint_state", _q_BB, _qdot_BB, _torque_BB);
	rxBody* ee = _robot->findBody(_T("Body7"));
	HTransform h = ee->T();
	_blackboard->setTransform("ee", h, "base");
	_blackboard->step();
}

void HybridAutomatonManager::updateHybridAutomaton()
{
	if (_blackboard->isUpdated("update_hybrid_automaton"))
	{
		rlab::String* ha_xml = dynamic_cast<rlab::String*>(_blackboard->getData("update_hybrid_automaton"));
		try
		{
			DeserializingThreadArguments* thread_args = new DeserializingThreadArguments();
			thread_args->_robot = this->_robot;
			thread_args->_string = ha_xml->get();
			thread_args->_dT = this->_dT;
			thread_args->_deserialize_mutex = &(this->_deserialize_mutex);
			thread_args->_deserialized_hybrid_automatons = &(this->_deserialized_hybrid_automatons);

			if (_beginthreadex(NULL, 0, deserializeHybridAutomaton, (void*)thread_args, 0, NULL) == 0)
			{
				std::cerr << "[HybridAutomatonManager::updateHybridAutomaton()] Error creating thread to deserialize xml string!" << std::endl;
			}
		}
		catch(::std::string e)
		{
			std::cout << "Error caught: " << std::endl;
			::std::cout << e << ::std::endl;
		}
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
	{
		if(_servo_on){
			torque[i] = static_cast<double>(_torque[i]);
			//std::cerr << torque[i] << std::endl;
		}else{
			torque[i] =0;
		}
	}
	int written = writeDeviceValue(_robotDevice, torque, _dof * sizeof(double));
	RASSERT(written > 0);
	_torque_BB = convert(_torque);
	delete[] torque;
}


void HybridAutomatonManager::_reflect()
{
	_robot->q(_q);

	_robot->qdot(_qdot);
}


void HybridAutomatonManager::_compute(const double& t)
{
	_torque = _activeMotionBehavior->update(t);	

	if( !_deserialized_hybrid_automatons.empty() )
	{
		if (WaitForSingleObject(_deserialize_mutex, 0) != WAIT_FAILED && !_deserialized_hybrid_automatons.empty())
		{
			delete _hybrid_automaton;
			_hybrid_automaton = _deserialized_hybrid_automatons.front();
			_deserialized_hybrid_automatons.pop_front();
			
			std::cout << "New Hybrid Automaton" << std::endl;
			Milestone* tmpMilestone = _hybrid_automaton->getStartNode();
			_activeMotionBehavior->deactivate();
			_activeMotionBehavior = _hybrid_automaton->outgoingEdges(*tmpMilestone)[0];
			_activeMotionBehavior->activate();
#ifdef NOT_IN_RT
			std::cout << _activeMotionBehavior->toStringXML() << ::std::endl;
			_activeMotionBehavior->print();
			std::cout << "Number of edges: " << _hybrid_automaton->getEdgeNumber() << std::endl;
			_hybrid_automaton->__printMatrix();
#endif
			ReleaseMutex(_deserialize_mutex);
		}
	}
	else if(_activeMotionBehavior->hasConverged() ){/*
		this->_q.print(_T("configuration"));*/
		if (_hybrid_automaton && !_hybrid_automaton->outgoingEdges(*(_activeMotionBehavior->getChild())).empty()) {
			std::cout << "Switching controller" << std::endl;
			_activeMotionBehavior->deactivate();
			
			_activeMotionBehavior = _hybrid_automaton->outgoingEdges(*(_activeMotionBehavior->getChild()))[0];
			_activeMotionBehavior->activate();

#ifdef NOT_IN_RT
			std::cout << _activeMotionBehavior->toStringXML() << ::std::endl;
			_activeMotionBehavior->print();
#endif
		}
	}
	
	//_torque.print(_T("_torquenew"));
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

	case SERVO_ON:
		{
			_servo_on = true;
			std::cout << "Servo ON" << std::endl;
		}
		break;

	default:
		break;
	}

	return 0;
}


void HybridAutomatonManager::datanames(vector<string_type>& names, int channel)
{
	switch (channel) {
		case 1: names.push_back(_T("Torque"));
		case 2: names.push_back(_T("Q"));
		case 3: names.push_back(_T("Velocity"));
		case 4: names.push_back(_T("Error"));
		case 5: names.push_back(_T("Desired Q"));
		case 6: names.push_back(_T("Error Velocity"));
		case 7: names.push_back(_T("Reference Velocity"));
		case 8: names.push_back(_T("Reference Acceleration"));
		case 9: names.push_back(_T("Desired Velocity"));
	}
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
	else if (channel == 4)
	{
		dVector e = _activeMotionBehavior->getError();
		if (e.size() > 0) {
			for(int i = 0; i < _dof; ++i)
				data.push_back(e[i]);
		}
		else {
			for(int i = 0; i < _dof; ++i)
				data.push_back(666);
		}
	}
	else if (channel == 5)
	{
		dVector d = _activeMotionBehavior->getDesired();
		if (d.size() > 0) {
			for(int i = 0; i < _dof; ++i)
				data.push_back(d[i]);
		}
		else {
			for(int i = 0; i < _dof; ++i)
				data.push_back(666);
		}
	}
	else if (channel == 6)
	{
		dVector d = _activeMotionBehavior->getErrorDot();
		if (d.size() > 0) {
			for(int i = 0; i < _dof; ++i)
				data.push_back(d[i]);
		}
		else {
			for(int i = 0; i < _dof; ++i)
				data.push_back(666);
		}
	}
	else if (channel == 7)
	{
		dVector d = _activeMotionBehavior->getCurrentDotReference();
		if (d.size() > 0) {
			for(int i = 0; i < _dof; ++i)
				data.push_back(d[i]);
		}
		else {
			for(int i = 0; i < _dof; ++i)
				data.push_back(666);
		}
	}
	else if (channel == 8)
	{
		dVector d = _activeMotionBehavior->getCurrentDotDotReference();
		if (d.size() > 0) {
			for(int i = 0; i < _dof; ++i)
				data.push_back(d[i]);
		}
		else {
			for(int i = 0; i < _dof; ++i)
				data.push_back(666);
		}
	}
	else if (channel == 9)
	{
		dVector d = _activeMotionBehavior->getDesiredDot();
		if (d.size() > 0) {
			for(int i = 0; i < _dof; ++i)
				data.push_back(d[i]);
		}
		else {
			for(int i = 0; i < _dof; ++i)
				data.push_back(666);
		}
	}
}

void HybridAutomatonManager::onSetInterestFrame(const TCHAR* name, const HTransform& T)
{
}

rControlAlgorithm* CreateControlAlgorithm(rDC& rdc)
{
	return new HybridAutomatonManager(rdc);
}
