/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
*
* This library is commercial and cannot be redistributed, and/or modified
* WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
*/
#include "hybrid_automaton/include/hybrid_automaton_manager/AbstractHybridAutomatonManager.h"
#include "hybrid_automaton/include/hybrid_automaton_manager/AbstractHybridAutomatonManagerCmd.h"

#include "ros_bridge\include\msgs\String.h"
#include "ros_bridge\include\msgs\Transform.h"

#include <process.h>

unsigned __stdcall deserializeHybridAutomaton(void *udata)
{
	DeserializingThreadArguments* thread_args = static_cast<DeserializingThreadArguments*>(udata);
	HybridAutomaton* ha = NULL;
	try {
		ha = XMLDeserializer::createHybridAutomaton(thread_args->_string, thread_args->_sys, thread_args->_dT);
		//std::cout << "--------------- recieving hybrid automaton ------------------" << std::endl;
		//std::cout << ha->toStringXML() << std::endl;
	}
	catch(std::string e)
	{
		std::cerr << "[deserializeHybridAutomaton] ERROR while deserializing hybrid automaton!" << std::endl;
		std::cerr << e << std::endl;

		delete ha;
		delete thread_args;
		return 0;
	}

	WaitForSingleObject(*(thread_args->_deserialize_mutex), INFINITE);
	if(thread_args->_noQueue)
	{
		for(size_t i=0;i<thread_args->_deserialized_hybrid_automatons->size();i++)
		{
			delete thread_args->_deserialized_hybrid_automatons->at(i);
		}
		thread_args->_deserialized_hybrid_automatons->clear();	
	}

	thread_args->_deserialized_hybrid_automatons->push_back(ha);
	ReleaseMutex(*(thread_args->_deserialize_mutex));
	
	delete thread_args;

	return 0;
}

AbstractHybridAutomatonManager::AbstractHybridAutomatonManager(rDC rdc) 
:rControlAlgorithm(rdc)
, _sys(NULL)
, _ft_sensor(NULL)
, _blackboard(NULL)
, _activeMotionBehaviour(NULL)
, _dof(0)
, _servo_on(false)
, _hybrid_automaton(NULL)
, _qdot_filter(NULL)
, _noQueue(false)
{
	_deserialize_mutex = CreateMutex(0, FALSE, 0);
	if( !_deserialize_mutex ) {
		throw(std::string("[AbstractHybridAutomatonManager::HybridAutomatonManager] ERROR: Mutex was not created (_deserialize_mutex)!"));
	}
}

AbstractHybridAutomatonManager::~AbstractHybridAutomatonManager()
{
	if(_activeMotionBehaviour)
		delete _activeMotionBehaviour;

	if(_blackboard)
	{
		delete _blackboard;
		_blackboard = NULL;
	}

	if(_qdot_filter)
	{
		delete _qdot_filter;
	}	

	/*
	FREE_SYSTEMS();
	*/
}

void AbstractHybridAutomatonManager::init(int mode)
{
	_sys = LOAD_SYSTEM(_path, _aml, _T0, _q0);
	assert(_sys);

	//Find the robot device. This device is used to read position and write torques
	//to the robot
	_robot = findDevice(_T("ROBOT"));
	RASSERT(_robot != INVALID_RHANDLE);

	_ft_sensor = (rxDevice*)findDevice(_T("FT_SENSOR"));
	if (_ft_sensor)
		_sys->addDevice(_ft_sensor);

	_dof = _sys->jointDOF() + _sys->earthDOF() + _sys->constraintDOF();

	//Initialize variables
	_q.resize(_dof);
	_qOld.resize(_dof);
	_qdot.resize(_dof);
	_torque.resize(_dof);

	_q.zero();
	_qOld.zero();
	_torque.zero();
	_qdot.zero();

	_qdot_filter = new rxFilteredDerivative(this->_dT, 10.0*this->_dT, this->_sys->jdof());

	_activeMotionBehaviour = new MotionBehaviour(new Milestone(), new Milestone(), _sys);

}

void AbstractHybridAutomatonManager::activateBlackboard(std::string &rlab_host, int rlab_port, std::string &ros_host, int ros_port)
{
	_blackboard = RTBlackBoard::getInstance(rlab_host, rlab_port, ros_host, ros_port);
}

void AbstractHybridAutomatonManager::setHybridAutomaton(const std::string& _new_hybrid_automaton_str)
{
	DeserializingThreadArguments* thread_args = new DeserializingThreadArguments();
	thread_args->_sys = this->_sys;
	thread_args->_string = _new_hybrid_automaton_str;
	thread_args->_dT = this->_dT;
	thread_args->_noQueue = this->_noQueue;
	thread_args->_deserialize_mutex = &(this->_deserialize_mutex);
	thread_args->_deserialized_hybrid_automatons = &(this->_deserialized_hybrid_automatons);

	if (_beginthreadex(NULL, 0, deserializeHybridAutomaton, (void*)thread_args, 0, NULL) == 0)
	{
		std::cerr << "[AbstractHybridAutomatonManager::updateHybridAutomaton] Error creating thread to deserialize xml string!" << std::endl;
	}
}

void AbstractHybridAutomatonManager::setHybridAutomaton(HybridAutomaton* _new_hybrid_automaton)
{
	WaitForSingleObject(_deserialize_mutex, INFINITE);
	this->_deserialized_hybrid_automatons.push_back(_new_hybrid_automaton);
	ReleaseMutex(_deserialize_mutex);
}

void AbstractHybridAutomatonManager::update(const rTime& t)
{
	this->updateHybridAutomaton();

	this->updateMotionBehaviour(t);

	if (_activeMotionBehaviour)
	{
		Milestone* currentMilestone = (Milestone*)(_activeMotionBehaviour->getChild());
		currentMilestone->update(t);
	}

	rControlAlgorithm::update(t);

	this->updateBlackboard();
}

void AbstractHybridAutomatonManager::updateHybridAutomaton()
{
	if (_blackboard && _blackboard->isUpdated("update_hybrid_automaton"))
	{
		rlab::String* ha_xml = dynamic_cast<rlab::String*>(_blackboard->getData("update_hybrid_automaton"));
		try
		{
			DeserializingThreadArguments* thread_args = new DeserializingThreadArguments();
			thread_args->_sys = this->_sys;
			thread_args->_string = ha_xml->get();
			thread_args->_dT = this->_dT;
			thread_args->_noQueue= this->_noQueue;
			thread_args->_deserialize_mutex = &(this->_deserialize_mutex);
			thread_args->_deserialized_hybrid_automatons = &(this->_deserialized_hybrid_automatons);

			if (_beginthreadex(NULL, 0, deserializeHybridAutomaton, (void*)thread_args, 0, NULL) == 0)
			{
				std::cerr << "[AbstractHybridAutomatonManager::updateHybridAutomaton] Error creating thread to deserialize xml string!" << std::endl;
			}
		}
		catch(::std::string e)
		{
			std::cout << "[AbstractHybridAutomatonManager::updateHybridAutomaton] Error caught: " << std::endl;
			::std::cout << e << ::std::endl;
		}
	}
}

void AbstractHybridAutomatonManager::updateBlackboard()
{
	if(!_blackboard)
		return;

	_blackboard->setJointState("/joint_states", _sys->q(), _sys->qdot(), _torque);

	HTransform relative_transform;
	rxBody* end_effector = _sys->getUCSBody(_T("EE"), relative_transform);
	HTransform absolute_transform = end_effector->T() * relative_transform;
	_blackboard->setTransform("ee", absolute_transform, "base_link");
	
	_blackboard->step();
}

void AbstractHybridAutomatonManager::setNominalSystem(const TCHAR* path, const TCHAR* aml, const HTransform& T0, const dVector& q0)
{
	this->_path = path;
	this->_aml = aml;
	this->_T0 = T0;
	this->_q0 = q0;
}

void AbstractHybridAutomatonManager::setPeriod(const rTime& dT)
{
	this->_dT = dT;
}


void AbstractHybridAutomatonManager::_readDevices()
{	
	_qOld = _q;

	int r = readDeviceValue(_robot, &_q[0], sizeof(double) * _q.size(), 0);
	assert (r == sizeof(double) * _q.size());

}


void AbstractHybridAutomatonManager::_writeDevices()
{
	int w = writeDeviceValue(_robot, &_torque[0], sizeof(double) * _torque.size());
	assert(w == sizeof(double)*_torque.size());
}


void AbstractHybridAutomatonManager::_reflect()
{
	_sys->q(_q);
	_sys->qdot(_qdot);
}

void AbstractHybridAutomatonManager::_compute(const double& t)
{
	if (_servo_on)
	{
		_torque = _activeMotionBehaviour->update(t);
	}
	else 
	{
		_torque.zero();
	}

	setTorque(_torque);
}



void AbstractHybridAutomatonManager::_estimate()
{
	this->_qdot_filter->compute(this->_q, this->_qdot);
}

int AbstractHybridAutomatonManager::command(const short& cmd, const int& arg)
{
	switch (cmd)
	{
	case SERVO_ON:
		{
			_servo_on = !_servo_on;

			if(_servo_on)
				this->_qdot_filter->initialize(this->_q);

			std::cout << "[AbstractHybridAutomatonManager::command] Servo ON: " << _servo_on << std::endl;
		}
		break;
		
	case BLACKBOARD_ON:
		{
			std::map<int, std::string> domain_names;
			domain_names[URI_LOCAL] = "";
			domain_names[URI_BOTTOM_1]	= "130.149.238.178";
			domain_names[URI_BOTTOM_2]	= "130.149.238.179";
			domain_names[URI_BOTTOM_3]	= "130.149.238.180";
			domain_names[URI_LOHENGRIN] = "130.149.238.186";
			domain_names[URI_HASMA]		= "130.149.238.184";
			domain_names[URI_LEIBNIZ]	= "130.149.238.185";
            domain_names[URI_POSEIDON]	= "130.149.238.193";
			domain_names[URI_FIRSTMM]	= "130.149.238.220";
			domain_names[URI_SHOEFER]	= "130.149.238.182";
			domain_names[URI_TOP_1A]	= "130.149.238.188";
			domain_names[URI_TOP_1B]	= "130.149.238.189";
			domain_names[URI_ARIS]		= "130.149.238.194";
			domain_names[URI_ANGEL]		= "130.149.238.149";
			int bit_code = arg;
			
			int port = bit_code >> 16;
			bit_code &= ~(port << 16);
			
			int local_host = bit_code >> 8;
			bit_code &= ~(local_host << 8);

			int remote_host = bit_code;
			
			std::cout << "Enabled BlackBoard. Establishing connection between " << domain_names[local_host] << ":" << port << " and " << domain_names[remote_host] << ":" << port << "." << std::endl;	
			
			if (local_host != URI_LOCAL && domain_names[local_host].empty() ||
				remote_host != URI_LOCAL && domain_names[remote_host].empty())
			{
				std::cout << "WARNING! Unknown host: " << local_host << " or " << remote_host << "! (Will only establish unconnected BlackBoard)" << std::endl;
			}

			activateBlackboard(domain_names[local_host], port, domain_names[remote_host], port);
		}
		break;

	default:
		break;
	}

	return 0;
}


void AbstractHybridAutomatonManager::datanames(vector<string_type>& names, int channel)
{
	switch (channel) {
		case PLOT_TORQUE: names.push_back(_T("Torque"));
		case PLOT_Q: names.push_back(_T("Q"));
		case PLOT_VELOCITY: names.push_back(_T("Velocity"));
		case PLOT_EEFRAME: names.push_back(_T("EEPosition"));
		case PLOT_FT: names.push_back(_T("FT"));
	}
}

void AbstractHybridAutomatonManager::collect(vector<double>& data, int channel)
{
	if (channel == PLOT_TORQUE)
	{
		for(int i = 0; i < _dof; ++i)
			data.push_back(_torque[i]);
	}
	else if (channel == PLOT_Q)
	{
		for(int i = 0; i < _dof; ++i)
			data.push_back(_sys->q()[i]);
	}
	else if (channel == PLOT_VELOCITY)
	{
		for(int i = 0; i < _dof; ++i)
			data.push_back(_sys->qdot()[i]);
	}	
	else if (channel == PLOT_EEFRAME)
	{
		HTransform relative_transform;
		rxBody* end_effector = _sys->getUCSBody(_T("EE"), relative_transform);
		HTransform absolute_transform = end_effector->T() * relative_transform;
		for(int i = 0; i < 3; ++i)
			data.push_back(absolute_transform.r(i));
	}
	else if (channel == PLOT_FT)
	{
		if (_ft_sensor)
		{
			dVector currentFTMeasurement(6);
			_ft_sensor->readDeviceValue(&currentFTMeasurement[0], sizeof(double)*6);

			for (int i = 0; i < currentFTMeasurement.size(); ++i)
				data.push_back(currentFTMeasurement[i]);
		}
	}
}

void AbstractHybridAutomatonManager::onSetInterestFrame(const TCHAR* name, const HTransform& T)
{
}

/*
rControlAlgorithm* CreateControlAlgorithm(rDC& rdc)
{
	return new AbstractHybridAutomatonManager(rdc);
}
*/