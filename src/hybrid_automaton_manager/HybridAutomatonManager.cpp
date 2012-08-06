/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
*
* This library is commercial and cannot be redistributed, and/or modified
* WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
*/
#include "HybridAutomatonManager.h"
#include "HybridAutomatonManagerCmd.h"

#include "XMLDeserializer.h"

#include "msgs\String.h"

#include <process.h>

//#define NOT_IN_RT

#ifdef DRAW_HYBRID_AUTOMATON

rID HybridAutomatonManager::drawLine(const rMath::Displacement &start, const rMath::Displacement &end, const rID &drawID, const rColor &color)
{
	rID id = drawID;
	if (drawID != INVALID_RID) // update 
	{
		rCustomDrawInfo info;
		info.flag = CUSTOM_DRAW_UPDATE_POSE;
		info.id = id;

		info.drawType = 0x20;// update 

		info.T.r = start;

		rMath::Vector3D dir = end - start;
		double length = dir.Norm();

		dir.Normalize();
		rMath::Vector3D zAxis(0.0, 0.0, 1.0);
		rMath::Vector3D axis = zAxis.Cross(dir);
		double half = acos(zAxis.Dot(dir))*0.5;
		info.T.R.Set(cos(half), axis[0]*sin(half), axis[1]*sin(half), axis[2]*sin(half) );

		info.scale[0] = 1.0;
		info.scale[1] = 1.0;
		info.scale[2] = length;

		_physics_world->customDraw(info);
	}
	else
	{
		rCustomDrawInfo info;
		info.flag = CUSTOM_DRAW_NEW;
		info.id = INVALID_RID;

		info.name = "";

		info.drawType = CUSTOM_DRAW_POLYLINE;
		info.lineWidth = 3;
		info.fill = 0;

		info.robotId = 0;
		info.bodyId = 0;

		info.vertexCnt = 2;

		info.T.r = start;

		rMath::Vector3D dir = end - start;
		double length = dir.Norm();

		dir.Normalize();
		rMath::Vector3D zAxis(0.0, 0.0, 1.0);
		rMath::Vector3D axis = zAxis.Cross(dir);
		double half = acos(zAxis.Dot(dir))*0.5;
		info.T.R.Set(cos(half), axis[0]*sin(half), axis[1]*sin(half), axis[2]*sin(half) );

		info.scale[0] = 1.0;
		info.scale[1] = 1.0;
		info.scale[2] = length;

		info.vertexPoints.push_back(rMath::Vector3D(0, 0, 0));
		info.vertexPoints.push_back(rMath::Vector3D(0, 0, 1));

		rMath::Vector3D normal;
		normal.Set(0.0, 0.0, 1.0);
		info.vertexNormals.push_back(normal);
		info.vertexNormals.push_back(normal);

		info.vertexColors.push_back(color);
		info.vertexColors.push_back(color);

		info.color[0] = color.r;
		info.color[1] = color.g;
		info.color[2] = color.b;
		info.color[3] = color.a;

		info.param[0] = info.param[1] = info.param[2] = 0.0f;
		id = _physics_world->customDraw(info);
	}

	return id;
}

rID path[10] = {INVALID_RID,INVALID_RID,INVALID_RID,INVALID_RID,INVALID_RID,INVALID_RID,INVALID_RID,INVALID_RID,INVALID_RID,INVALID_RID};
#endif

unsigned __stdcall deserializeHybridAutomaton(void *udata)
{
	DeserializingThreadArguments* thread_args = static_cast<DeserializingThreadArguments*>(udata);
	HybridAutomaton* ha = NULL;
	try {
		ha = XMLDeserializer::createHybridAutomaton(thread_args->_string, thread_args->_robot, thread_args->_dT);
		std::cout << "--------------- recieving hybrid automaton ------------------" << std::endl;
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
		throw(std::string("[HybridAutomatonManager::HybridAutomatonManager] ERROR: Mutex was not created (_deserialize_mutex)!"));
	}
}

HybridAutomatonManager::~HybridAutomatonManager()
{
	delete _activeMotionBehavior;

	if(_blackboard)
		delete _blackboard;

	//delete _defaultMotionBehavior;

	FREE_SYSTEMS();
}

void HybridAutomatonManager::init(int mode)
{
	_robot = LOAD_SYSTEM(_path, _aml, _T0, _q0);
	assert(_robot);

	//std::cerr << " ROBOT " << _robot << std::endl;

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

	_defaultMotionBehavior = new MotionBehaviour(new Milestone(), new Milestone(),_robot);
	/*rxJointController* jc = new rxJointController(_robot,_dT);
	jc->addPoint(_robot->q(),5,true);
	jc->setGain(50.0,200.0);
	_defaultMotionBehavior->addController(jc,false);*/
	_activeMotionBehavior = _defaultMotionBehavior;
	//_activeMotionBehavior->activate();
}

void HybridAutomatonManager::activateBlackboard(std::string &rlab_host, int rlab_port, std::string &ros_host, int ros_port)
{
	_blackboard = RTBlackBoard::getInstance(rlab_host, rlab_port, ros_host, ros_port);
}

bool HybridAutomatonManager::isBlackboardActive() const
{
	return (_blackboard != NULL);
}

void HybridAutomatonManager::setCollisionInterface(CollisionInterface* collision_interface)
{
		CollisionInterface::instance = collision_interface;
}

void HybridAutomatonManager::setPhysicsWorld(rxWorld* physics_world)
{
		_physics_world = physics_world;
}

void HybridAutomatonManager::setHybridAutomaton(std::string _new_hybrid_automaton_str, CollisionInterface* collision_interface)
{
	if(!CollisionInterface::instance)
	{
		CollisionInterface::instance = collision_interface;
	}
	DeserializingThreadArguments* thread_args = new DeserializingThreadArguments();
	thread_args->_robot = this->_robot;
	thread_args->_string = _new_hybrid_automaton_str;
	thread_args->_dT = this->_dT;
	thread_args->_deserialize_mutex = &(this->_deserialize_mutex);
	thread_args->_deserialized_hybrid_automatons = &(this->_deserialized_hybrid_automatons);

	if (_beginthreadex(NULL, 0, deserializeHybridAutomaton, (void*)thread_args, 0, NULL) == 0)
	{
		std::cerr << "[HybridAutomatonManager::updateHybridAutomaton] Error creating thread to deserialize xml string!" << std::endl;
	}
}

void HybridAutomatonManager::setHybridAutomaton(HybridAutomaton* _new_hybrid_automaton)
{
	WaitForSingleObject(_deserialize_mutex, INFINITE);
	this->_deserialized_hybrid_automatons.push_back(_new_hybrid_automaton);
	ReleaseMutex(_deserialize_mutex);
}

void HybridAutomatonManager::update(const rTime& t)
{
	this->updateHybridAutomaton();

	rControlAlgorithm::update(t);

	this->updateBlackboard();
}

void HybridAutomatonManager::updateBlackboard()
{
	if(!this->isBlackboardActive())
		return;

	_blackboard->setJointState("joint_state", _q_BB, _qdot_BB, _torque_BB);

	HTransform relative_transform;
	rxBody* end_effector = _robot->getUCSBody(_T("EE"), relative_transform);
	HTransform absolute_transform = end_effector->T() * relative_transform;
	_blackboard->setTransform("ee", absolute_transform, "base_link");
	
	_blackboard->step();
}

void HybridAutomatonManager::updateHybridAutomaton()
{
	if(!this->isBlackboardActive())
		return;

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
				std::cerr << "[HybridAutomatonManager::updateHybridAutomaton] Error creating thread to deserialize xml string!" << std::endl;
			}
		}
		catch(::std::string e)
		{
			std::cout << "[HybridAutomatonManager::updateHybridAutomaton] Error caught: " << std::endl;
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
		}else{
			torque[i] = 0;
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
    Milestone* childMs=(Milestone*)(_activeMotionBehavior->getChild());

	if( !_deserialized_hybrid_automatons.empty() )
	{
		if (WaitForSingleObject(_deserialize_mutex, 0) != WAIT_FAILED && !_deserialized_hybrid_automatons.empty())
		{
			delete _hybrid_automaton;
			_hybrid_automaton = _deserialized_hybrid_automatons.front();
			_deserialized_hybrid_automatons.pop_front();
			
			std::cout << "[HybridAutomatonManager::_compute] INFO: New Hybrid Automaton" << std::endl;
			Milestone* tmpMilestone = _hybrid_automaton->getStartNode();
			_activeMotionBehavior->deactivate();
			_activeMotionBehavior = _hybrid_automaton->getNextMotionBehaviour(tmpMilestone);
			_activeMotionBehavior->activate();
#ifdef NOT_IN_RT
			std::cout << _activeMotionBehavior->toStringXML() << ::std::endl;
			_activeMotionBehavior->print();
			std::cout << "Number of edges: " << _hybrid_automaton->getEdgeNumber() << std::endl;
			_hybrid_automaton->__printMatrix();
#endif
			ReleaseMutex(_deserialize_mutex);
#ifdef DRAW_HYBRID_AUTOMATON
			for(int i = 0; i < 10; i++)
			{
				if(!_hybrid_automaton->outgoingEdges(*tmpMilestone).empty())
				{
					Milestone* tmpMilestone2 = _hybrid_automaton->outgoingEdges(*tmpMilestone)[0]->getChild();
					// TODO: Handlepoints should be part of Milestone! All children have them....?
					cout << "DrawLine" << std::endl;
					path[i] = drawLine(((OpSpaceMilestone*)tmpMilestone)->getHandlePoint(0),((OpSpaceMilestone*)tmpMilestone2)->getHandlePoint(0),path[i],BLUE);
					tmpMilestone = tmpMilestone2;
				}
				else
				{
					_physics_world->eraseCustomDraw(path[i]);
				}

			}
#endif
		}
	}
	else if(childMs->hasConverged(_robot) ){
		if (_hybrid_automaton && _hybrid_automaton->getNextMotionBehaviour(childMs) != NULL) {
			std::cout << "[HybridAutomatonManager::_compute] INFO: Switching controller" << std::endl;
			_activeMotionBehavior->deactivate();
			_activeMotionBehavior = _hybrid_automaton->getNextMotionBehaviour(childMs);
			_activeMotionBehavior->activate();

#ifdef NOT_IN_RT 
			std::cout << _activeMotionBehavior->toStringXML() << ::std::endl;
			_activeMotionBehavior->print();
			_hybrid_automaton->__printMatrix();
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
	case PAUSE:
	case RESUME:
		break;

	case SERVO_ON:
		{
			_servo_on = !_servo_on;
			std::cout << "[HybridAutomatonManager::command] Servo ON: " << _servo_on << std::endl;
		}
		break;
		
	case BLACKBOARD_ON:
		{
			std::map<int, std::string> domain_names;
			domain_names[URI_LOCAL] = "";
			domain_names[URI_BOTTOM_1] = "130.149.238.178";
			domain_names[URI_BOTTOM_2] = "130.149.238.179";
			domain_names[URI_BOTTOM_3] = "130.149.238.180";
			domain_names[URI_LOHENGRIN] = "130.149.238.186";
			domain_names[URI_HASMA] = "130.149.238.184";
			domain_names[URI_LEIBNIZ] = "130.149.238.185";
            domain_names[URI_POSEIDON] = "130.149.238.193";

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
	if (channel == PLOT_TORQUE)
	{
		for(int i = 0; i < _dof; ++i)
			data.push_back(_torque[i]);
	}
	else if (channel == PLOT_Q)
	{
		for(int i = 0; i < _dof; ++i)
			data.push_back(_q[i]);
	}
	else if (channel == PLOT_VELOCITY)
	{
		for(int i = 0; i < _dof; ++i)
			data.push_back(_qdot[i]);
	}
	else if (channel == PLOT_ERROR)
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
	else if (channel == PLOT_DESIRED_Q)
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
	else if (channel == PLOT_ERROR_VELOCITY)
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
	else if (channel == PLOT_DESIRED_VELOCITY)
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
