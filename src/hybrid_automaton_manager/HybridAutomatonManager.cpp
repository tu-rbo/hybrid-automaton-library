/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
*
* This library is commercial and cannot be redistributed, and/or modified
* WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
*/
#include "HybridAutomatonManager.h"
#include "HybridAutomatonManagerCmd.h"

#include "..\..\..\applications\elastic_roadmap\src\ERMDemoDefines.h"

#include "XMLDeserializer.h"

#include "msgs\String.h"

#include <process.h>

//#define NOT_IN_RT

#define SENSOR_FREQUENCY 0.1 // seconds
//#define SINGULARITY_HANDLING

//CustomDrawManager drawDebug;

void HybridAutomatonManager::drawLine(const rMath::Displacement &start, const rMath::Displacement &end, drawLists index, const rColor &color, const char lineWidth)
{
	rCustomDrawInfo info;
	info.flag = CUSTOM_DRAW_NEW;
	info.id = INVALID_RID;

	info.name = "";

	info.drawType = CUSTOM_DRAW_POLYLINE;
	info.lineWidth = lineWidth;
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
	axis.Normalize();
	double half = acos(zAxis.Dot(dir))*0.5;
	info.T.R.Set(cos(half), axis[0]*sin(half), axis[1]*sin(half), axis[2]*sin(half) );
	/*Vector3D z_axis =  dir;
	Vector3D y_axis = z_axis.Cross(Vector3D(1,0,0));
	y_axis.Normalize();
	Vector3D x_axis = z_axis.Cross(y_axis);
	x_axis.Normalize();
	info.T.R.Set(x_axis[0],x_axis[1],x_axis[2],y_axis[0],y_axis[1],y_axis[2],z_axis[0],z_axis[1],z_axis[2],true);*/

	info.scale[0] = 1.0;
	info.scale[1] = 1.0;
	info.scale[2] = (float)length;

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
	if(!this->_request_draw[index])
	{
		_draw_objects[index].push_back(info);
	}
	else
	{
		//cout << "requested not drawn!!" << endl;
	}
}

void HybridAutomatonManager::drawMilestones(const PostureMilestone* ms, drawListsM index)
{
	_draw_objects_m[index].push_back(ms->getCConfiguration());
	return;
}

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
//, _physics_world(NULL)
, _criterion(NULL)
, _t_old(0.0)
, max_ori_error_norm(0.0)
,minObstDistanceBase(999.0)
,minObstDistanceEE(999.0)
{
	max_ori_error[0] = 0.0;
	max_ori_error[1] = 0.0;
	max_ori_error[2] = 0.0;
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
	rxJointController* jc = new rxJointController(_robot,_dT);
	jc->addPoint(_robot->q(),5,true);
	jc->setGain(20.0,600.0);
	jc->setGain((size_t) 0, 10.0, 900.0); 
	jc->setGain((size_t) 1, 20.0, 2500.0); 
	jc->setGain((size_t) 2, 5.0, 600.0); 
	jc->setGain((size_t) 3, 2.0, 500.0); 
	jc->setGain((size_t) 4, 0.5, 50.0); 
	jc->setGain((size_t) 5, 0.5, 50.0); 
	jc->setGain((size_t) 6, 0.05, 40.0);
	_defaultMotionBehavior->addController(jc,false);
	_activeMotionBehavior = _defaultMotionBehavior;
	_activeMotionBehavior->activate();
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
/*
void HybridAutomatonManager::setPhysicsWorld(rxWorld* physics_world)
{
		_physics_world = physics_world;
		drawDebug.setPhysicsWorld(physics_world);
}*/

void HybridAutomatonManager::setLocalDecisionCriterion(LocalDecisionCriterion* criterion)
{
	this->_criterion = criterion;
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

bool singularity_wait = false;
vector<pair<std::string,std::string>> bad_edges;
void HybridAutomatonManager::_compute(const double& t)
{
#ifdef SINGULARITY_HANDLING
	for(int i = 0; i < _qdot.size(); i++)
	{
		if(abs(_qdot[i]) > 10.0)
		{
			//std::cout << "** Current MB is bad!! Local Minimum! Kernel Panic! General Failure!!" << std::endl;
			Milestone* parentMs=(Milestone*)(_activeMotionBehavior->getParent());
			Milestone* childMs=(Milestone*)(_activeMotionBehavior->getChild());
			//std::cout << "** Blocking Edge: " << parentMs->getName() << " to " << childMs->getName() <<std::endl;
			//bad_edges.push_back(pair<std::string,std::string>(childMs->getName(), parentMs->getName()));
			//_activeMotionBehavior->deactivate();
			/*if(_hybrid_automaton->incomingEdges(parentMs).size() > 0)
			{
				_activeMotionBehavior = (MotionBehaviour*)_hybrid_automaton->incomingEdges(parentMs).front();
			}
			else
			{
				cout << "no staying option!" << endl;
				_activeMotionBehavior = _hybrid_automaton->getNextMotionBehaviour(parentMs, _criterion, &bad_edges);
			}*/
			singularity_wait = true;
			//cout << "entering wait mode..." << endl;
			_activeMotionBehavior->waitMode(); /// makes active motion behaviour wait at current ee pos and config!
			//_activeMotionBehavior->
			//_activeMotionBehavior->activate();
		}
	}
#endif


	_torque = _activeMotionBehavior->update(t);	
    Milestone* childMs=(Milestone*)(_activeMotionBehavior->getChild());
	bool drawPath = false;

	if( !_deserialized_hybrid_automatons.empty() )
	{
		if (WaitForSingleObject(_deserialize_mutex, 0) != WAIT_FAILED && !_deserialized_hybrid_automatons.empty())
		{
			delete _hybrid_automaton;
			_hybrid_automaton = _deserialized_hybrid_automatons.front();

			_deserialized_hybrid_automatons.pop_front();
			
			//std::cout << "[HybridAutomatonManager::_compute] INFO: New Hybrid Automaton" << std::endl;
			Milestone* tmpMilestone = _hybrid_automaton->getStartNode();
			_activeMotionBehavior->deactivate();
			_activeMotionBehavior = _hybrid_automaton->getNextMotionBehaviour(tmpMilestone, _criterion, &bad_edges);
			_activeMotionBehavior->activate();
#ifdef NOT_IN_RT
			std::cout << _activeMotionBehavior->toStringXML() << ::std::endl;
			_activeMotionBehavior->print();
			std::cout << "Number of edges: " << _hybrid_automaton->getEdgeNumber() << std::endl;
			_hybrid_automaton->__printMatrix();
#endif
			ReleaseMutex(_deserialize_mutex);
#ifdef DRAW_HYBRID_AUTOMATON

			if(demo::useESP)
			{
				//TODO : implement ha->getCurrentGoal()
				MDPNode* hack_goal = (MDPNode*)_hybrid_automaton->getSortedOutgoingEdges(_hybrid_automaton->getStartNode()).front()->getChild();
				std::vector<const MDPNode*> milestones = _hybrid_automaton->getShortestPath(_hybrid_automaton->getStartNode(),hack_goal,1.0);
			}
			else
			{
				std::vector<const MDPNode*> milestones;
				Milestone* ms = _hybrid_automaton->getStartNode();
				milestones.push_back(ms);
				MotionBehaviour* mb = _hybrid_automaton->getNextMotionBehaviour(ms,_criterion, &bad_edges);
				while(mb && mb->getChild() != ms)
				{
					ms = (Milestone*)mb->getChild();
					milestones.push_back(ms);
					mb = _hybrid_automaton->getNextMotionBehaviour(ms,_criterion, &bad_edges);
				}
			}

			int count = 0;
			int i = 0;
			const OpSpaceMilestone* ms_old = NULL;
			for(std::vector<const MDPNode*>::iterator mit = milestones.begin(); mit != milestones.end() && i < 20; mit++)
			{
				const PostureMilestone* ms = (const PostureMilestone*)(*mit);
				/*HTransform ht;
				ht.Reset();
				ht.r = ms->getHandlePoint(0);
				//cout << "draw " << ms->getName() << endl;
				ht.Print();
				config[i] = drawDebug.drawCylinder(config[i],ht,0.35,0.78,rColor(1.0,0.0,0.0,0.3));*/
				drawMilestones(ms,ePATH_MILESTONES);
				i++;
				if(ms_old != NULL)
				{
					Displacement pos1, pos2;
					if(demo::baseActuated)
					{
						pos1 = ms_old->getHandlePoint(3);
						pos2 = ms->getHandlePoint(3);
						pos1[2] += 0.5;
						pos2[2] += 0.5;
						
					}
					else
					{
						pos1 = ms_old->getHandlePoint(2);
						pos2 = ms->getHandlePoint(2);
					}
					drawLine(pos1,pos2,ePATH);

					
				}
				ms_old = ms;
			}
	
			setRequestDraw(ePATH,true);
			setRequestDrawM(ePATH_MILESTONES,true);
#endif
			// draw current choice + next decision
			singularity_wait = false;
			drawPath = true;
			_t_old = t;
		}
	}
	else if(childMs->hasConverged(_robot) && !singularity_wait)
	{
		if(_hybrid_automaton)
		{
			MotionBehaviour* nextMotion = _hybrid_automaton->getNextMotionBehaviour(childMs,_criterion, &bad_edges);
			if (nextMotion != NULL && nextMotion != _activeMotionBehavior) 
			{
				std::cout << "[HybridAutomatonManager::_compute] INFO: Switching controller" << std::endl;
				std::cout << _activeMotionBehavior->getParent() << " versus " << nextMotion->getParent() << std::endl;
				_activeMotionBehavior->deactivate();
				_activeMotionBehavior = nextMotion; // if criterion is not set the old behaviour remains
				_activeMotionBehavior->activate();
				drawPath = true;
				_t_old = t;

	#ifdef NOT_IN_RT 
				std::cout << _activeMotionBehavior->toStringXML() << ::std::endl;
				_activeMotionBehavior->print();
				_hybrid_automaton->__printMatrix();
	#endif
			}
		}
	}
	else if(_criterion && (t - _t_old > SENSOR_FREQUENCY)){
		_t_old = t;
		// make new local decision:
		MotionBehaviour* newChoice = _hybrid_automaton->getNextMotionBehaviour((Milestone*)_activeMotionBehavior->getParent(),_criterion, &bad_edges);
		if(newChoice != _activeMotionBehavior)
		{
			std::cout << "[HybridAutomatonManager::_compute] INFO: Switching controller due to local decision!" << std::endl;
			_activeMotionBehavior->deactivate();
			_activeMotionBehavior = newChoice;
			_activeMotionBehavior->activate();
			drawPath = true;
#ifdef DRAW_LOCAL_DECISION
			HTransform ht;
			ht.Reset();
			PostureMilestone* ms = ((PostureMilestone*) _activeMotionBehavior->getChild());
			drawMilestones(ms,eLD_MILESTONES);
			setRequestDrawM(eLD_MILESTONES,true);
			cout << "Request LD!" << endl;
		}
		if(drawPath)
		{
			// a new path arrived, if the LD is the same as the path it should be deleted...
			setRequestDrawM(eLD_MILESTONES,true);
#endif
		}

	}
#ifdef DRAW_LOCAL_DECISION
	if(drawPath)
	{
		//drawDebug.reset(drawGroups::PATH);
		//drawDebug.clearUnused(drawGroups::PATH);
		const OpSpaceMilestone* ms1 = (const OpSpaceMilestone*)_activeMotionBehavior->getParent();		
		std::vector<const MDPEdge*> outgoing = _hybrid_automaton->getSortedOutgoingEdges(ms1);
		int count = 0;
		for(std::vector<const MDPEdge*>::iterator eit = outgoing.begin(); eit != outgoing.end(); eit++)
		{
			const OpSpaceMilestone* ms2 = (const OpSpaceMilestone*)(*eit)->getChild();
			Displacement pos1,pos2;
			if(demo::baseActuated)
			{
				rxBody* XR     = _robot->findBody(_T("Body"));
				pos1 = XR->T().r;
				pos2 = ms2->getHandlePoint(3);
				pos1[2] += 0.5;
				pos2[2] += 0.5;
			}
			else
			{
				rxBody* ee     = _robot->findBody(_T("Body7"));
				pos1 = ee->T().r;
				pos2 = ms2->getHandlePoint(2);
			}

			if(ms2 != _activeMotionBehavior->getChild()) // dont draw return path (overlapping)
			{
				drawLine(pos1,pos2,eRAYS,rColor(0.0,0.0,0.0));
			}
			else
			{
				drawLine(pos1,pos2,eRAYS,rColor(1.0,1.0,1.0));
				break;
			}
			count++;
			if(count == 10)
				break;
		}
		setRequestDraw(eRAYS,true);
	}
#endif
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
			domain_names[URI_FIRSTMM] = "130.149.238.220";

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

dVector e_old_ori; // smoothing switches
dVector e_old_line;
#define TASK_SIZE 2
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
	else if (channel == PLOT_ORI_TASK_ERROR)
	{
		dVector e = _activeMotionBehavior->getOriTaskError();
		if (e.size() > 0) {

			bool zero = true;
			for(int i = 0; i < e.size(); ++i)
			{
				if(e[i] != 0)
				{
					zero = false;
				}
			}
			if(zero)
			{
				if(e_old_ori.size() > 0)
				{
					for(int i = 0; i < e_old_ori.size(); ++i)
						data.push_back(e_old_ori[i]);
				}
				else
				{
					for(int i = 0; i < 3; ++i)
						data.push_back(0);
				}
			}
			else
			{
				for(int i = 0; i < e.size(); ++i)
				{
					data.push_back(e[i]);
					if(e[i] > max_ori_error[i])
					{
						max_ori_error[i] = e[i];
					}
				}
				if(e.norm() > max_ori_error_norm)
				{
					max_ori_error_norm = e.norm();
				}
				e_old_ori = e;
			}
		}
		else {
			for(int i = 0; i < 3; ++i)
				data.push_back(0);
		}
	}
	else if (channel == PLOT_LINE_TASK_ERRROR)
	{
		dVector e = _activeMotionBehavior->getLineTaskError();
		if (e.size() > 0) {
			bool zero = true;
			for(int i = 0; i < e.size(); ++i)
			{
				if(e[i] != 0)
				{
					zero = false;
				}
			}
			if(zero)
			{
				if(e_old_line.size() > 0)
				{
					for(int i = 0; i < e_old_line.size(); ++i)
						data.push_back(e_old_line[i]);
				}
				else
				{
					for(int i = 0; i < TASK_SIZE; ++i)
						data.push_back(666);
				}
			}
			else
			{
				for(int i = 0; i < e.size(); ++i)
					data.push_back(e[i]);
				e_old_line = e;
			}
		}
		else {
			if(e_old_line.size() > 0)
			{
				for(int i = 0; i < e_old_line.size(); ++i)
					data.push_back(e_old_line[i]);
			}
			else
			{
				for(int i = 0; i < TASK_SIZE; ++i)
					data.push_back(0);
			}
		}
	}

	pair<double,double> temp = _activeMotionBehavior->getDistanceToNearestObst();
	if(temp.first < minObstDistanceBase)
	{
		minObstDistanceBase = temp.first;
	}
	if(temp.second < minObstDistanceEE)
	{
		minObstDistanceEE = temp.second;
	}
}

void HybridAutomatonManager::onSetInterestFrame(const TCHAR* name, const HTransform& T)
{
}

rControlAlgorithm* CreateControlAlgorithm(rDC& rdc)
{
	return new HybridAutomatonManager(rdc);
}

std::vector<rCustomDrawInfo> HybridAutomatonManager::getDrawObjects(drawLists index)
{
	std::vector<rCustomDrawInfo> copy;
	for(unsigned int i = 0; i < _draw_objects[index].size(); i++)
	{
		copy.push_back(_draw_objects[index][i]);
	}
	_draw_objects[index].clear();
	return copy;
}

std::vector<dVector> HybridAutomatonManager::getDrawMilestones(drawListsM index)
{
	std::vector<dVector> copy;
	for(unsigned int i = 0; i < _draw_objects_m[index].size(); i++)
	{
		copy.push_back(_draw_objects_m[index][i]);
	}
	_draw_objects_m[index].clear();
	return copy;
}
