/*
 * Copyright 2015-2017, Robotics and Biology Lab, TU Berlin
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the 
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "hybrid_automaton/JumpCondition.h"
#include "hybrid_automaton/HybridAutomaton.h"

namespace ha {

	JumpCondition::JumpCondition():
		_goalSource(CONSTANT),
		_jump_criterion(NORM_L1),
		_epsilon(0.0),
        _is_goal_relative(false),
        _negate(false)
	{

	}

	JumpCondition::~JumpCondition()
	{

	}

	JumpCondition::JumpCondition(const ha::JumpCondition &jc)
	{
		this->_goalSource = jc._goalSource;
		this->_goal = jc._goal;
		this->_controller = jc._controller;
		this->_sensor = jc._sensor;
		this->_jump_criterion = jc._jump_criterion;
		this->_norm_weights = jc._norm_weights;
		this->_epsilon = jc._epsilon;
		this->_ros_topic_goal_name = jc._ros_topic_goal_name;
		this->_ros_topic_goal_type = jc._ros_topic_goal_type;
        this->_negate=jc._negate;
	}

	void JumpCondition::initialize(const double& t) 
	{
		this->_sensor->initialize(t); 
		if (this->_goalSource == ROSTOPIC) {
			_system->subscribeToROSMessage(_ros_topic_goal_name);
		}
		if (this->_goalSource == ROSTOPIC_TF) {
			_system->subscribeToTransform(_ros_tf_goal_child, _ros_tf_goal_parent);
		}
	}

	void JumpCondition::terminate() 
	{
		this->_sensor->terminate();
	}

	void JumpCondition::step(const double& t) 
	{
		this->_sensor->step(t);
	}

	bool JumpCondition::isActive() const 
	{
		if (!this->_sensor->isActive()) {
			return false;
		}

		::Eigen::MatrixXd current = this->_sensor->getCurrentValue();
		::Eigen::MatrixXd initial = this->_sensor->getInitialValue();
		::Eigen::MatrixXd desired = this->getGoal();

		if(desired.cols()== 0 && desired.rows() ==0)
		{
			static int not_yet_ctr = 0;

			if(not_yet_ctr++%500 == 0)
			{
				HA_INFO("JumpCondition.isActive","Goal is not set yet. Returning false. Are you running a BB controller?");
			}
			return false;
		}

		if(desired.rows() != current.rows() || desired.cols() != current.cols())
		{
			HA_THROW_ERROR("JumpCondition.isActive", "Dimension mismatch in sensor and goal - sensor: "
			<<current.rows()<<"x"<<current.cols()<<", current: "<<desired.rows()<<"x"<<desired.cols()<<"!"
			<< " Sensor Type: " << this->_sensor->getType());
		}
		 
		if(initial.rows() != current.rows() || initial.cols() != current.cols())
		{
			HA_THROW_ERROR("JumpCondition.isActive", "Dimension mismatch in initial and current sensor values: "
			<<current.rows()<<"x"<<current.cols()<<", current: "<<initial.rows()<<"x"<<initial.cols()<<"!");
		}
		 
		if(this->_is_goal_relative)
		{
            if (!_negate){
                return (this->_computeJumpCriterion(this->_sensor->getRelativeCurrentValue(), desired) <= _epsilon);
            } else {
                return (this->_computeJumpCriterion(this->_sensor->getRelativeCurrentValue(), desired) > _epsilon);
            }
		}else{
            if (!_negate){
                return (this->_computeJumpCriterion( this->_sensor->getCurrentValue(), desired) <= _epsilon);
            } else {
                return (this->_computeJumpCriterion( this->_sensor->getCurrentValue(), desired) > _epsilon);
            }

		}
	}

	double JumpCondition::_computeJumpCriterion(const ::Eigen::MatrixXd& x, const ::Eigen::MatrixXd& y) const
	{
		//First check if weights are given - otherwise use default weights (=1.0)
		::Eigen::MatrixXd weights;
		if(_norm_weights.rows() == 0)
		{
			weights.resize(x.rows(), x.cols());
			weights.setConstant(1.0);
		}
		else
		{
			weights = _norm_weights;
		}
		
		double ret = 0;
		switch(_jump_criterion) {
			case NORM_L1: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{ 
						ret += weights(i,j) * fabs(x(i,j) - y(i,j));
					}
				}
				break;
			
			case NORM_L2: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						ret += weights(i,j) * pow(fabs(x(i,j) - y(i,j)),2);
					}
				}
				ret = sqrt(ret);
				break;

			case NORM_L_INF: 
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						ret = std::max(ret, weights(i,j) * fabs(x(i,j) - y(i,j)));
					}
				}
				break;
			case NORM_ROTATION:
				{
					if(x.cols() != 3 || x.rows() != 3|| y.cols() != 3|| x.rows() != 3)
					{
						HA_THROW_ERROR("JumpCondition._computeJumpCriterion", "Either the goal or the current value is not a rotation matrix." <<
							"x dim = " << x.cols()  << "x" <<  x.rows()<<". y dim = " << y.cols()  << "x" <<  y.rows());
					}

					//Compute relative Rotation from x0 to xf
					Eigen::Matrix3d xRot = x.inverse()*y;

					//Conver to angle axis to get angle
					Eigen::AngleAxisd xRotAA;
					xRotAA = xRot;

					ret = xRotAA.angle();

					//HA_INFO("JumpCondition._computeJumpCriterion", "Angle between goal and current: " << ret);
				}
				break;
			case NORM_TRANSFORM:
				{
					if(x.cols() != 4 || x.rows() != 4|| y.cols() != 4|| x.rows() != 4)
					{
						HA_THROW_ERROR("JumpCondition._computeJumpCriterion", "Either the goal or the current value is not a homogeneous transformation matrix." <<
							"x dim = " << x.cols()  << "x" <<  x.rows()<<". y dim = " << y.cols()  << "x" <<  y.rows());
					}

					//Compute relative Rotation from x0 to xf
					Eigen::Matrix3d xRot = x.block(0,0,3,3).inverse()*y.block(0,0,3,3);

					//Conver to angle axis to get angle
					Eigen::AngleAxisd xRotAA;
					xRotAA = xRot;

					static int rate_print = 0;

					double angle_diff = xRotAA.angle();

					
					
					Eigen::Vector3d xDisp = x.block(0,3,3,1)-y.block(0,3,3,1);
					double disp_diff = xDisp.norm();

					if(_norm_weights.cols() !=1 || _norm_weights.rows() !=2)
					{
						//HA_THROW_ERROR("JumpCondition._computeJumpCriterion", "We need 2 weights for the estimation of the norm between 2 HTransforms, " <<
						//	"one for rotation and one for translation. weights dim = " <<weights.cols()<<" " << weights.rows());
						weights = Eigen::MatrixXd(2,1);
						weights << 0.1, 1.;
					}
					ret = weights(0,0) * angle_diff + weights(1,0) * disp_diff;

					if(rate_print++%500 == 0)
					{
					  //HA_INFO("JumpCondition._computeJumpCriterion", "Angle between goal and current: " << angle_diff);
					  //HA_INFO("JumpCondition._computeJumpCriterion", "Distance between goal and current: " << disp_diff);
					  //HA_INFO("JumpCondition._computeJumpCriterion", "Weighted difference: " << ret << " (epsilon=" << _epsilon << ")");
					}
				}
				break;

			case THRESH_UPPER_BOUND:
				
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						//std::cout << "upper bound" << weights(i,j)*(y(i,j) - x(i,j)) << std::endl;
						ret = std::max(ret, weights(i,j)*(y(i,j) - x(i,j)));
					}
				}
				break;

			case THRESH_LOWER_BOUND:
				for(int i = 0; i<x.rows(); i++)
				{
					for(int j = 0; j<y.cols(); j++)
					{
						//std::cout << weights(i,j)*(x(i,j) - y(i,j)) << std::endl;
						ret = std::max(ret, weights(i,j)*(x(i,j) - y(i,j)));
					}
				}
				break;
			default:
				HA_THROW_ERROR("JumpCondition._computeJumpCriterion", "Not Implemented: unknown jump criterion");
		}

		return ret;
	}

	void JumpCondition::setControllerGoal(const Controller::ConstPtr& controller)
	{
		_goalSource = CONTROLLER;
		_controller = controller;
	}

	void JumpCondition::setConstantGoal(const ::Eigen::MatrixXd goal)
	{
		_goalSource = CONSTANT;
		_goal = goal;
	}

	void JumpCondition::setConstantGoal(double goal)
	{
		_goalSource = CONSTANT;
		_goal.resize(1,1);
		_goal<<goal;
	}

	void JumpCondition::setROSTopicGoal(const std::string& topic, const std::string& topic_type) {
		// TODO
		HA_THROW_ERROR("JumpCondition.setROSTopicGoal", "Not implemented / tested");
		_goalSource = ROSTOPIC;
		_ros_topic_goal_name = topic;
		_ros_topic_goal_type = topic_type;
	}

	void JumpCondition::setROSTfGoal(const std::string& child, const std::string& parent) {
		// TODO
		HA_THROW_ERROR("JumpCondition.setROSTopicGoal", "Not implemented / tested");
		_goalSource = ROSTOPIC_TF;
		_ros_tf_goal_child = child;
		_ros_tf_goal_parent = parent;
	}


	::Eigen::MatrixXd JumpCondition::getGoal() const	
	{
		switch(_goalSource) {
			case CONSTANT: 
				return _goal;
			
			case CONTROLLER: 
				return _controller->getGoal();

			case ROSTOPIC: {
				::Eigen::MatrixXd pose;
				if(!_system->getROSPose(_ros_topic_goal_name, _ros_topic_goal_type, pose)) {
					HA_ERROR("JumpCondition.getGoal", "Unable to fetch pose from ROS");
				} else {
					return pose;
				}
			}
			case ROSTOPIC_TF: {
				::Eigen::MatrixXd pose;
				if(!_system->getROSTfPose(_ros_tf_goal_child, _ros_tf_goal_parent, pose)) {
					HA_ERROR("JumpCondition.getGoal", "Unable to fetch pose from ROS TF");
				} else {
					return pose;
				}
			}

			default:
				HA_THROW_ERROR("JumpCondition.getGoal", "Not Implemented: unknown goal source");
		}

		return ::Eigen::MatrixXd();
	}
	
	void JumpCondition::setSensor(const Sensor::Ptr sensor) 
	{
		_sensor = sensor;
	}

	Sensor::ConstPtr JumpCondition::getSensor() const 
	{
		return _sensor;
	} 

	void JumpCondition::setJumpCriterion(JumpCriterion jump_criterion, ::Eigen::MatrixXd weights)
	{
//		if(weights.rows() == 0)
//			HA_WARN("JumpCondition::setJumpCriterion", "No value given for weights. Using default weights of 1.");
		_jump_criterion = jump_criterion;
		_norm_weights = weights;
	}
	
	JumpCondition::JumpCriterion JumpCondition::getJumpCriterion() const
	{
		return _jump_criterion;
	}

	::Eigen::MatrixXd JumpCondition::getNormWeights() const
	{
		return _norm_weights;
	}

	void JumpCondition::setEpsilon(double epsilon)
	{
		_epsilon = epsilon;
	}

	double JumpCondition::getEpsilon() const
	{
		return _epsilon;
	}

	void JumpCondition::setSourceModeName(const std::string& sourceModeName)
	{
		_sourceModeName = sourceModeName;
	}

	DescriptionTreeNode::Ptr JumpCondition::serialize(const DescriptionTree::ConstPtr& factory) const 
	{ 
		DescriptionTreeNode::Ptr tree = factory->createNode("JumpCondition");

		switch(_goalSource) {
			case CONSTANT: 
				tree->setAttribute<Eigen::MatrixXd>(std::string("goal"), _goal);
				break;
			
			case CONTROLLER: 
				tree->setAttribute<std::string>(std::string("controller"), this->_controller->getName());
				break;
			
			case ROSTOPIC:
				tree->setAttribute<std::string>(std::string("ros_topic"), _ros_topic_goal_name);
				tree->setAttribute<std::string>(std::string("ros_topic_type"), _ros_topic_goal_type);
				break;

			case ROSTOPIC_TF:
				tree->setAttribute<std::string>(std::string("ros_tf_child"), _ros_tf_goal_child);
				tree->setAttribute<std::string>(std::string("ros_tf_parent"), _ros_tf_goal_parent);
				break;

			default:
				HA_THROW_ERROR("JumpCondition::serialize", "Not Implemented: unknown goal source");
		}

		tree->setAttribute<int>(std::string("jump_criterion"), this->_jump_criterion);
		if(this->_norm_weights.rows() > 0){
			tree->setAttribute< ::Eigen::MatrixXd>(std::string("norm_weights"), this->_norm_weights);
		}

		tree->setAttribute<double>(std::string("epsilon"), this->_epsilon);

		if (!this->_sensor) {
			HA_THROW_ERROR("JumpCondition::serialize", "All JumpConditions need to have a sensor!");
		}


		tree->setAttribute<bool>(std::string("goal_is_relative"), this->_is_goal_relative);

        tree->setAttribute<bool>(std::string("negate"), this->_negate);

		tree->addChildNode(this->_sensor->serialize(factory));

		return tree;
	}

	void JumpCondition::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha) 
	{
		if (tree->getType() != "JumpCondition") {
			HA_THROW_ERROR("JumpCondition.deserialize", "JumpCondition must have type 'JumpCondition', not '" << tree->getType() << "'!");
		}

		//////////////////////////////
		////SENSORS///////////////////
		//////////////////////////////
		DescriptionTreeNode::ConstNodeList sensorList;
		tree->getChildrenNodes("Sensor", sensorList);

		if (sensorList.empty()) {
			HA_THROW_ERROR("JumpCondition.deserialize", "No Sensor found!");
		}

		if (sensorList.size() > 1) {
			HA_THROW_ERROR("JumpCondition.deserialize", "Too many (>1) sensors found!");
		}

		DescriptionTreeNode::ConstPtr first = * (sensorList.begin());
		this->_sensor = HybridAutomaton::createSensor(first, system, ha);

		this->_system = system;

		//////////////////////////////
		////GOALS/////////////////////
		//////////////////////////////
		std::string controllerName;
		bool setController = false;
		if(tree->getAttribute<std::string>(std::string("controller"), controllerName))
		{
			//Now match name to controller
			if(_sourceModeName == "")
				HA_THROW_ERROR("JumpCondition.deserialize", "When deserializing a controller goal, you need to call JumpCondition::setSourceModeName() and pass the name of the mode this Jump Condition's Switch emanates from!!!");

			Controller::ConstPtr ctrl = ha->getControllerByName(_sourceModeName, controllerName);
			if(!ctrl)
				HA_THROW_ERROR("JumpCondition.deserialize", "Controller "<<controllerName<<"not found in source control mode "<<_sourceModeName<<".");


			setController = true;
			this->setControllerGoal(ctrl);
		}
		
		Eigen::MatrixXd goal;
		if(tree->getAttribute<Eigen::MatrixXd>(std::string("goal"), goal))
		{
			if(setController)
			{
				HA_THROW_ERROR("JumpCondition.deserialize", "Use either \"controller\" OR \"goal\" parameter!");
			}
			
			this->setConstantGoal(goal);
		}
		else
		{
			if(!setController)
				HA_WARN("JumpCondition.deserialize", "No \"controller\" OR \"goal\" parameter given  in JumpCondition - using default values");
		}

		//////////////////////////////////
		////ROS GOAL STUFF////////////////
		//////////////////////////////////
		std::string ros_goal;
		if(tree->getAttribute<std::string>(std::string("ros_topic"), ros_goal)) {
			std::string type;
			if(!tree->getAttribute<std::string>(std::string("ros_topic_type"), type)) {
				HA_THROW_ERROR("JumpCondition.deserialize", "If you use ros_topic as goal ros_topic_type must be set!");
			}
			this->setROSTopicGoal(ros_goal, type);
		}

		if(tree->getAttribute<std::string>(std::string("ros_tf_child"), ros_goal)) {
			std::string parent;
			if(!tree->getAttribute<std::string>(std::string("ros_tf_parent"), parent)) {
				HA_THROW_ERROR("JumpCondition.deserialize", "If you use ros_tf as goal ros_tf_parent must be set!");
			}
			this->setROSTfGoal(ros_goal, parent);
		}

		//////////////////////////////
		////PARAMETERS////////////////
		//////////////////////////////
		tree->getAttribute<Eigen::MatrixXd>("norm_weights", _norm_weights);
			//HA_WARN("JumpCondition.deserialize", "No \"norm_weights\" parameter given in JumpCondition - using default values");
		
		if(!tree->getAttribute<double>("epsilon", _epsilon))
			HA_WARN("JumpCondition.deserialize", "No \"epsilon\" parameter given in JumpCondition - using default values");

		std::string jump_criterion_as_string;
		if(tree->getAttribute<std::string>("jump_criterion", jump_criterion_as_string))
		{
			if (!jump_criterion_as_string.empty() && jump_criterion_as_string.find_first_not_of("0123456789") == std::string::npos)
			{
				// it's a digit
				int jump_criterion = atoi(jump_criterion_as_string.c_str());

				if(jump_criterion < 0 || jump_criterion >= NUM_CRITERIA)
					HA_THROW_ERROR("JumpCondition.deserialize", "Unknown jumpCriterion: " << jump_criterion);

				//Cast int to enum
				_jump_criterion = static_cast<JumpCriterion> (jump_criterion);	
			}
			else
			{
				if (jump_criterion_as_string == "NORM_L1")
					_jump_criterion = NORM_L1;
				else if (jump_criterion_as_string == "NORM_L2")
					_jump_criterion = NORM_L2;
				else if (jump_criterion_as_string == "NORM_L_INF")
					_jump_criterion = NORM_L_INF;
				else if (jump_criterion_as_string == "NORM_ROTATION")
					_jump_criterion = NORM_ROTATION;
				else if (jump_criterion_as_string == "NORM_TRANSFORM")
					_jump_criterion = NORM_TRANSFORM;
				else if (jump_criterion_as_string == "THRESH_UPPER_BOUND")
					_jump_criterion = THRESH_UPPER_BOUND;
				else if (jump_criterion_as_string == "THRESH_LOWER_BOUND")
					_jump_criterion = THRESH_LOWER_BOUND;
				else
					HA_THROW_ERROR("JumpCondition.deserialize", "Unknown jumpCriterion: '" << jump_criterion_as_string << "'");
			}
		}
		else
		{
			HA_WARN("JumpCondition.deserialize", "No \"jump_criterion\" parameter given in JumpCondition - using default values");
		}

		if(!tree->getAttribute<bool>("goal_is_relative", _is_goal_relative))
			HA_WARN("JumpCondition.deserialize", "No \"goal_is_relative\" parameter given in JumpCondition - using default values");

        if(!tree->getAttribute<bool>("negate", _negate))
            HA_WARN("JumpCondition.deserialize", "No \"negate\" parameter given in JumpCondition - using default values");

	}

    std::string JumpCondition::toString(bool ) {
        // TODO use latex

        std::stringstream ss;

        bool norm;

        if (this->_jump_criterion == THRESH_UPPER_BOUND
                || this->_jump_criterion == THRESH_LOWER_BOUND
                || this->_jump_criterion == NUM_CRITERIA) {
            norm = false;
        } else {
            norm = true;
        }

        if (norm) ss << "|";

        if (_norm_weights.rows() > 0) {
            if (_norm_weights.rows() > 1 && _norm_weights.cols() > 1) {
                ss << "NormMatrix";
            } else {
               ss << "(";
               // vector
               for (int i = 0; i < _norm_weights.rows(); i++) {
                    for (int j = 0; j < _norm_weights.cols(); j++) {
                        ss << _norm_weights(i,j) << " ";
                    }
               }
               ss << ")";
            }
        }

        ss << "x_" << this->_sensor->getType();

        ss << " - ";

        if (this->_goalSource == CONSTANT) {
            if (_goal.rows() > 0) {
                if (_goal.rows() > 1 && _goal.cols() > 1) {
                    ss << "GoalMatrix";
                } else {
                    ss << "(";
                    // vector
                    for (int i = 0; i < _goal.rows(); i++) {
                        for (int j = 0; j < _goal.cols(); j++) {
                            ss << _goal(i,j) << " ";
                        }
                    }
                    ss << ")";
                }
            }
        } else if (this->_goalSource == CONTROLLER)
            ss << "x_[ctrl:" << _controller->getName() << "]";
        else if (this->_goalSource == ROSTOPIC)
			ss << "x_[ROS:" << _ros_topic_goal_name << ":"<< _ros_topic_goal_type <<"]"; 
        else if (this->_goalSource == ROSTOPIC_TF)
			ss << "x_[ROS_TF:" << _ros_tf_goal_child << ":"<< _ros_tf_goal_parent <<"]"; 

        if (norm) {
            ss << "|";
            if (this->_jump_criterion == NORM_L1) ss << "_1";
            else if (this->_jump_criterion == NORM_L2) ss << "_2";
            else if (this->_jump_criterion == NORM_L_INF) ss << "_oo";
            else if (this->_jump_criterion == NORM_ROTATION) ss << "_R";
            else if (this->_jump_criterion == NORM_TRANSFORM) ss << "_T";

            ss << (!_negate?" < ":" > ") << _epsilon;
        } else {
            if (this->_jump_criterion == THRESH_UPPER_BOUND) ss << " > ";
            else if (this->_jump_criterion == THRESH_LOWER_BOUND) ss << " < ";
            ss << _epsilon;
            // TODO
            //else if (this->_jump_criterion == NUM_CRITERIA)
        }

        return ss.str();
    }

	void JumpCondition::setGoalRelative()
	{
		this->_is_goal_relative = true;
	}

	void JumpCondition::setGoalAbsolute()
	{
		this->_is_goal_relative = false;
	}

	bool JumpCondition::isGoalRelative() const
	{
		return this->_is_goal_relative;
	}

    void JumpCondition::setNegate(bool negate){
        this->_negate=negate;
    }

    bool JumpCondition::isNegate() const
    {
        return this->_negate;
    }
}
