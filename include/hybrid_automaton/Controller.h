/*!
* <File comment goes here!!>
* 
* Copyright (c) 200x by <your name/ organization here>
*/
#ifndef HYBRID_AUTOMATON_CONTROLLER_H_
#define HYBRID_AUTOMATON_CONTROLLER_H_

#include "hybrid_automaton/Serializable.h"
#include "hybrid_automaton/System.h"
#include "hybrid_automaton/HybridAutomatonRegistration.h"
#include "hybrid_automaton/error_handling.h"

#include "hybrid_automaton/HybridAutomatonOStringStream.h"

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ha {

	class Controller;
	typedef boost::shared_ptr<Controller> ControllerPtr;
	typedef boost::shared_ptr<const Controller> ControllerConstPtr;

	class Controller : public Serializable {

	public:
		typedef boost::shared_ptr<Controller> Ptr;
		typedef boost::shared_ptr<const Controller> ConstPtr;

		/*!
		* Constructor
		*/
		Controller();

		/*!
		* Destructor
		*/
		virtual ~Controller();

		/*!
		* Copy constructor
		*/
		Controller(const Controller& controller);

		/*!
		* @brief Activate the controller for execution
	    */
		virtual void initialize() {
			HA_THROW_ERROR("Controller.initialize", "not implemented");
		}

		/*!
		* @brief Deactivate the controller for execution
	    */
		virtual void terminate() {
			HA_THROW_ERROR("Controller.terminate", "not implemented");
		}

		/*!
		* \brief
		* Step function
		* 
		* 
		* \throws <exception class>
		* Description of criteria for throwing this exception.
		* 
		* Write detailed description for deserialize here.
		* 
		* \remarks
		* Write remarks for deserialize here.
		* 
		* \see
		* Separate items with the '|' character.
		*/
		virtual ::Eigen::MatrixXd step(const double& t) {
			HA_THROW_ERROR("Controller.step", "not implemented");
		}

		/*!
		* @brief Transform a goal in relative coordinates into absolute coordinates.
	    */
		virtual ::Eigen::MatrixXd relativeGoalToAbsolute(const Eigen::MatrixXd& goalRel) const{
			HA_THROW_ERROR("Controller.relativeGoalToAbsolute", "not implemented - You can not use a relative goal with this type of controller!: "<<this->_type);
		}

		/*!
		* \brief
		* Write brief comment for deserialize here.
		* 
		* \param tree
		* Description of parameter tree.
		* 
		* \throws <exception class>
		* Description of criteria for throwing this exception.
		* 
		* Write detailed description for deserialize here.
		* 
		* \remarks
		* Write remarks for deserialize here.
		* 
		* \see
		* Separate items with the '|' character.
		*/
		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		/*!
		* \brief
		* Write brief comment for deserialize here.
		* 
		* \param tree
		* Description of parameter tree.
		* 
		* \throws <exception class>
		* Description of criteria for throwing this exception.
		* 
		* Write detailed description for deserialize here.
		* 
		* \remarks
		* Write remarks for deserialize here.
		* 
		* \see
		* Separate items with the '|' character.
		*/
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		/*!
		* \brief
		* Clone function
		* 
		* 
		* \throws <exception class>
		* Description of criteria for throwing this exception.
		* 
		* Write detailed description for deserialize here.
		* 
		* \remarks
		* Write remarks for deserialize here.
		* 
		* \see
		* Separate items with the '|' character.
		*/
		ControllerPtr clone() const {
			return ControllerPtr(_doClone());
		}

		/**
		 * @deprecated
		 */
		virtual int getDimensionality() const;

		virtual Eigen::MatrixXd getGoal() const;
		virtual void setGoal(const Eigen::MatrixXd& new_goal);
	
		virtual bool getGoalIsRelative() const;
		virtual void setGoalIsRelative(bool is_goal_relative);

		virtual Eigen::MatrixXd getKp() const;
		virtual void setKp(const Eigen::MatrixXd& new_kp);

		virtual Eigen::MatrixXd getKv() const;
		virtual void setKv(const Eigen::MatrixXd& new_kv);

		/**
		 * @brief Completion times to reach goal and intermediate points
		 *
		 * Optional attribute, e.g. makes sense for interpolated
		 * controllers/
		 */
		virtual void setCompletionTimes(const Eigen::MatrixXd& new_times);
		virtual void setCompletionTime(const double& t); //for convenience - creates 1x1 matrix	
		virtual Eigen::MatrixXd getCompletionTimes() const;

		/**
		 * @brief Priority of this controller in a ControlSet
		 *
		 * Optional attribute.
		 */
		virtual double getPriority() const;
		virtual void setPriority(double priority);
		
		virtual std::string getName() const;
		virtual void setName(const std::string& new_name);

		virtual const std::string getType() const;
		virtual void setType(const std::string& new_type);

		/**
		 * @brief Pass any additional argument for your controller
		 */
		template <typename T> void setArgument(const std::string& field_name, const T& field_value)
		{
			ha_ostringstream ha_oss;
			ha_oss << field_value;
			this->setArgumentString(field_name, ha_oss.str());
		}

		template <typename T> bool getArgument(const std::string& field_name, T& return_value) const
		{
			std::string val;
			bool ret = this->getArgumentString(field_name, val);
			if (ret)
			{
				std::istringstream ss(val);
				ss >> return_value;
				return true;
			}
			else
			{
				return false;
			}
		}

	protected:

		virtual void setArgumentString(const std::string& name, const std::string& value)
		{
			this->_additional_arguments[name] = value;
		}

		virtual bool getArgumentString(const std::string& name, std::string& value) const 
		{
			std::map<std::string, std::string>::const_iterator it;
			it = this->_additional_arguments.find(name);
			if (it == this->_additional_arguments.end())
				return false;
			value = it->second;
			return true;
		}

		/*!
		* \brief
		* 
		* \throws <exception class>
		* Description of criteria for throwing this exception.
		* 
		* Write detailed description for deserialize here.
		* 
		* \remarks
		* Write remarks for deserialize here.
		* 
		* \see
		* Separate items with the '|' character.
		*/
		virtual Controller* _doClone() const {
			return new Controller(*this);
		}

		
	protected:

		Eigen::MatrixXd		_goal;
		bool				_goal_is_relative;
		Eigen::MatrixXd		_kp;
		Eigen::MatrixXd		_kv;
		Eigen::MatrixXd		_completion_times;
		double				_priority;
		std::string			_name;
		std::string			_type;

		std::map<std::string, std::string> _additional_arguments;
	};

}

#endif
