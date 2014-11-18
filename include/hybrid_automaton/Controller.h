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
		virtual void activate() {
			HA_THROW_ERROR("Controller.activate", "not implemented");
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

		virtual int getDimensionality() const;

		virtual Eigen::MatrixXd getGoal() const;

		virtual void setGoal(const Eigen::MatrixXd& new_goal);

		virtual Eigen::MatrixXd getKp() const;

		virtual void setKp(const Eigen::MatrixXd& new_kp);

		virtual Eigen::MatrixXd getKv() const;

		virtual void setKv(const Eigen::MatrixXd& new_kv);

		virtual std::string getName() const;

		virtual void setName(const std::string& new_name);

		virtual const std::string getType() const;

		virtual void setType(const std::string& new_type);

	protected:

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
		Eigen::MatrixXd		_kp;
		Eigen::MatrixXd		_kv;
		std::string			_name;
		std::string			_type;
	};

}

#endif
