/*!
* <File comment goes here!!>
* 
* Copyright (c) 200x by <your name/ organization here>
*/
#ifndef HYBRID_AUTOMATON_CONTROLLER_H_
#define HYBRID_AUTOMATON_CONTROLLER_H_

#include "hybrid_automaton/Serializable.h"
#include "hybrid_automaton/hybrid_automaton_registration.h"

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ha {

	class Controller;
	typedef boost::shared_ptr<Controller> ControllerPtr;

	class Controller : public Serializable {

	public:
		typedef boost::shared_ptr<Controller> Ptr;

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
		virtual void step() {
			throw "not implemented";
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
		virtual void serialize(DescriptionTreeNode::Ptr& tree) const;

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
		virtual void deserialize(const DescriptionTreeNode::Ptr tree);

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
		int priority;
		double max_velocity;

		Eigen::VectorXd _goal;
		Eigen::VectorXd _kp;
		Eigen::VectorXd _kv;
	};

}

#endif
