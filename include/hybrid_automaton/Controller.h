/*!
* <File comment goes here!!>
* 
* Copyright (c) 200x by <your name/ organization here>
*/
#ifndef HYBRID_AUTOMATON_CONTROLLER_H_
#define HYBRID_AUTOMATON_CONTROLLER_H_

#include "hybrid_automaton/Serializable.h"
#include "hybrid_automaton/System.h"
#include "hybrid_automaton/hybrid_automaton_registration.h"

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
		virtual void serialize(const DescriptionTreeNode::Ptr& tree) const;

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
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree);

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

		virtual void setDimensionality(const int& new_dimensionality);

		virtual Eigen::VectorXd getGoal() const;

		virtual void setGoal(const Eigen::VectorXd& new_goal);

		virtual Eigen::VectorXd getKp() const;

		virtual void setKp(const Eigen::VectorXd& new_kp);

		virtual Eigen::VectorXd getKv() const;

		virtual void setKv(const Eigen::VectorXd& new_kv);

		virtual std::string getName() const;

		virtual void setName(const std::string& new_name);

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

		int					_dimensionality;
		Eigen::VectorXd		_goal;
		Eigen::VectorXd		_kp;
		Eigen::VectorXd		_kv;
		std::string			_name;
	};

}

#endif
