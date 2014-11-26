#ifndef HYBRID_AUTOMATON_CONTROLSET_H_
#define HYBRID_AUTOMATON_CONTROLSET_H_

#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/Serializable.h"
#include "hybrid_automaton/HybridAutomatonRegistration.h"

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include <map>

namespace ha {

	class ControlSet;
	typedef boost::shared_ptr<ControlSet> ControlSetPtr;
	typedef boost::shared_ptr<const ControlSet> ControlSetConstPtr;

	class ControlSet : public Serializable {

	public:
		typedef boost::shared_ptr<ControlSet> Ptr;
		typedef boost::shared_ptr<const ControlSet> ConstPtr;

		ControlSet(); 

		ControlSet(const ControlSet& cs);

		virtual void activate();

		virtual void deactivate();

		virtual ::Eigen::MatrixXd step(const double& t);

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		ControlSetPtr clone() const {
			return ControlSetPtr(_doClone());
		}

		virtual void setType(const std::string& new_type);

		virtual const std::string getType() const;

		void appendController(const Controller::Ptr& controller);

		// what is this method for?
		//virtual const std::vector<Controller::Ptr>& getControllers() const;

		virtual void setName(const std::string& new_name);

		virtual const std::string getName() const;

		virtual Controller::ConstPtr getControllerByName(const std::string& name) const; 

	protected:
		virtual ControlSet* _doClone() const {
			return new ControlSet(*this);
		}

		virtual void _addController(const Controller::Ptr& cntrl);

	protected:
		std::string						_name;
		std::string						_type;
		std::map<std::string, Controller::Ptr>	_controllers;
	};

}

#endif
