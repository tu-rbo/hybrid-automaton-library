#ifndef HYBRID_AUTOMATON_CONTROLSET_H_
#define HYBRID_AUTOMATON_CONTROLSET_H_

#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/Serializable.h"
#include "hybrid_automaton/HybridAutomatonRegistration.h"

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include <vector>

namespace ha {

	class ControlSet;
	typedef boost::shared_ptr<ControlSet> ControlSetPtr;
	typedef boost::shared_ptr<const ControlSet> ControlSetConstPtr;

	class ControlSet : public Serializable {

	public:
		typedef boost::shared_ptr<ControlSet> Ptr;
		typedef boost::shared_ptr<const ControlSet> ConstPtr;

		ControlSet() {
		}

		virtual void activate() {
			throw "not implemented";
		}

		virtual void deactivate() {
			throw "not implemented";
		}

		virtual ::Eigen::MatrixXd step(const double& t) {
			throw "not implemented"; 
		}

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree);

		ControlSetPtr clone() const {
			return ControlSetPtr(_doClone());
		}

		virtual void setType(const std::string& new_type);

		virtual const std::string& getType() const;

		void appendController(const Controller::Ptr& controller);

		virtual const std::vector<Controller::Ptr>& getControllers() const;

	protected:
		virtual ControlSet* _doClone() const {
			return new ControlSet(*this);
		}

		virtual void _addController(const Controller::Ptr) {
			// empty - can be overloaded			
		}

	protected:
		std::string						_name;
		std::string						_type;
		std::vector<Controller::Ptr>	_controllers;
	};

}

#endif
