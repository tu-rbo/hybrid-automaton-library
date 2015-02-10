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

		virtual void initialize();

		virtual void terminate();

		virtual ::Eigen::MatrixXd step(const double& t);

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		ControlSetPtr clone() const {
			return ControlSetPtr(_doClone());
		}

		virtual void setType(const std::string& new_type);

		virtual const std::string getType() const;

		/**
		 * @brief Pass any additional argument for your controlset
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

		void appendController(const Controller::Ptr& controller);

		// what is this method for?
		//virtual const std::vector<Controller::Ptr>& getControllers() const;

		virtual void setName(const std::string& new_name);

		virtual const std::string getName() const;

		virtual Controller::ConstPtr getControllerByName(const std::string& name) const; 

        /**
         * @brief Get the list of stored controllers (const reference)
         *
         * This method is used by HybridAutomaton to visualize the controllers.
         *
         */
        virtual const std::map<std::string, Controller::Ptr>& getControllers()  {
            return _controllers;
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

		virtual ControlSet* _doClone() const {
			return new ControlSet(*this);
		}

		virtual void _addController(const Controller::Ptr& cntrl);



	protected:
		std::string						_name;
		std::string						_type;
		std::map<std::string, Controller::Ptr>	_controllers;
	
		std::map<std::string, std::string> _additional_arguments;
	};

}

#endif
