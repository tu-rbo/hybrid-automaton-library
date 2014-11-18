#ifndef HYBRID_AUTOMATON_HYBRID_AUTOMATON_H_
#define HYBRID_AUTOMATON_HYBRID_AUTOMATON_H_

#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/ControlMode.h"
#include "hybrid_automaton/ControlSwitch.h"
#include "hybrid_automaton/Serializable.h"

#include "hybrid_automaton/System.h"
#include "hybrid_automaton/DescriptionTreeNode.h"

#include <string>
#include <map>
#include <assert.h>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/labeled_graph.hpp>

#include <Eigen/Dense>

namespace ha {

	class HybridAutomaton;
	typedef boost::shared_ptr<HybridAutomaton> HybridAutomatonPtr;
	typedef boost::shared_ptr<const HybridAutomaton> HybridAutomatonConstPtr;

	/**
	 * @brief Hybrid Automaton implementation and interface
	 */
	class HybridAutomaton : public Serializable {

	public:
		class HybridAutomaton;
		typedef boost::shared_ptr<HybridAutomaton> Ptr;
		typedef boost::shared_ptr<const HybridAutomaton> ConstPtr;

		typedef ::ha::Controller::Ptr (*ControllerCreator) (const ::ha::DescriptionTreeNode::ConstPtr, const ::ha::System::ConstPtr, const HybridAutomaton*);
		typedef ::ha::ControlSet::Ptr (*ControlSetCreator) (const ::ha::DescriptionTreeNode::ConstPtr, const ::ha::System::ConstPtr, const HybridAutomaton*);

		// a directed labeled graph based on an adjacency list
		typedef ::boost::labeled_graph< boost::adjacency_list< boost::vecS, boost::vecS, boost::directedS, ControlMode::Ptr, ControlSwitch::Ptr >, std::string > Graph;

		//Handle objects for vertices and edges of the graph structure - you can obtain the 
		//Modes and Switches by calling Graph[Handle]
		typedef ::boost::graph_traits< Graph >::vertex_descriptor ModeHandle;
		typedef ::boost::graph_traits< Graph >::edge_descriptor SwitchHandle;
		typedef ::boost::graph_traits< Graph > GraphTraits;

		//Iterators
		typedef ::boost::graph_traits< Graph >::vertex_iterator ModeIterator;
		typedef ::boost::graph_traits< Graph >::edge_iterator SwitchIterator;
		typedef ::boost::graph_traits< Graph >::out_edge_iterator OutEdgeIterator;

	protected:
		Graph _graph;
		ControlMode::Ptr _current_control_mode;

		std::string _name;

		bool _active;

		// helper function -- not virtual!
		void _activateCurrentControlMode(const double& t);

	private:  

		// see http://stackoverflow.com/questions/8057682/accessing-a-static-map-from-a-static-member-function-segmentation-fault-c
		static std::map<std::string, ControllerCreator> & getControllerTypeMap() {
			static std::map<std::string, ControllerCreator> controller_type_map;
			return controller_type_map; 
		}

		static std::map<std::string, ControlSetCreator> & getControlSetTypeMap() {
			static std::map<std::string, ControlSetCreator> controlset_type_map;
			return controlset_type_map; 
		}

	public:
		/** 
		 * @brief Instantiate a controller of given type 
		 *
		 * In order to work you must register your controller properly
		 */
		static Controller::Ptr createController(const DescriptionTreeNode::ConstPtr& node, const System::ConstPtr& system, const HybridAutomaton* ha);

		/** 
		 * @brief Instantiate a control set of given type 
		 *
		 * In order to work you must register your control set properly
		 */
		static ControlSet::Ptr createControlSet(const DescriptionTreeNode::ConstPtr& node, const System::ConstPtr& system, const HybridAutomaton* ha);

		/**
		 * @brief Register a controller with the hybrid automaton
		 *
		 * Do not call this function yourself but rather use the macros:
		 *  HA_RLAB_CONTROLLER_REGISTER_HEADER()
		 *		-> to your header file
		 *  HA_RLAB_CONTROLLER_REGISTER_CPP("YourController", YourController)
		 *      -> to your cpp file
		 *
		 * @see hybrid_automaton/hybrid_automaton_registration.h
		 */
		static void registerController(const std::string& crtl_type, ControllerCreator cc);

		/**
		 * @brief Check whether a controller with given type is registered
		 */
		static bool isControllerRegistered(const std::string& crtl_type);

		/**
		 * @brief Unregister a controller with the hybrid automaton
		 *
		 * Usually you do not need that except for testing
		 */
		static void unregisterController(const std::string& crtl_type);

		/**
		 * @brief Register a control set with the hybrid automaton
		 *
		 * Do not call this function yourself but rather use the macros:
		 *  HA_RLAB_CONTROLSET_REGISTER_HEADER()
		 *		-> to your header file
		 *  HA_RLAB_CONTROLSET_REGISTER_CPP("YourControlSet", YourControlSet)
		 *      -> to your cpp file
	     *
		 * @see hybrid_automaton/hybrid_automaton_registration.h
		 */
		static void registerControlSet(const std::string& crtl_name, ControlSetCreator cc);

		/**
		 * @brief Check whether a control set with given type is registered
		 */
		static bool isControlSetRegistered(const std::string& crtl_type);

		/**
		 * @brief Unregister a control set with the hybrid automaton
		 *
		 * Usually you do not need that except for testing
		 */
		static void unregisterControlSet(const std::string& crtl_name);

		void addControlMode(const ControlMode::Ptr& control_mode);
		void addControlSwitch(const std::string& source_mode, const ControlSwitch::Ptr& control_switch, const std::string& target_mode);
		void addControlSwitchAndMode(const std::string& source_mode, const ControlSwitch::Ptr& control_switch, const ControlMode::Ptr& target_mode);

		::Eigen::MatrixXd step(const double& t);

		void setName(const std::string& name);
		const std::string getName() const;

		virtual bool isActive() const {
			return _active;
		}

		void activate(const double& t);
		void deactivate();

		void setCurrentControlMode(const std::string& control_mode);
		ControlMode::Ptr getCurrentControlMode() const;

		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system) {
			this->deserialize(tree, system, this);
		}
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

		virtual bool existsControlMode(const std::string& control_mode);

		virtual Controller::Ptr getControllerByName(const std::string& control_mode_name, const std::string& controller_name);

		HybridAutomatonPtr clone() const {
			return HybridAutomatonPtr(_doClone());
		}

	protected:
		virtual HybridAutomaton* _doClone() const {
			return new HybridAutomaton(*this);
		}
	};

}

#endif
