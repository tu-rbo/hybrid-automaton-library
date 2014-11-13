#ifndef HYBRID_AUTOMATON_HYBRID_AUTOMATON_H_
#define HYBRID_AUTOMATON_HYBRID_AUTOMATON_H_

#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/ControlMode.h"
#include "hybrid_automaton/ControlSwitch.h"
#include "hybrid_automaton/Serializable.h"

#include <string>
#include <map>
#include <assert.h>

#include <boost/shared_ptr.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/labeled_graph.hpp>
#include <iostream>


////Some typedefs for our classes
//
//
//int main()
//{
////Now create an empty graph
//Graph g;
//
////add vertices
//ControlMode mode1,mode2;
//mode1.val=1; mode2.val=2;
//
//ModeHandle m1 = boost::add_vertex(mode1, g);
//ModeHandle m2 = boost::add_vertex(mode2, g);
//
////add edge
//ControlSwitch s1;
//s1.val = 1;
//boost::add_edge(m1,m2,s1,g);
////iterate over outgoing edges of mode1
//std::pair<OutEdgeIterator, OutEdgeIterator> outEdges = boost::out_edges(m1, g);
//for(; outEdges.first != outEdges.second; ++outEdges.first)
//{
//SwitchHandle s = *outEdges.first;
////Get the switch object
//ControlSwitch testSwitch = g[s];
////Find what the edge is pointing to
//ModeHandle m = boost::target(s, g);
////Receive the mode object - should be mode2
//ControlMode testMode = g[m];

namespace ha {

	class HybridAutomaton;
	typedef boost::shared_ptr<HybridAutomaton> HybridAutomatonPtr;

	/**
	 * @brief Hybrid Automaton implementation and interface
	 */
	class HybridAutomaton : public Serializable {

	public:
		class HybridAutomaton;
		typedef boost::shared_ptr<HybridAutomaton> Ptr;

		typedef ControllerPtr (*ControllerCreator) (void);
		typedef ControlSetPtr (*ControlSetCreator) (void);

		//First a directed graph based on an adjacency list
		typedef ::boost::labeled_graph< boost::adjacency_list< boost::vecS, boost::vecS, boost::directedS, ControlMode::Ptr, ControlSwitch::Ptr >, std::string > Graph;

		//Handle objects for vertices and edges of the graph structure - you can obtain the 
		//Modes and Switches by calling Graph[Handle]
		typedef ::boost::graph_traits< Graph >::vertex_descriptor ModeHandle;
		typedef ::boost::graph_traits< Graph >::edge_descriptor SwitchHandle;

		//an Iterator - there are more variations possible
		typedef ::boost::graph_traits< Graph >::out_edge_iterator OutEdgeIterator;

	protected:
		Graph _graph;
		ControlMode::Ptr _current_control_mode;

		std::string _name;

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
		 * @brief Instantiate a controller with given name 
		 *
		 * In order to work you must register your controller properly
		 */
		static Controller::Ptr createController(const std::string& crtl_name);

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
		static void registerController(const std::string& crtl_name, ControllerCreator cc);
		
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

		void addControlMode(const ControlMode::Ptr& control_mode) {
			boost::add_vertex(control_mode->getName(), control_mode, _graph);
		}

		void addControlSwitch(const std::string& source_mode, const ControlSwitch::Ptr& control_switch, const std::string& target_mode) {
			boost::add_edge_by_label(source_mode, target_mode, control_switch, _graph);
		}

		void addControlSwitchAndMode(const std::string& source_mode, const ControlSwitch::Ptr& control_switch, const ControlMode::Ptr& target_mode) {
			boost::add_vertex(target_mode->getName(), target_mode, _graph);
			boost::add_edge_by_label(source_mode, target_mode->getName(), _graph);
		}

		void step() {
			_current_control_mode->step(); 
		}

		void setName(const std::string& name) {
			_name = name;
		}

		const std::string& getName() const {
			return _name;
		}

		virtual void serialize(DescriptionTreeNode& tree) const;
		virtual void deserialize(const DescriptionTreeNode& tree);

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
