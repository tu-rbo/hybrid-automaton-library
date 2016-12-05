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
     * @brief The Hybrid Automaton implementation and interface
     *
     * A HybridAutomaton is a directed graph with ControlMode as nodes and ControlSwitch as edges.
     * The HybridAutomaton class encapsulated all graph functions, node, and edge access using ::boost::adjacency_list
     * The step() function executes one control cycle of the HybridAutomaton and should be called in your control loop.
	 */
	class HybridAutomaton : public Serializable {

	public:
		typedef boost::shared_ptr<HybridAutomaton> Ptr;
		typedef boost::shared_ptr<const HybridAutomaton> ConstPtr;

		typedef ::ha::Controller::Ptr (*ControllerCreator) (const ::ha::DescriptionTreeNode::ConstPtr, const ::ha::System::ConstPtr, const HybridAutomaton*);
		typedef ::ha::ControlSet::Ptr (*ControlSetCreator) (const ::ha::DescriptionTreeNode::ConstPtr, const ::ha::System::ConstPtr, const HybridAutomaton*);
		typedef ::ha::Sensor::Ptr (*SensorCreator) (const ::ha::DescriptionTreeNode::ConstPtr, const ::ha::System::ConstPtr, const HybridAutomaton*);

        /**
         * @brief Helper: a directed graph based on an adjacency list
         */
        typedef ::boost::adjacency_list< boost::vecS, boost::vecS, boost::directedS, ControlMode::Ptr, ControlSwitch::Ptr > AdjacencyListGraph;

        /**
         * @brief The Graph structure of the hybrid automaton: a directed, labeled adjacency list.
         */
        typedef ::boost::labeled_graph< AdjacencyListGraph, std::string > Graph;

        /**
         * @brief Handle for vertices in the graph - you can obtain the ControlModes by calling Graph[Handle]
         */
		typedef ::boost::graph_traits< Graph >::vertex_descriptor ModeHandle;

        /**
         * @brief Handle for edges in the graph - you can obtain the ControlSwitches by calling Graph[Handle]
         */
		typedef ::boost::graph_traits< Graph >::edge_descriptor SwitchHandle;

        /**
         * @brief GraphTraits object needed for the iterators
         */
        typedef ::boost::graph_traits< Graph > GraphTraits;

        /**
         * @brief Iterator for all ControlModes
         */
		typedef ::boost::graph_traits< Graph >::vertex_iterator ModeIterator;

        /**
         * @brief Iterator for all ControlSwitches
         */
		typedef ::boost::graph_traits< Graph >::edge_iterator SwitchIterator;

        /**
         * @brief Iterator for all Edges emanating from a node
         */
		typedef ::boost::graph_traits< Graph >::out_edge_iterator OutEdgeIterator;

	protected:
        /**
         * @brief The underlying graph structure
         */
        Graph _graph;

        /**
         * @brief The currently active control mode
         */
		ControlMode::Ptr _current_control_mode;

		/**
         * @brief The last active control switch
         */
		ControlSwitch::Ptr _last_active_control_switch;

        /**
         * @brief maps switch names to edges in the graph
         */
		std::map<std::string, SwitchHandle> _switchMap;

        /**
         * @brief the name of this HybridAutomaton
         */
		std::string _name;

        /**
         * @brief true, if this HybridAutomaton is currently being executed
         */
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
		
		static std::map<std::string, SensorCreator> & getSensorTypeMap() {
			static std::map<std::string, SensorCreator> sensor_type_map;
			return sensor_type_map; 
		}

        bool _deserialize_default_entities; /**< if true deserialization will not try to find the actual controller etc., but deserialize them raw */

	public:
		HybridAutomaton();
		virtual ~HybridAutomaton();

		/** 
         * @brief Instantiate a Controller of given type
		 *
         * In order to work you must register your Controller properly
		 */
		static Controller::Ptr createController(const DescriptionTreeNode::ConstPtr& node, const System::ConstPtr& system, const HybridAutomaton* ha);

		/** 
         * @brief Instantiate a ControlSet of given type
		 *
		 * In order to work you must register your control set properly
		 */
		static ControlSet::Ptr createControlSet(const DescriptionTreeNode::ConstPtr& node, const System::ConstPtr& system, const HybridAutomaton* ha);

        /**
         * @brief Instantiate a Sensor of given type
         *
         * In order to work you must register your sensor properly
         */
		static Sensor::Ptr createSensor(const DescriptionTreeNode::ConstPtr& node, const System::ConstPtr& system, const HybridAutomaton* ha);

		/**
         * @brief Register a Controller with the hybrid automaton
         *
         * Do not call this function yourself but rather use the macros:
         *  HA_CONTROLLER_INSTANCE(...) {...}
         *		-> to your header file
         *  HA_CONTROLLER_REGISTER("YourController", YourController)
         *      -> to your cpp file
         *
         * @see hybrid_automaton/hybrid_automaton_registration.h
		 */
		static void registerController(const std::string& crtl_type, ControllerCreator cc);

		/**
         * @brief Check whether a Controller with given type is registered
		 */
		static bool isControllerRegistered(const std::string& crtl_type);

		/**
         * @brief Unregister a Controller with the hybrid automaton
		 *
		 * Usually you do not need that except for testing
		 */
		static void unregisterController(const std::string& crtl_type);

		/**
         * @brief Register a ControlSet with the hybrid automaton
		 *
		 * Do not call this function yourself but rather use the macros:
         *  HA_CONTROLSET_INSTANCE(...) {...}
		 *		-> to your header file
         *  HA_CONTROLSET_REGISTER("YourControlSet", YourControlSet)
		 *      -> to your cpp file
	     *
		 * @see hybrid_automaton/hybrid_automaton_registration.h
		 */
		static void registerControlSet(const std::string& crtl_name, ControlSetCreator cc);

		/**
         * @brief Check whether a ControlSet with given type is registered
		 */
		static bool isControlSetRegistered(const std::string& crtl_type);

		/**
         * @brief Unregister a ControlSet with the HybridAutomaton
		 * Usually you do not need that except for testing
		 */
		static void unregisterControlSet(const std::string& crtl_name);

        /**
         * @brief Register a Sensor with the HybridAutomaton
         *
         * Do not call this function yourself but rather use the macros:
         *  HA_SENSOR_INSTANCE(...) {...}
         *		-> to your header file
         *  HA_SENSOR_REGISTER("YourSensor", YourSensor)
         *      -> to your cpp file
         *
         * @see hybrid_automaton/hybrid_automaton_registration.h
         */
		static void registerSensor(const std::string& sensor_type, SensorCreator sc);

        /**
         * @brief Check whether a Sensor with given type is registered
         */
		static bool isSensorRegistered(const std::string& sensor_type);

        /**
         * @brief Unregister a Sensor with the HybridAutomaton
         *
         * Usually you do not need that except for testing
         */
		static void unregisterSensor(const std::string& sensor_type);

        /**
         * @brief Add a ControlMode to this HybridAutomaton
         */
		void addControlMode(const ControlMode::Ptr& control_mode);

        /**
         * @brief Add a ControlSwitch pointing from \a source_mode to \a target_mode to this HybridAutomaton
         */
		void addControlSwitch(const std::string& source_mode, const ControlSwitch::Ptr& control_switch, const std::string& target_mode);

        /**
         * @brief Add a ControlMode \target_mode and ControlSwitch pointing from \a source_mode to \a target_mode to this HybridAutomaton
         */
		void addControlSwitchAndMode(const std::string& source_mode, const ControlSwitch::Ptr& control_switch, const ControlMode::Ptr& target_mode);

        /**
         * @brief Compute the output of the currently active ControlSet and switch to the next ControlMode if the Jumpconditions are active
         *
         * This is the main function of the HybridAutomaton. It should be called once per control loop. It outputs the torque
         * and updates the actice mode of the HybridAutomaton
         *
         * @param t the current time of your system
         * @return the control output to your hardware (usually a dim x 1 torque vector)
         */
		::Eigen::MatrixXd step(const double& t);

		void setName(const std::string& name);
		const std::string getName() const;

		virtual bool isActive() const {
			return _active;
		}

		void initialize(const double& t);
		void terminate();

		void setCurrentControlMode(const std::string& control_mode);
		ControlMode::Ptr getCurrentControlMode() const;
		ControlSwitch::Ptr getLastActiveControlSwitch() const;

        /**
         * @brief write this HybridAutomaton into a DescriptionTreeNode.
         *
         * @param factory a DescriptionTree object which implements methods to create DescriptionTreeNodes
         * @returns a DescriptionTreeNode containing all the information of this HybridAutomaton
         */
		virtual DescriptionTreeNode::Ptr serialize(const DescriptionTree::ConstPtr& factory) const;

        /**
         * @brief write the contents of DescriptionTreeNode into this HybridAutomaton.
         *
         * @param tree the DescriptionTreeNode containing the parameters
         * @param system a System pointer to the currently active robot
         */
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system) {
			this->deserialize(tree, system, this);
		}

        /**
         * @brief write the contents of DescriptionTreeNode into this HybridAutomaton.
         */
		virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha);

        /**
         * @brief returns if ControlMode \a control_mode is part of this HybridAutomaton
         */
		virtual bool existsControlMode(const std::string& control_mode) const;

        /**
         * @brief returns if ControlSwitch \a control_switch is part of this HybridAutomaton
         */
		virtual	bool existsControlSwitch(const std::string& control_switch) const; 

        /**
         * @brief returns Controller with name \a controller_name
         *
         * @throws error if Controller does not exist
         */
		virtual Controller::ConstPtr getControllerByName(const std::string& control_mode_name, const std::string& controller_name) const;

        /**
         * @brief returns ControlMode with name \a control_mode_name
         *
         * @throws error if ControlMode does not exist
         */
		virtual ControlMode::ConstPtr getControlModeByName(const std::string& control_mode_name) const;

        /**
         * @brief returns ControlSwitch with name \a control_switch_name
         *
         * @throws error if ControlSwitch does not exist
         */
		virtual ControlSwitch::ConstPtr getControlSwitchByName(const std::string& control_switch_name) const;
	
        /**
         * @brief returns the ControlMode the ControlSwitch with name \a control_switch is emanating from
         *
         * @throws error if ControlSwitch does not exist
         */
		virtual ControlMode::ConstPtr getSourceControlMode(const std::string& controlSwitch) const;

        /**
         * @brief returns the ControlMode the ControlSwitch with name \a control_switch is pointing to
         *
         * @throws error if ControlSwitch does not exist
         */
		virtual ControlMode::ConstPtr getTargetControlMode(const std::string& controlSwitch) const;

        /**
         * @brief Create a .dot visualization of this HybridAutomaton and write it into \a filename
         */
        virtual void visualizeGraph(const std::string& filename);


        /**
         * @brief If set to true the deserialization process will translate controllers and control sets to
         *  Controller and ControlSet classes, and not the specific classes.
         *
         * This flag is useful if you want to deserialize a hybrid automaton on the client system,
         * i.e. the system that does not contain the concrete implementations, but which only
         * assembles the automaton.
         *
         * @see visualizeGraph()
         *
         * @param b
         */
        virtual void setDeserializeDefaultEntities(bool b) {
            _deserialize_default_entities = b;
        }
        virtual bool getDeserializeDefaultEntities() const {
            return _deserialize_default_entities;
        }

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
