#ifndef HYBRID_AUTOMATON_HYBRID_AUTOMATON_H_
#define HYBRID_AUTOMATON_HYBRID_AUTOMATON_H_

#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/ControlMode.h"
#include "hybrid_automaton/Serializable.h"

#include <string>
#include <map>
#include <assert.h>

#include <boost/shared_ptr.hpp>

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

	protected:

		// FIXME
		std::vector<ControlMode::Ptr> control_modes;

		std::string name;

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


		void addControlMode(ControlMode::Ptr cm) {
			// FIXME
			control_modes.push_back(cm);
		}

		void step() {
			// TODO
			control_modes[0]->step();
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
