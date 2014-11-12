#ifndef HYBRID_AUTOMATON_HYBRID_AUTOMATON_H_
#define HYBRID_AUTOMATON_HYBRID_AUTOMATON_H_

#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/ControlMode.h"
#include "hybrid_automaton/Serializable.h"

#include <string>
#include <map>
#include <assert.h>

#include <boost/shared_ptr.hpp>

// FIXME remove
#include <iostream>

namespace ha {

	class HybridAutomaton;
	typedef boost::shared_ptr<HybridAutomaton> HybridAutomatonPtr;

	class HybridAutomaton : public Serializable {

	public:
		class HybridAutomaton;
		typedef boost::shared_ptr<HybridAutomaton> Ptr;

		typedef ControllerPtr (*ControllerCreator) (void);

	protected:

		// FIXME
		ControlMode::Ptr control_mode;

	private:  

		// see http://stackoverflow.com/questions/8057682/accessing-a-static-map-from-a-static-member-function-segmentation-fault-c
		static std::map<std::string, ControllerCreator> & getControllerTypeMap() {
			static std::map<std::string, ControllerCreator> controller_type_map;
			return controller_type_map; 
		}

	public:

		static void registerController(const std::string& crtl_name, ControllerCreator cc);


		void addControlMode(ControlMode::Ptr cm) {
			// FIXME
			control_mode = cm;
		}

		void step() {
			control_mode->step();
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
