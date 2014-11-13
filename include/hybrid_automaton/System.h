#ifndef HYBRID_AUTOMATON_SYSTEM_H_
#define HYBRID_AUTOMATON_SYSTEM_H_

#include <boost/shared_ptr.hpp>

namespace ha {

	class System;
	typedef boost::shared_ptr<System> SystemPtr;

	class System {

	protected:

	public:
		typedef boost::shared_ptr<System> Ptr;

		System() {
		}

		virtual ~System() {
		}

	};

}

#endif // HYBRID_AUTOMATON_SYSTEM_H_
