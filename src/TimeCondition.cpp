#include "hybrid_automaton/TimeCondition.h"

namespace ha {
	
	TimeCondition::TimeCondition(double duration)
		: _duration(duration)
	{
	}
	
	void TimeCondition::setDuration(double duration) {
		_duration = duration;
	}

	double TimeCondition::getDuration() const {
		return _duration;
	}
	
	void TimeCondition::activate(const double& t) {
		_start_time = t;
	}

	void TimeCondition::deactivate() {
	}

	bool TimeCondition::isActive() const {
		return (_current_time - _start_time) > _duration;
	}

	void TimeCondition::step(const double& t) {
		_current_time = t;
	}

	DescriptionTreeNode::Ptr TimeCondition::serialize(const DescriptionTree::ConstPtr& factory) const {
		// TODO
		return DescriptionTreeNode::Ptr();
	}
	
	void TimeCondition::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system) {
		// TODO
	}

}