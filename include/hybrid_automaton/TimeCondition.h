#ifndef HYBRID_AUTOMATON_TIME_CONDITION_H
#define HYBRID_AUTOMATON_TIME_CONDITION_H

#include "hybrid_automaton/JumpCondition.h"

#include <boost/shared_ptr.hpp>

namespace ha {

    class TimeCondition;
    typedef boost::shared_ptr<TimeCondition> TimeConditionPtr;

	class TimeCondition : public JumpCondition
    {
    public:

    typedef boost::shared_ptr<TimeCondition> Ptr;

    TimeCondition(double duration);
    virtual ~TimeCondition() {}

    TimeConditionPtr clone() const
    {
      return (TimeConditionPtr(_doClone()));
    }

	virtual void activate(const double& t);
	virtual void deactivate();

	virtual void step(const double& t);
    virtual bool isActive() const;

	virtual void serialize(const DescriptionTreeNode::ConstPtr& factory) const;
	virtual void deserialize(const DescriptionTreeNode::ConstPtr& tree);

	void setDuration(double duration);
	double getDuration() const;

    protected:
		double _duration;
		double _current_time;
		double _start_time;

    virtual TimeCondition* _doClone() const
    {
      return (new TimeCondition(*this));
    }

};

}

#endif
