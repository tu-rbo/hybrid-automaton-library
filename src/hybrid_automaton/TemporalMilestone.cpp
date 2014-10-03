#include "TemporalMilestone.h"

#include <iostream>
#include <string>

#include "tinyxml.h"
#include "XMLDeserializer.h"

TemporalMilestone::TemporalMilestone() :
OpSpaceMilestone("Default"),
t_(0.0),
duration_(0.0),
desiredT_(0.0),
firstUpdateSinceActivated_(true)
{
}

TemporalMilestone::TemporalMilestone(const std::string& osm_name,  Displacement position, Rotation orientation, const std::string& frame_id,
		const rTime& duration,
		PosiOriSelector posi_ori_selection, MotionBehaviour * motion_behaviour,
		std::vector<double>& region_convergence_radius, Milestone::Status status, 
		std::vector<Point> handle_points, rxSystem* sys) : 
OpSpaceMilestone(osm_name, position, orientation, frame_id, posi_ori_selection, motion_behaviour, region_convergence_radius, status, handle_points),
t_(0.0),
duration_(duration),
desiredT_(0.0),
firstUpdateSinceActivated_(true)
{
}

TemporalMilestone::TemporalMilestone(const TemporalMilestone & cmilestone_cpy) :
OpSpaceMilestone(cmilestone_cpy),
t_(cmilestone_cpy.t_),
duration_(cmilestone_cpy.duration_),
desiredT_(cmilestone_cpy.desiredT_),
firstUpdateSinceActivated_(cmilestone_cpy.firstUpdateSinceActivated_)
{
}

TemporalMilestone* TemporalMilestone::clone() const
{
	TemporalMilestone* new_t_milestone = new TemporalMilestone(*this);
	return new_t_milestone;
}

TemporalMilestone& TemporalMilestone::operator=(const TemporalMilestone & cmilestone_assignment)
{
	OpSpaceMilestone::operator=(cmilestone_assignment);
	this->t_ = cmilestone_assignment.t_;
	this->t_ = cmilestone_assignment.duration_;
	this->t_ = cmilestone_assignment.desiredT_;
	this->t_ = cmilestone_assignment.firstUpdateSinceActivated_;

	return *this;
}

TemporalMilestone::~TemporalMilestone()
{	
}

rTime TemporalMilestone::getDuration() const
{
	return this->duration_;
}

rTime TemporalMilestone::getDesiredTime() const
{
	return this->desiredT_;
}

void TemporalMilestone::activate(rxSystem* sys)
{
	OpSpaceMilestone::activate(sys);

	this->activated_ = true;
	this->firstUpdateSinceActivated_ = true;
}

void TemporalMilestone::update(const rTime& t)
{
	this->t_ = t;

	if (this->firstUpdateSinceActivated_)
	{
		this->desiredT_ = t + duration_;
		this->firstUpdateSinceActivated_ = false;
	}
}

void TemporalMilestone::deactivate()
{
	this->activated_ = false;
}

bool TemporalMilestone::hasConverged(rxSystem* sys) 
{
	return (t_ > desiredT_);
}
