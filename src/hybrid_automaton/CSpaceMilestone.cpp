#include "CSpaceMilestone.h"

#include <iostream>
#include <string>

#include "tinyxml.h"
#include "XMLDeserializer.h"

using namespace std;

#define RADIUS 0.4

//TODO: (When creating milestone or setting the MB) Is ok to force the MB to have this Milestone as parent and child?

CSpaceMilestone::CSpaceMilestone() :
Milestone("Default"),
motion_behaviour_(NULL)
{
}

CSpaceMilestone::CSpaceMilestone(const std::string& name) : 
Milestone(name),
motion_behaviour_(NULL)
{
}

CSpaceMilestone::CSpaceMilestone(const std::string& name, const std::vector<double>& configuration, MotionBehaviour * motion_behaviour, const std::vector<double>& region_convergence_radius) :
Milestone(name),
configuration_(configuration),
region_convergence_radius_(region_convergence_radius)
{
	if(motion_behaviour)
	{
		this->motion_behaviour_ = motion_behaviour->clone();
		// Force motion_behaviour to have this milestone as parent and child
		this->motion_behaviour_->setChild(this);
		this->motion_behaviour_->setParent(this);
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}

	Point tmp(configuration[0],configuration[1],0.0);
	tmp.x += RADIUS;
	handle_points_.push_back(tmp);
	tmp.x -= 2 * RADIUS;
	handle_points_.push_back(tmp);
	tmp.x += RADIUS;
	tmp.y += RADIUS;
	handle_points_.push_back(tmp);
	tmp.y -= 2 * RADIUS;
	handle_points_.push_back(tmp);
}

CSpaceMilestone::CSpaceMilestone(const std::string& name, const std::vector<double>& configuration, MotionBehaviour * motion_behaviour,
								 const std::vector<double>& region_convergence_radius, Milestone::Status status, const std::vector<Point>& handle_points):
Milestone(name),
configuration_(configuration),
region_convergence_radius_(region_convergence_radius),
handle_points_(handle_points)
{
	this->status_ = status;
	if(motion_behaviour)
	{
		this->motion_behaviour_ = motion_behaviour->clone();
		// Force motion_behaviour to have this milestone as parent and child
		this->motion_behaviour_->setChild(this);
		this->motion_behaviour_->setParent(this);
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}
}

CSpaceMilestone::CSpaceMilestone(const CSpaceMilestone & cmilestone_cpy) :
Milestone(cmilestone_cpy),
configuration_(cmilestone_cpy.configuration_),
region_convergence_radius_(cmilestone_cpy.region_convergence_radius_),
handle_points_(cmilestone_cpy.handle_points_)
{
	if(cmilestone_cpy.motion_behaviour_)
	{
		this->motion_behaviour_ = cmilestone_cpy.motion_behaviour_->clone();
		this->motion_behaviour_->setChild(this);
		this->motion_behaviour_->setParent(this);
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}
}

CSpaceMilestone::~CSpaceMilestone()
{	
	// Do not delete anything but the MotionBehaviour, it is the only thing created/copied in the constructor
	delete motion_behaviour_;
	motion_behaviour_ = NULL;
}

void CSpaceMilestone::setMotionBehaviour(MotionBehaviour * motion_behaviour)
{
	if(motion_behaviour)
	{
		this->motion_behaviour_ = motion_behaviour->clone();
		this->motion_behaviour_->setChild(this);
		this->motion_behaviour_->setParent(this);
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}
}

std::vector<double> CSpaceMilestone::getConfigurationSTDVector() const
{
	return this->configuration_;	
}

dVector CSpaceMilestone::getConfiguration() const
{
	dVector ret_value;
	for(int i = 0; i<configuration_.size(); i++)
	{
		ret_value.expand(1, configuration_[i]);
	}
	return ret_value;
}

void CSpaceMilestone::update()
{
	if(motion_behaviour_ != NULL){
		cout << "[CSpaceMilestone::update] Updating CS Milestone (not implemented)" << endl;
	}else{
		cout << "[CSpaceMilestone::update] Can't update anything, motion_behaviour_ is NULL!!!" << endl;
	}
}

std::string CSpaceMilestone::toStringXML() const
{
	TiXmlDocument document_xml;
	TiXmlElement * ms_element = this->toElementXML();
	document_xml.LinkEndChild(ms_element);

	// Declare a printer
	TiXmlPrinter printer_xml;
	// attach it to the document you want to convert in to a std::string
	document_xml.Accept(&printer_xml);
	// Create a std::string and copy your document data in to the string
	std::string return_value = printer_xml.CStr();

	//TODO: Memory leaks?????????
	delete ms_element;
	return return_value;
}

TiXmlElement* CSpaceMilestone::toElementXML() const 
{
	TiXmlElement* cspace_ms_xml = new TiXmlElement("Milestone");
	cspace_ms_xml->SetAttribute("type", "CSpace");
	cspace_ms_xml->SetAttribute("status", this->status_);
	cspace_ms_xml->SetAttribute("name", this->name_.c_str());

	std::stringstream value_ss;
	for(int i=0; i<this->configuration_.size()-1; i++)
	{
		value_ss << configuration_.at(i) << " ";
	}
	value_ss << configuration_.at(configuration_.size()-1);
	cspace_ms_xml->SetAttribute("value", value_ss.str().c_str());

	std::stringstream epsilon_ss;
	for(int i=0; i<this->region_convergence_radius_.size()-1; i++)
	{
		epsilon_ss << region_convergence_radius_.at(i) << " ";
	}
	epsilon_ss << region_convergence_radius_.at(region_convergence_radius_.size()-1);
	cspace_ms_xml->SetAttribute("epsilon", epsilon_ss.str().c_str());

	TiXmlElement * handlePoints_txe = new TiXmlElement("HandlePoints");
	cspace_ms_xml->LinkEndChild(handlePoints_txe);
	std::vector<Point>::const_iterator handlePoints_it = handle_points_.begin();
	for (; handlePoints_it
		!= handle_points_.end(); handlePoints_it++) {
			TiXmlElement * handlePoint_txe;
			handlePoint_txe = new TiXmlElement("HandlePoint");
			handlePoints_txe->LinkEndChild(handlePoint_txe);
			handlePoint_txe->SetDoubleAttribute("x", handlePoints_it->x);
			handlePoint_txe->SetDoubleAttribute("y", handlePoints_it->y);
			handlePoint_txe->SetDoubleAttribute("z", handlePoints_it->z);
	}

	if(motion_behaviour_){
		cspace_ms_xml->LinkEndChild(this->motion_behaviour_->toElementXML());
	}
	return cspace_ms_xml;
}

CSpaceMilestone* CSpaceMilestone::clone() const
{
	CSpaceMilestone* new_c_space_milestone = new CSpaceMilestone(*this);
	return new_c_space_milestone;
}

CSpaceMilestone& CSpaceMilestone::operator=(const CSpaceMilestone & cmilestone_assignment)
{
	Milestone::operator=(cmilestone_assignment);
	this->configuration_ = cmilestone_assignment.configuration_;
	if(cmilestone_assignment.motion_behaviour_)
	{
		this->motion_behaviour_ = cmilestone_assignment.motion_behaviour_->clone();
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}


	this->region_convergence_radius_ = cmilestone_assignment.region_convergence_radius_;
	this->handle_points_ = cmilestone_assignment.handle_points_;
	return *this;
}

bool CSpaceMilestone::hasConverged(rxSystem* sys) 
{
	assert (sys != NULL);

	const dVector& q = sys->q();
	for (int i = 0; i < sys->jdof(); i++) {
		double e = ::std::abs(q[i] - configuration_[i]);
		if (e > region_convergence_radius_[i])
		{
#ifdef NOT_IN_RT
			std::cout << "Error in " << i << " is too large = " << e << std::endl;
#endif
			return false;
		}
	}
	return true;
}