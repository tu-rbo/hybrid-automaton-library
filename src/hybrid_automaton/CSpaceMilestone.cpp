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
//dofs_(-1),
motion_behaviour_(NULL)
//object_id_(-1)
{
}

CSpaceMilestone::CSpaceMilestone(std::string csm_name) : 
Milestone(csm_name),
//dofs_(dofs),
motion_behaviour_(NULL)
//object_id_(-1)
{
}

CSpaceMilestone::CSpaceMilestone(std::string csm_name, std::vector<double>& configuration, MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius) :
Milestone(csm_name),
//dofs_(dofs),
configuration_(configuration),
region_convergence_radius_(region_convergence_radius)
//object_id_(object_id)
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
	//	if(this->motion_behaviour_)
	//	{
	//		this->motion_behaviour_->activate();
	//		double t = 0.0;
	//		for(int i = 0; i < 1000; ++i)
	//		{
	//			this->motion_behaviour_->update(t);
	//			t += 0.001;
	//		}
	//		::std::cout << " finished computing milestone" << ::std::endl;
	//		//this->motion_behaviour_->deactivate();
	//	}
}

CSpaceMilestone::CSpaceMilestone(TiXmlElement* milestone_xml, rxSystem* robot, double dt):
Milestone("temp"),
//dofs_(-1),
motion_behaviour_(NULL)
//object_id_(-1)
{
	XMLDeserializer xml_deserializer_(milestone_xml);
	this->status_ = (Status)xml_deserializer_.deserializeInteger("status");
	this->name_ = xml_deserializer_.deserializeString("name");
	//this->object_id_ = xml_deserializer_.deserializeInteger("objectId");
	//this->dofs_ = xml_deserializer_.deserializeInteger("DOFs");
	this->configuration_ = xml_deserializer_.deserializeVectorDouble("value");
	if(this->configuration_.size()==0)
	{
		throw std::string("ERROR: [CSpaceMilestone::CSpaceMilestone(TiXmlElement* milestone_xml, rxSystem* robot)] Child Element named \"value\" not found in XML element milestone_xml.");
	}
	this->region_convergence_radius_ = xml_deserializer_.deserializeVectorDouble("epsilon");
	if(this->region_convergence_radius_.size() == 0)
	{
		throw std::string("ERROR: [CSpaceMilestone::CSpaceMilestone(TiXmlElement* milestone_xml, rxSystem* robot)] Child Element named \"epsilon\" not found in XML element milestone_xml.");
	}

	TiXmlElement* handle_point_set_element = milestone_xml->FirstChildElement("HandlePoints");
	if(handle_point_set_element != NULL)
	{
	for(TiXmlElement* handle_point_element = handle_point_set_element->FirstChildElement("HandlePoint"); handle_point_element; handle_point_element = handle_point_element->NextSiblingElement())
	{
		XMLDeserializer xml_deserializer_handle_point(handle_point_element);
		Point handle_point_value(-1.f,-1.f,-1.f);
		handle_point_value.x = xml_deserializer_handle_point.deserializeDouble("x");
		handle_point_value.y = xml_deserializer_handle_point.deserializeDouble("y");
		handle_point_value.z = xml_deserializer_handle_point.deserializeDouble("z");
		(this->handle_points_).push_back(handle_point_value);
	}
	}

	TiXmlElement* mb_element = milestone_xml->FirstChildElement("MotionBehaviour");
	if(mb_element)
	{
		this->motion_behaviour_ = new MotionBehaviour(mb_element, this, this, robot, dt);
	}
}

CSpaceMilestone::CSpaceMilestone(const CSpaceMilestone & cmilestone_cpy) :
Milestone(cmilestone_cpy),
//dofs_(cmilestone_cpy.dofs_),
configuration_(cmilestone_cpy.configuration_),
region_convergence_radius_(cmilestone_cpy.region_convergence_radius_),
//object_id_(cmilestone_cpy.object_id_),
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

//int CSpaceMilestone::getObjectId() const
//{
//	return this->object_id_;
//}
//
//int CSpaceMilestone::getDofs() const
//{
//	return this->dofs_;
//}

void CSpaceMilestone::update()
{
	if(motion_behaviour_ != NULL){
		cout << "CSM> Updating CS Milestone" << endl;
	}else{
		cout << "CSM> Can't update anything, motion_behaviour_ is NULL!!!" << endl;
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
	//cspace_ms_xml->SetAttribute("ObjectId", object_id_);

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

//bool CSpaceMilestone::operator ==(const CSpaceMilestone & n) const
//{
//	return (this->name_ == n.getName());
//	//return (this->configuration_ == n.getConfigurationSTDVector() 
//	//	&& this->dofs_==n.getDofs() && this->handle_points_ == n.handle_points_ 
//	//	&& this->object_id_ == n.getObjectId());
//}
//
//bool CSpaceMilestone::operator ==(const Milestone & n) const
//{
//	const CSpaceMilestone* n_cspace = dynamic_cast<const CSpaceMilestone*>(&n);
//	if(n_cspace)
//		return (this->name_ == n_cspace->getName());
//		//return (this->configuration_ == n_cspace->getConfigurationSTDVector() 
//		//&& this->dofs_==n_cspace->getDofs() && this->handle_points_ == n_cspace->handle_points_ 
//		//&& this->object_id_ == n_cspace->getObjectId());
//	else
//		return false;
//}
//
//bool CSpaceMilestone::operator !=(const CSpaceMilestone & n) const
//{
//	return !(*this==n);
//}
//
//bool CSpaceMilestone::operator !=(const Milestone & n) const
//{
//	return !(*this==n);
//}

CSpaceMilestone& CSpaceMilestone::operator=(const CSpaceMilestone & cmilestone_assignment)
{
	Milestone::operator=(cmilestone_assignment);
	//this->dofs_ = cmilestone_assignment.dofs_;
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
	//this->object_id_ = cmilestone_assignment.object_id_;
	this->handle_points_ = cmilestone_assignment.handle_points_;
	return *this;
}

//bool CSpaceMilestone::operator ==(const Milestone * n) const{
//	const CSpaceMilestone* n_cspace = dynamic_cast<const CSpaceMilestone*>(n);
//	if(n_cspace)
//		return (configuration_ == n_cspace->configuration_);
//	else
//		return false;
//}
//bool CSpaceMilestone::operator ==(const CSpaceMilestone * n) const{
//	return (configuration_ == n->configuration_);
//}
//
//bool CSpaceMilestone::operator ==(CSpaceMilestone * n) const{
//	return (configuration_ == n->configuration_);
//}