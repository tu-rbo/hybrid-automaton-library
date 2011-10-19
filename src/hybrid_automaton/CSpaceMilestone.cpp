#include "CSpaceMilestone.h"

#include <iostream>
#include <string>

#include "tinyxml.h"
#include "XMLDeserializer.h"
#include "MilestoneFactory.h"

using namespace std;

#define RADIUS 0.4

//TODO: (When creating milestone or setting the MB) Is ok to force the MB to have this Milestone as parent and child?

CSpaceMilestone::CSpaceMilestone() :
Milestone(),
dofs_(-1),
motion_behaviour_(NULL),
object_id_(-1)
{
}

CSpaceMilestone::CSpaceMilestone(int dofs) : 
Milestone(),
dofs_(dofs),
motion_behaviour_(NULL),
object_id_(-1)
{
}

CSpaceMilestone::CSpaceMilestone(int dofs, std::vector<double>& configuration, MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius,int object_id) :
Milestone(),
dofs_(dofs),
configuration_(configuration),
region_convergence_radius_(region_convergence_radius),
object_id_(object_id)
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

CSpaceMilestone::CSpaceMilestone(TiXmlElement* milestone_xml, rxSystem* robot):
Milestone(),
dofs_(-1),
motion_behaviour_(NULL),
object_id_(-1)
{
	XMLDeserializer xml_deserializer_(milestone_xml);
	this->setStatus((Status)xml_deserializer_.deserializeInteger("Status"));
	this->object_id_ = xml_deserializer_.deserializeInteger("ObjectId");
	this->dofs_ = xml_deserializer_.deserializeInteger("DOFs");

	TiXmlElement* configuration_element = milestone_xml->FirstChildElement("Configuration");
	if(configuration_element == NULL)
	{
		throw std::string("ERROR: [CSpaceMilestone::CSpaceMilestone(TiXmlElement* milestone_xml, rxSystem* robot)] Child Element named \"Configuration\" not found in XML element milestone_xml.");
	}

	for(TiXmlElement* joint_element = configuration_element->FirstChildElement("Joint"); joint_element; joint_element = joint_element->NextSiblingElement())
	{
		XMLDeserializer xml_deserializer_joint(joint_element);
		(this->configuration_).push_back(xml_deserializer_joint.deserializeDouble("value"));
	}

	TiXmlElement* radius_convergence_element = milestone_xml->FirstChildElement("RadiusRoC");
	if(radius_convergence_element == NULL)
	{
		throw std::string("ERROR: [CSpaceMilestone::CSpaceMilestone(TiXmlElement* milestone_xml, rxSystem* robot)] Child Element named \"RadiusRoC\" not found in XML element milestone_xml.");
	}

	for(TiXmlElement* radius_element = radius_convergence_element->FirstChildElement("Radius"); radius_element; radius_element = radius_element->NextSiblingElement())
	{
		XMLDeserializer xml_deserializer_radius(radius_element);
		(this->region_convergence_radius_).push_back(xml_deserializer_radius.deserializeDouble("value"));		
	}

	TiXmlElement* handle_point_set_element = milestone_xml->FirstChildElement("HandlePoints");
	if(handle_point_set_element == NULL)
	{
		throw std::string("ERROR: [CSpaceMilestone::CSpaceMilestone(TiXmlElement* milestone_xml, rxSystem* robot)] Child Element named \"HandlePoints\" not found in XML element milestone_xml.");
	}

	for(TiXmlElement* handle_point_element = handle_point_set_element->FirstChildElement("HandlePoint"); handle_point_element; handle_point_element = handle_point_element->NextSiblingElement())
	{
		XMLDeserializer xml_deserializer_handle_point(handle_point_element);
		Point handle_point_value(-1.f,-1.f,-1.f);
		handle_point_value.x = xml_deserializer_handle_point.deserializeDouble("x");
		//std::cout << handle_point_value.x << std::endl;
		handle_point_value.y = xml_deserializer_handle_point.deserializeDouble("y");
		//std::cout << handle_point_value.y << std::endl;
		handle_point_value.z = xml_deserializer_handle_point.deserializeDouble("z");
		//std::cout << handle_point_value.z << std::endl;
		(this->handle_points_).push_back(handle_point_value);
	}

	TiXmlElement* controller_element = milestone_xml->FirstChildElement("Controller");
	if(controller_element == NULL)
	{
		throw std::string("ERROR: [CSpaceMilestone::CSpaceMilestone(TiXmlElement* milestone_xml, rxSystem* robot)] Child Element named \"Controller\" not found in XML element milestone_xml.");
	}

	const char* controller_pointer = controller_element->Attribute("Pointer");
	if( controller_pointer && std::string(controller_pointer)=="NULL")
	{
		this->motion_behaviour_ = NULL;
	}else
	{
		//this->motion_behaviour_ = createMotionBehaviour(controller_element, this, this, robot);
		this->motion_behaviour_ = new MotionBehaviour(controller_element, this, this, robot);
	}
}

CSpaceMilestone::CSpaceMilestone(const CSpaceMilestone & cmilestone_cpy) :
Milestone(cmilestone_cpy),
dofs_(cmilestone_cpy.dofs_),
configuration_(cmilestone_cpy.configuration_),
region_convergence_radius_(cmilestone_cpy.region_convergence_radius_),
object_id_(cmilestone_cpy.object_id_),
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

std::vector<double> CSpaceMilestone::getConfiguration() const
{
	return this->configuration_;	
}

int CSpaceMilestone::getObjectId() const
{
	return this->object_id_;
}

int CSpaceMilestone::getDofs() const
{
	return this->dofs_;
}

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
	TiXmlElement * node_element = new TiXmlElement("Node");
	document_xml.LinkEndChild(node_element);

	this->toElementXML(node_element);

	// Declare a printer
	TiXmlPrinter printer_xml;
	// attach it to the document you want to convert in to a std::string
	document_xml.Accept(&printer_xml);
	// Create a std::string and copy your document data in to the string
	std::string return_value = printer_xml.CStr();

	//TODO: Memory leaks?????????
	return return_value;
}

void CSpaceMilestone::toElementXML(TiXmlElement* cspace_milestone_xml) const 
{
	cspace_milestone_xml->SetAttribute("Type", "CSpaceMilestone");
	cspace_milestone_xml->SetAttribute("Status", this->getStatus());
	cspace_milestone_xml->SetAttribute("ObjectId", object_id_);
	cspace_milestone_xml->SetAttribute("DOFs", dofs_);

	TiXmlElement * configurations_txe = new TiXmlElement("Configuration");
	cspace_milestone_xml->LinkEndChild(configurations_txe);
	configurations_txe->SetAttribute("Size", configuration_.size());
	std::vector<double>::const_iterator config_it = configuration_.begin();
	for (; config_it
		!= configuration_.end(); config_it++) {
			TiXmlElement * configuration_txe;
			configuration_txe = new TiXmlElement("Joint");
			configurations_txe->LinkEndChild(configuration_txe);
			configuration_txe->SetDoubleAttribute("value", *config_it);
	}

	TiXmlElement * rocrs_txe = new TiXmlElement("RadiusRoC");
	cspace_milestone_xml->LinkEndChild(rocrs_txe);
	rocrs_txe->SetAttribute("Size", region_convergence_radius_.size());
	std::vector<double>::const_iterator rocr_it = region_convergence_radius_.begin();
	for (; rocr_it
		!= region_convergence_radius_.end(); rocr_it++) {
			TiXmlElement * roc_txe;
			roc_txe = new TiXmlElement("Radius");
			rocrs_txe->LinkEndChild(roc_txe);
			roc_txe->SetDoubleAttribute("value", *rocr_it);
	}

	TiXmlElement * handlePoints_txe = new TiXmlElement("HandlePoints");
	cspace_milestone_xml->LinkEndChild(handlePoints_txe);
	handlePoints_txe->SetAttribute("Size", handle_points_.size());
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

	//TODO: How insert the information of MotionBehaviour* motion_behaviour_ as the result of a method of MotionBehaviour?
	TiXmlElement * controller_txe = new TiXmlElement("Controller");
	cspace_milestone_xml->LinkEndChild(controller_txe);

	if(motion_behaviour_){
		motion_behaviour_->toElementXML(controller_txe);
	}else{
		controller_txe->SetAttribute("Pointer", "NULL");
	}
}

CSpaceMilestone* CSpaceMilestone::clone() const
{
	CSpaceMilestone* new_c_space_milestone = new CSpaceMilestone(*this);
	return new_c_space_milestone;
}

bool CSpaceMilestone::operator ==(const CSpaceMilestone & n) const
{
	return (this->configuration_ == n.getConfiguration() 
		&& this->dofs_==n.getDofs() && this->handle_points_ == n.handle_points_ 
		&& this->object_id_ == n.getObjectId());
}

bool CSpaceMilestone::operator ==(const Milestone & n) const
{
	const CSpaceMilestone* n_cspace = dynamic_cast<const CSpaceMilestone*>(&n);
	if(n_cspace)
		return (this->configuration_ == n_cspace->getConfiguration() 
		&& this->dofs_==n_cspace->getDofs() && this->handle_points_ == n_cspace->handle_points_ 
		&& this->object_id_ == n_cspace->getObjectId());
	else
		return false;
}

bool CSpaceMilestone::operator !=(const CSpaceMilestone & n) const
{
	return !(*this==n);
}

bool CSpaceMilestone::operator !=(const Milestone & n) const
{
	return !(*this==n);
}

CSpaceMilestone& CSpaceMilestone::operator=(const CSpaceMilestone & cmilestone_assignment)
{
	Milestone::operator=(cmilestone_assignment);
	this->dofs_ = cmilestone_assignment.dofs_;
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
	this->object_id_ = cmilestone_assignment.object_id_;
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