#include "OpSpaceMilestone.h"

#include <iostream>
#include <string>

#include "tinyxml.h"
#include "XMLDeserializer.h"
#include "Quaternion.h"

using namespace std;

#define RADIUS 0.4

OpSpaceMilestone::OpSpaceMilestone() :
Milestone("Default"),
motion_behaviour_(NULL),
posi_ori_selection_(POSITION_SELECTION)	// Default value in the default constructor
{
}

OpSpaceMilestone::OpSpaceMilestone(std::string osm_name) :
Milestone(osm_name),
motion_behaviour_(NULL),
posi_ori_selection_(POSITION_SELECTION)	// Default value in the default constructor
{
}

OpSpaceMilestone::OpSpaceMilestone(std::string osm_name, std::vector<double>& posi_ori_value, PosiOriSelector posi_ori_selection, MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius) :
Milestone(osm_name),
posi_ori_selection_(posi_ori_selection)
{
	configuration_.resize(6, -1.0);
	region_convergence_radius_.resize(4, -1.0);
	int counter_offset = -1;
	int max_count = -1;
	switch(posi_ori_selection_){
		case POSITION_SELECTION:
			counter_offset = 0;
			max_count = 3;
			break;
		case ORIENTATION_SELECTION:
			counter_offset = 3;
			max_count = 4;
			break;
		case POS_AND_ORI_SELECTION:
			counter_offset = 0;
			max_count = 4;
			break;
		default:
			throw std::string("ERROR [OpSpaceMilestone::OpSpaceMilestone(std::vector<double>& posi_ori_value, PosiOriSelection posi_ori_selection, MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius,int object_id)]: Wrong posi_ori_selection value.");
			break;
	}
	int i = counter_offset;
	int n = 0;
	for(; i<max_count; i++, n++){
		configuration_[i] = posi_ori_value[n];
		region_convergence_radius_[i] = region_convergence_radius[n];
	}
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

	// TODO (Roberto): This is old code from Lefteris. It sets the handle points of a Milestone (for ray-tracing) to be 4 points:
	// Two adding to the center point + or -RADIUS in x direction and two adding + or -RADIUS in y direction.
	/*Point tmp(configuration_[0],configuration_[1],0.0);
	tmp.x += RADIUS;
	handle_points_.push_back(tmp);
	tmp.x -= 2 * RADIUS;
	handle_points_.push_back(tmp);
	tmp.x += RADIUS;
	tmp.y += RADIUS;
	handle_points_.push_back(tmp);
	tmp.y -= 2 * RADIUS;
	handle_points_.push_back(tmp);*/
}

OpSpaceMilestone::OpSpaceMilestone(std::string osm_name, std::vector<double>& posi_ori_value, PosiOriSelector posi_ori_selection, 
								   MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius, Milestone::Status status, 
								   std::vector<Point> handle_points):
Milestone(osm_name),
posi_ori_selection_(posi_ori_selection),
handle_points_(handle_points)
{
	this->status_ = status;
	configuration_.resize(6, -1.0);
	region_convergence_radius_.resize(4, -1.0);
	int counter_offset = -1;
	int max_count = -1;
	switch(posi_ori_selection_){
		case POSITION_SELECTION:
			counter_offset = 0;
			max_count = 3;
			break;
		case ORIENTATION_SELECTION:
			counter_offset = 3;
			max_count = 4;
			break;
		case POS_AND_ORI_SELECTION:
			counter_offset = 0;
			max_count = 4;
			break;
		default:
			throw std::string("ERROR [OpSpaceMilestone::OpSpaceMilestone(std::vector<double>& posi_ori_value, PosiOriSelection posi_ori_selection, MotionBehaviour * motion_behaviour, std::vector<double>& region_convergence_radius,int object_id)]: Wrong posi_ori_selection value.");
			break;
	}
	int i = counter_offset;
	int n = 0;
	for(; i<max_count; i++, n++){
		configuration_[i] = posi_ori_value[n];
		region_convergence_radius_[i] = region_convergence_radius[n];
	}
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

OpSpaceMilestone::OpSpaceMilestone(const OpSpaceMilestone & op_milestone_cpy) :
Milestone(op_milestone_cpy),
configuration_(op_milestone_cpy.configuration_),
region_convergence_radius_(op_milestone_cpy.region_convergence_radius_),
handle_points_(op_milestone_cpy.handle_points_),
posi_ori_selection_(op_milestone_cpy.posi_ori_selection_)
{
	if(op_milestone_cpy.motion_behaviour_)
	{
		this->motion_behaviour_ = op_milestone_cpy.motion_behaviour_->clone();
		this->motion_behaviour_->setChild(this);
		this->motion_behaviour_->setParent(this);
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}
}

OpSpaceMilestone::~OpSpaceMilestone()
{	
	// Do not delete anything but the MotionBehaviour, it is the only thing created/copied in the constructor
	delete motion_behaviour_;
	motion_behaviour_ = NULL;
}

void OpSpaceMilestone::setMotionBehaviour(MotionBehaviour * motion_behaviour)
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

void OpSpaceMilestone::setConfigurationSTDVector(std::vector<double> configuration_in, PosiOriSelector posi_ori_selector)
{
	configuration_.resize(6);
	int counter_offset = -1;
	int max_count = -1;
	switch(posi_ori_selector){
		case POSITION_SELECTION:
			counter_offset = 0;
			max_count = 3;
			switch(this->posi_ori_selection_){
		case ORIENTATION_SELECTION:
			this->posi_ori_selection_ = POS_AND_ORI_SELECTION;
			break;
		case POSITION_SELECTION:
		case POS_AND_ORI_SELECTION:
			break;
		default:
			this->posi_ori_selection_ = POSITION_SELECTION;
			break;
			}
			break;
		case ORIENTATION_SELECTION:
			switch(this->posi_ori_selection_){
		case POSITION_SELECTION:
			this->posi_ori_selection_ = POS_AND_ORI_SELECTION;
			break;
		case ORIENTATION_SELECTION:
		case POS_AND_ORI_SELECTION:
			break;
		default:
			this->posi_ori_selection_ = ORIENTATION_SELECTION;
			break;
			}
			counter_offset = 3;
			max_count = 6;
			break;
		case POS_AND_ORI_SELECTION:
			counter_offset = 0;
			max_count = 6;
			this->posi_ori_selection_ = POS_AND_ORI_SELECTION;
			break;
		default:
			throw std::string("ERROR [OpSpaceMilestone::setConfigurationSTDVector(std::vector<double> configuration_in, PosiOriSelector posi_ori_selector)]: Wrong posi_ori_selection value.");
			break;
	}
	int i = counter_offset;
	int n = 0;
	for(; i<max_count; i++, n++){
		configuration_[i] = configuration_in[n];
	}
}

std::vector<double> OpSpaceMilestone::getConfigurationSTDVector() const
{
	return this->configuration_;	
}

void OpSpaceMilestone::setConfiguration(dVector configuration_in, PosiOriSelector posi_ori_selector)
{
	configuration_.resize(6);
	int counter_offset = -1;
	int max_count = -1;
	switch(posi_ori_selector){
		case POSITION_SELECTION:
			counter_offset = 0;
			max_count = 3;
			switch(this->posi_ori_selection_){
		case ORIENTATION_SELECTION:
			this->posi_ori_selection_ = POS_AND_ORI_SELECTION;
			break;
		case POSITION_SELECTION:
		case POS_AND_ORI_SELECTION:
			break;
		default:
			this->posi_ori_selection_ = POSITION_SELECTION;
			break;
			}
			break;
		case ORIENTATION_SELECTION:
			switch(this->posi_ori_selection_){
		case POSITION_SELECTION:
			this->posi_ori_selection_ = POS_AND_ORI_SELECTION;
			break;
		case ORIENTATION_SELECTION:
		case POS_AND_ORI_SELECTION:
			break;
		default:
			this->posi_ori_selection_ = ORIENTATION_SELECTION;
			break;
			}
			counter_offset = 3;
			max_count = 6;
			break;
		case POS_AND_ORI_SELECTION:
			counter_offset = 0;
			max_count = 6;
			this->posi_ori_selection_ = POS_AND_ORI_SELECTION;
			break;
		default:
			throw std::string("ERROR [OpSpaceMilestone::setConfigurationSTDVector(std::vector<double> configuration_in, PosiOriSelector posi_ori_selector)]: Wrong posi_ori_selection value.");
			break;
	}
	int i = counter_offset;
	int n = 0;
	for(; i<max_count; i++, n++){
		configuration_[i] = configuration_in[n];
	}
}

dVector OpSpaceMilestone::getConfiguration() const
{
	// Using 6 values for the configuration: 3 position + 3 orientation
	dVector ret_value(6);
	ret_value.all(0);
	for(int i = 0; i<6; i++)
	{
		ret_value[i] = configuration_[i];
	}
	return ret_value;
}

void OpSpaceMilestone::update()
{
	if(motion_behaviour_ != NULL){
		cout << "CSM> Updating OpS Milestone" << endl;
	}else{
		cout << "CSM> Can't update anything, motion_behaviour_ is NULL!!!" << endl;
	}
}

std::string OpSpaceMilestone::toStringXML() const
{
	TiXmlDocument document_xml;
	TiXmlElement * node_element = this->toElementXML();
	document_xml.LinkEndChild(node_element);

	// Declare a printer
	TiXmlPrinter printer_xml;
	// attach it to the document you want to convert in to a std::string
	document_xml.Accept(&printer_xml);
	// Create a std::string and copy your document data in to the string
	std::string return_value = printer_xml.CStr();

	//TODO: Memory leaks?????????
	delete node_element;
	return return_value;
}

TiXmlElement* OpSpaceMilestone::toElementXML() const 
{
	TiXmlElement* op_space_ms_xml = new TiXmlElement("Milestone");
	op_space_ms_xml->SetAttribute("type", "OpSpace");
	op_space_ms_xml->SetAttribute("status", this->getStatus());
	op_space_ms_xml->SetAttribute("name", this->name_.c_str());
	op_space_ms_xml->SetAttribute("PosiOriSelector", posi_ori_selection_);

	std::stringstream value_ss;
	for(unsigned int i=0; i<this->configuration_.size()-1; i++)
	{
		value_ss << configuration_.at(i) << " ";
	}
	value_ss << configuration_.at(configuration_.size()-1);
	op_space_ms_xml->SetAttribute("value", value_ss.str().c_str());

	std::stringstream epsilon_ss;
	for(unsigned int i=0; i<this->region_convergence_radius_.size()-1; i++)
	{
		epsilon_ss << region_convergence_radius_.at(i) << " ";
	}
	epsilon_ss << region_convergence_radius_.at(region_convergence_radius_.size()-1);
	op_space_ms_xml->SetAttribute("epsilon", epsilon_ss.str().c_str());

	TiXmlElement * handlePoints_txe = new TiXmlElement("HandlePoints");
	op_space_ms_xml->LinkEndChild(handlePoints_txe);
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
		op_space_ms_xml->LinkEndChild(this->motion_behaviour_->toElementXML());
	}
	return op_space_ms_xml;
}

OpSpaceMilestone* OpSpaceMilestone::clone() const
{
	OpSpaceMilestone* new_op_space_milestone = new OpSpaceMilestone(*this);
	return new_op_space_milestone;
}

OpSpaceMilestone& OpSpaceMilestone::operator=(const OpSpaceMilestone & op_milestone_assignment)
{
	Milestone::operator=(op_milestone_assignment);
	this->configuration_ = op_milestone_assignment.getConfigurationSTDVector();
	if(op_milestone_assignment.motion_behaviour_)
	{
		this->motion_behaviour_ = op_milestone_assignment.motion_behaviour_->clone();
	}
	else
	{
		this->motion_behaviour_ = NULL;
	}
	this->region_convergence_radius_ = op_milestone_assignment.region_convergence_radius_;
	//this->object_id_ = op_milestone_assignment.getObjectId();
	this->handle_points_ = op_milestone_assignment.handle_points_;
	this->posi_ori_selection_ = op_milestone_assignment.getPosiOriSelector();
	return *this;
}

Displacement OpSpaceMilestone::getHandlePoint(int i) const
{
	return Displacement(this->handle_points_[i].x, this->handle_points_[i].y, this->handle_points_[i].z);
}

int OpSpaceMilestone::getHandlePointNumber() const
{
	return handle_points_.size();
}

PosiOriSelector OpSpaceMilestone::getPosiOriSelector() const
{
	return posi_ori_selection_;
}

bool OpSpaceMilestone::hasConverged(rxSystem* sys) 
{
	assert (sys != NULL);

	
	HTransform ht;
	rxBody* EE = sys->getUCSBody(_T("EE"),ht);
	if (posi_ori_selection_ == POSITION_SELECTION
		|| posi_ori_selection_ == POS_AND_ORI_SELECTION)
	{
		Displacement current_r = ht.r + EE->T().r; //parent->getConfiguration();
		for (int i = 0; i < 3; i++) {;
			double e = ::std::abs(current_r[i] - configuration_[i]);
			if (e > region_convergence_radius_[i])
			{
#ifdef NOT_IN_RT
				std::cout << "Error in pos " << i << " is too large = " << e << std::endl;
#endif
				return false;
			}
		}
	}

	if (posi_ori_selection_ == ORIENTATION_SELECTION
		|| posi_ori_selection_ == POS_AND_ORI_SELECTION)
	{
		// desired orientation
		Rotation Rd(configuration_[3],
			configuration_[4],
			configuration_[5]);

		dVector quatd;
		Rd.GetQuaternion(quatd);
		Quaternion qd(quatd);

		// current orientation
		dVector quat;
		EE->T().R.GetQuaternion(quat);
		Quaternion q(quat);

		q.invert();
		Quaternion qres = q*qd;
		dVector axis;
		double angle;
		qres.to_axis_angle(axis, angle);

		double e = ::std::abs(angle);
		if (e > region_convergence_radius_[3]) 
		{
#ifdef NOT_IN_RT
			std::cout << "Error in orientation is too large = " << e << std::endl;
#endif
			return false;
		}
	}

	return true;
}