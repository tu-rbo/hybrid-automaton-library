#include "MilestoneFactory.h"
#include "CSpaceMilestone.h"
#include "OpSpaceMilestone.h"
#include <iostream>
#include <string>

MilestoneFactory::MilestoneFactory()
{
}

MilestoneFactory::~MilestoneFactory()
{
}
	
Milestone* MilestoneFactory::createMilestone(TiXmlElement* milestone_xml, rxSystem* robot )
{
	std::string milestone_type = std::string(milestone_xml->Attribute("Type"));
	Milestone* return_milestone = NULL;

	if(milestone_type == "CSpaceMilestone"){
		return_milestone = new CSpaceMilestone(milestone_xml, robot);
	}else if(milestone_type == "OpSpaceMilestone"){
		return_milestone = new OpSpaceMilestone(milestone_xml, robot);
	}else{
		throw std::string("Wrong type of milestone");
	}
	return return_milestone;
}