#include "MilestoneFactory.h"
#include "CSpaceMilestone.h"
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
	}else{
		throw std::string("Wrong time of milestone");
	}
	return return_milestone;
}