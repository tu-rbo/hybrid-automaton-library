/*
 * Factories.h
 *
 *  Created on: Jun 30, 2011
 *      Author: martin
 */

#ifndef MILESTONE_FACTORY_H_
#define MILESTONE_FACTORY_H_

#include "tinyxml.h"
#include "rxControlSDK\rxControlSDK.h"
#include "Milestone.h"

class MilestoneFactory
{
public:
	MilestoneFactory();
	virtual ~MilestoneFactory();
	Milestone* createMilestone(TiXmlElement* milestone_xml, rxSystem* robot );
};

#endif //MILESTONE_FACTORY_H_