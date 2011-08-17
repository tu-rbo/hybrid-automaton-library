/*
 *\brief NetworkUpdatable class
 *\detailed The NetworkUpdatable class is an interface for objects that can be
 *          updated by a network object (ie. new data received over the network)
 *          The update is done a-synch, allowing the updated object not to hang while
 *          waiting for network events.
 *$Author: dubik $
 *$Date: 2007/04/17 17:31:27 $
 *$Revision: 1.4 $
 */

#ifndef __NETWORKUPDATABLE__
#define __NETWORKUPDATABLE__

#include "msgs\NetworkData.h"

class NetworkUpdatable {
public:
	virtual void update(rlab::NetworkData *s) = 0;
	virtual ~NetworkUpdatable(){}
};

#endif

