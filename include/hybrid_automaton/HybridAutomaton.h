#ifndef HYBRID_AUTOMATON_
#define HYBRID_AUTOMATON_

#include "graph/include/MDP.h"
#include "graph/include/MDPNode.h"
#include "graph/include/MDPEdge.h"
#include "Milestone.h"
#include "MotionBehaviour.h"

/**
* HybridAutomaton class. 
* @brief Class extending DiGraph. It defines the types for the template to be Milestone and MotionBehaviour. Adds the required parser/deparser functionalities.
*/
class HybridAutomaton : public MDP
{
public:

	/**
	* Constructor. Creates an empty graph.
	*/
	HybridAutomaton();

	/**
	* Destructor.
	*/
	virtual ~HybridAutomaton();

	/**
	* Return a pointer to the first Milestone.
	*/
	Milestone* getStartNode() const;

	/**
	* Set the pointer to the first Milestone.
	* @param nodeID Pointer to the first Milestone.
	*/
	void setStartNode(Milestone* nodeID);

	/**
	* Return the pointer of a specific Milestone or NULL if it doesn't exist.
	* @param name Name of the Milestone.
	*/
	Milestone* getMilestoneByName(const std::string& name) const;

	/**
	* Return a string with XML format with all required parameters of the HybridAutomaton to recreate it 
	* in the other side of a network connection.
	* Note: It uses tinyXML library to create the string with XML format.
	*/
	virtual std::string toStringXML() const;

	/**
	* REPLACE the current values of the HybridAutomaton with the values contained in a string with XML format.
	* @param xmlString String with XML format containing the new values for the HybridAutomaton.
	* @param robot Pointer to the rxSystem, used to create the rxControllers.
	* @param dT Interval to be used by all the controllers in this HybridSystem
	* Note: It uses tinyXML library to read the string with XML format.
	*/
	//virtual void fromStringXML(const std::string& xml_string, rxSystem* robot, double dT);

private:
	Milestone* start_node_id_;	// Pointer to the first Milestone (if exists). 
	// CAREFUL! This pointer is the same as the stored in the vector of Milestones. Don't modify/delete it!
};

/**
* Creates an ostream with the data and parameters of a HybridAutomaton.
* @param out Object to add the data and be returned.
* @param hybrid_system HybridAutomaton to be parsed.
*/
ostream& operator<<(ostream & out, const HybridAutomaton & hybrid_system);

#endif