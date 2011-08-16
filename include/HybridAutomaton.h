#ifndef HYBRID_AUTOMATON_
#define HYBRID_AUTOMATON_

#include "DiGraph.h"
#include "node.h"
#include "edge.h"
#include "Milestone.h"
#include "MotionBehaviour.h"

/**
* HybridAutomaton class. 
* @brief Class extending DiGraph. It defines the types for the template to be Milestone and MotionBehaviour. Adds the required parser/deparser functionalities.
*/
class HybridAutomaton : public DiGraph < Milestone, MotionBehaviour >
{
public:

	/**
	* Constructor.
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
	* Return a string with XML format with all required parameters of the HybridAutomaton to recreate it 
	* in the other side of a network connection.
	* Note: It uses tinyXML library to create the string with XML format.
	*/
	virtual std::string toStringXML() const;

	/**
	* REPLACE the current values of the HybridAutomaton with the values contained in a string with XML format.
	* @param xmlString String with XML format containing the new values for the HybridAutomaton.
	* @param robot Pointer to the rxSystem, used to create the rxControllers.
	* Note: It uses tinyXML library to read the string with XML format.
	*/
	virtual void fromStringXML(std::string xmlString, rxSystem* robot);

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