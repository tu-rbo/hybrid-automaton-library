#include "Milestone.h"
#include "CSpaceMilestone.h"
#include "CSpaceBlackBoardMilestone.h"
#include "OpSpaceMilestone.h"

#include "MotionBehaviour.h"

#include "HybridAutomaton.h"

#include "XMLDeserializer.h"

#include "tinyxml.h"


std::string HybridAutomaton::toStringXML() const
{
	// If the HybridAutomaton is empty, return the corresponding string
	if(nodeNumber==0 && edgeNumber==0)
		return std::string("Empty HybridAutomaton");

	// Create the base document
	TiXmlDocument document;
	// Add the declaration
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	// Create the first (and only) root element and link it to the base document
	TiXmlElement * hyb = new TiXmlElement("HybridAutomaton");
	document.LinkEndChild(hyb);

	// Every new no-empty HybridAutomaton gets a new name
	static int name_counter = 0;
	std::string name_str;
	std::stringstream name_ss;
	name_ss << name_counter;
	name_str = name_ss.str();
	std::string name_complete = std::string("HS") + name_str;
	name_counter++;

	hyb->SetAttribute("Name", name_complete.c_str());

	if(start_node_id_ < 0)
		throw std::string("[HybridAutomaton::toStringXML] ERROR: Start node is non-valid.");

	hyb->SetAttribute("InitialMilestone", start_node_id_->getName().c_str());

	// Add the data of the nodes-milestones
	for(unsigned int i = 0; i<nodeNumber ; i++){
        Milestone* ms=(Milestone*)nodeList.at(i);
		if(ms!=NULL){
			TiXmlElement * mst_element = ms->toElementXML();
			hyb->LinkEndChild(mst_element);			
		}
	}

	// Add the data of the edges-motionbehaviours
	for(unsigned int i = 0; i<nodeNumber ; i++){
		for(unsigned int j = 0; j<nodeNumber ; j++){
            MotionBehaviour* mb=(MotionBehaviour*)adjacencyMatrix.at(i).at(j); 
			if( mb != NULL){
				TiXmlElement * mb_element = mb->toElementXML();
				hyb->LinkEndChild(mb_element);
			}
		}
	}

	// Declare a printer
	TiXmlPrinter printer;
	// Attach it to the document you want to convert in to a std::string
	document.Accept(&printer);
	// Create a std::string and copy your document data in to the string
	std::string ret_val = printer.CStr();

	//TODO: Memory leaks?
	return ret_val;
}

ostream& operator<<(ostream & out, const HybridAutomaton & hybrid_system){
	out << hybrid_system.toStringXML();
	return out;
}

std::string Milestone::toStringXML() const 
{
	throw std::string("WARNING: [Milestone::toStringXML()] Empty method. Derived class' method should have been called instead.");
}

TiXmlElement* Milestone::toElementXML() const 
{
	throw std::string("WARNING: [Milestone::toElementXML(TiXmlElement* root)] Empty method. Derived class' method should have been called instead.");
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
	for(unsigned int i=0; i<this->configuration_.size()-1; i++)
	{
		value_ss << configuration_.at(i) << " ";
	}
	value_ss << configuration_.at(configuration_.size()-1);
	cspace_ms_xml->SetAttribute("value", value_ss.str().c_str());

	std::stringstream epsilon_ss;
	for(unsigned int i=0; i<this->region_convergence_radius_.size()-1; i++)
	{
		epsilon_ss << region_convergence_radius_.at(i) << " ";
	}
	epsilon_ss << region_convergence_radius_.at(region_convergence_radius_.size()-1);
	cspace_ms_xml->SetAttribute("epsilon", epsilon_ss.str().c_str());

    cspace_ms_xml->SetDoubleAttribute("expectedLength", this->getExpectedLength());

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

std::string CSpaceBlackBoardMilestone::toStringXML() const
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

TiXmlElement* CSpaceBlackBoardMilestone::toElementXML() const 
{
	TiXmlElement* cspace_ms_xml = new TiXmlElement("Milestone");
	cspace_ms_xml->SetAttribute("type", "CSpace");
	cspace_ms_xml->SetAttribute("status", this->status_);
	cspace_ms_xml->SetAttribute("name", this->name_.c_str());

	std::stringstream value_ss;
	for(unsigned int i=0; i<this->configuration_.size()-1; i++)
	{
		value_ss << configuration_.at(i) << " ";
	}
	value_ss << configuration_.at(configuration_.size()-1);
	cspace_ms_xml->SetAttribute("value", value_ss.str().c_str());

	std::stringstream epsilon_ss;
	for(unsigned int i=0; i<this->region_convergence_radius_.size()-1; i++)
	{
		epsilon_ss << region_convergence_radius_.at(i) << " ";
	}
	epsilon_ss << region_convergence_radius_.at(region_convergence_radius_.size()-1);
	cspace_ms_xml->SetAttribute("epsilon", epsilon_ss.str().c_str());

    cspace_ms_xml->SetDoubleAttribute("expectedLength", this->getExpectedLength());

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

    op_space_ms_xml->SetDoubleAttribute("expectedLength", this->getExpectedLength());

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

std::string MotionBehaviour::toStringXML() const
{
	TiXmlDocument document_xml;
	TiXmlElement * mb_element = this->toElementXML();	
	document_xml.LinkEndChild(mb_element);

	// Declare a printer
	TiXmlPrinter printer_xml;
	// Attach it to the document you want to convert in to a std::string
	document_xml.Accept(&printer_xml);
	// Create a std::string and copy your document data in to the string
	std::string return_value = printer_xml.CStr();

	//TODO: Memory leaks????????????????
	//delete mb_element;
	return return_value;
}

TiXmlElement* MotionBehaviour::toElementXML() const
{
	TiXmlElement * mb_element = new TiXmlElement("MotionBehaviour");
	TiXmlElement * control_set_element = new TiXmlElement("ControlSet");
	mb_element->LinkEndChild(control_set_element);

	if(dynamic_cast<rxTPOperationalSpaceControlSet*>(control_set_))
	{
		control_set_element->SetAttribute("type", "rxTPOperationalSpaceControlSet");
	}
	else if(dynamic_cast<rxControlSet*>(control_set_))
	{
		control_set_element->SetAttribute("type", "rxControlSet");
	}

	std::list<rxController*> controllers = control_set_->getControllers();
	string_type controllers_to_string;
	for(std::list< rxController* >::const_iterator controllers_it = controllers.begin();
		controllers_it != controllers.end() ; controllers_it ++){
			string_type controller_to_string;
			(*controllers_it)->toString(controller_to_string);
			TiXmlElement * rxController_xml = new TiXmlElement("Controller");
			control_set_element->LinkEndChild(rxController_xml);			
			bool is_goal_controller = this->goal_controllers_.find((*controllers_it)->name())->second;
			this->RLabInfoString2ElementXML_(controller_to_string, rxController_xml, is_goal_controller);
			rxController_xml->SetAttribute("goalController", (is_goal_controller ? "true" : "false"));
			rxController_xml->SetAttribute("priority", (*controllers_it)->priority());
	}
	
    mb_element->SetAttribute("Parent",  ((Milestone*)parent)->getName().c_str());
    mb_element->SetAttribute("Child",   ((Milestone*)child )->getName().c_str());

    mb_element->SetDoubleAttribute("probability",  this->getProbability());
    mb_element->SetDoubleAttribute("length",this->getLength());

	return mb_element;
}

void MotionBehaviour::RLabInfoString2ElementXML_(string_type string_data, TiXmlElement* out_xml_element, bool is_goal_controller) const
{
#ifdef NOT_IN_RT
	//std::wcout << string_data.c_str() << std::endl;
#endif
	std::wstringstream data_ss(string_data);
	string_type temp_st;
	std::getline(data_ss, temp_st);	
	out_xml_element->SetAttribute("type", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());

	ControllerType type_of_controller = XMLDeserializer::controller_map_[XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))];

	std::getline(data_ss, temp_st);		// Discard field "name"
	std::getline(data_ss, temp_st);		// Discard field "system"
	std::getline(data_ss, temp_st);		// Discard field "type"
	std::getline(data_ss, temp_st);		// Field "dim"
	int dimension_int = -1;
	std::wstringstream dimension_ss(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")));
	dimension_ss >> dimension_int;
	std::getline(data_ss, temp_st);		// Discard field "dt"
	std::getline(data_ss, temp_st);		// Discard field "activated"
	std::getline(data_ss, temp_st);		// Field "ik"
	out_xml_element->SetAttribute("ik", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());

	string_type kp_st;
	string_type kv_st;
	string_type invL2sqr_st;
	for(int i=0; i<dimension_int ; i++)
	{
		std::getline(data_ss, temp_st);	
		kp_st += temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"));
		if(i!=dimension_int-1)
			kp_st += L" ";

		std::getline(data_ss, temp_st);	
		kv_st += temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"));
		if(i!=dimension_int-1)
			kv_st += L" ";

		std::getline(data_ss, temp_st);	
		invL2sqr_st += temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"));
		if(i!=dimension_int-1)
			invL2sqr_st += L" ";
	}
	out_xml_element->SetAttribute("kp",XMLDeserializer::wstring2string(kp_st).c_str());
	out_xml_element->SetAttribute("kv",XMLDeserializer::wstring2string(kv_st).c_str());
	out_xml_element->SetAttribute("invL2sqr",XMLDeserializer::wstring2string(invL2sqr_st).c_str());

	std::getline(data_ss, temp_st);			// Field "number of via points"
	int num_via_points_int = -1;
	std::wstringstream num_via_points_ss(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")));
	num_via_points_ss >> num_via_points_int;

	for(int i=0; i< num_via_points_int; i++)
	{
		std::getline(data_ss, temp_st);		// Field "time" of via points
		if(!is_goal_controller)
		{
			std::wstringstream time_goal_ss(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")));
			double time_goal = -1.;
			time_goal_ss >> time_goal;
			out_xml_element->SetDoubleAttribute("timeGoal", time_goal);
		}
		std::getline(data_ss, temp_st);		// Field "type" of via points
		if(!is_goal_controller)
		{
			std::wstringstream type_goal_ss(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")));
			int type_goal = -1;
			type_goal_ss >> type_goal;
			out_xml_element->SetAttribute("typeGoal", type_goal) ;
		}
		std::getline(data_ss, temp_st);		// Field "reuse" of via points
		if(!is_goal_controller)
		{
			std::string reuse_s = XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")));			
			out_xml_element->SetAttribute("reuseGoal",reuse_s.c_str() ) ;			
		}
		switch( type_of_controller.first )
		{
		case rxController::eControlType_Joint:
			std::getline(data_ss, temp_st);		// Discard field "dVector" of via points 
			if(!is_goal_controller)
			{
				out_xml_element->SetAttribute("dVectorGoal", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str());
			}
			break;
		case rxController::eControlType_Displacement:
			std::getline(data_ss, temp_st);		// Discard field "Vector3D" of via points 
			if(!is_goal_controller)
			{
				out_xml_element->SetAttribute("Vector3DGoal", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str());
			}
			break;
		case rxController::eControlType_Orientation:
			std::getline(data_ss, temp_st);	
			if(!is_goal_controller)
			{
				out_xml_element->SetAttribute("RGoal", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str());
			}
			break;
		case rxController::eControlType_HTransform:
			std::getline(data_ss, temp_st);	
			if(!is_goal_controller)
			{
				out_xml_element->SetAttribute("RGoal", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str());
			}
			std::getline(data_ss, temp_st);	
			if(!is_goal_controller)
			{
				out_xml_element->SetAttribute("rGoal", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str());
			}
			break;
		default:
			break;
		}
	}

	string_type alpha_string(_T("alpha"));
	string_type beta_string(_T("beta"));
	std::getline(data_ss, temp_st);	
	switch(type_of_controller.first)
	{
	case rxController::eControlType_Displacement:
		if(temp_st.compare(0, 5, alpha_string)==0)
		{
			out_xml_element->SetAttribute("alpha", colon2space( XMLDeserializer::wstring2string( temp_st.substr( temp_st.find(L"=") + 1, temp_st.find(L"\n") ) ) ).c_str() );
			std::getline(data_ss, temp_st);	
			out_xml_element->SetAttribute("alphaDisplacement", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
			std::getline(data_ss, temp_st);	
		}		
		if(temp_st.compare( 0, 4, beta_string)==0)
		{
			out_xml_element->SetAttribute("beta", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
			std::getline(data_ss, temp_st);	
			out_xml_element->SetAttribute("betaDisplacement",colon2space( XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
			std::getline(data_ss, temp_st);	
		}
		break;
	case rxController::eControlType_Orientation:		
		if(temp_st.compare(0, 5, alpha_string)==0)
		{
			out_xml_element->SetAttribute("alpha", colon2space( XMLDeserializer::wstring2string( temp_st.substr( temp_st.find(L"=") + 1, temp_st.find(L"\n") ) ) ).c_str() );
			std::getline(data_ss, temp_st);	
			out_xml_element->SetAttribute("alphaRotation", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
			std::getline(data_ss, temp_st);	
		}		
		if(temp_st.compare( 0, 4, beta_string)==0)
		{
			out_xml_element->SetAttribute("beta", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
			std::getline(data_ss, temp_st);	
			out_xml_element->SetAttribute("betaRotation",colon2space( XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
			std::getline(data_ss, temp_st);	
		}
		break;
	case rxController::eControlType_HTransform:
		if(temp_st.compare(0, 5, alpha_string)==0)
		{
			out_xml_element->SetAttribute("alpha", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
			std::getline(data_ss, temp_st);	
			out_xml_element->SetAttribute("alphaRotation", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
			std::getline(data_ss, temp_st);	
			out_xml_element->SetAttribute("alphaDisplacement", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
			std::getline(data_ss, temp_st);	
		}		
		if(temp_st.compare( 0, 4, beta_string)==0)
		{
			out_xml_element->SetAttribute("beta", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
			std::getline(data_ss, temp_st);	
			out_xml_element->SetAttribute("betaRotation", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
			std::getline(data_ss, temp_st);	
			out_xml_element->SetAttribute("betaDisplacement", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
			std::getline(data_ss, temp_st);	
		}
		break;
	default:
		break;
	}

	// We put the WITH_COMPLIANCE out of the second switch in case some of the special controllers can have also compliance
	// NOTE: HACK -> The compliance values are set manually!!!!!
	if(type_of_controller.second & WITH_COMPLIANCE)
	{
		std::string stiffness_b;
		std::string stiffness_k;
		for(int i = 0; i<dimension_int; i++)
		{
			stiffness_b += std::string("15");
			stiffness_k += std::string("100");
			if(i!=dimension_int -1)
			{
				stiffness_b += std::string(" ");
				stiffness_k += std::string(" ");
			}
		}
		out_xml_element->SetAttribute("stiffness_b", stiffness_b.c_str() );
		out_xml_element->SetAttribute("stiffness_k", stiffness_k.c_str() );
	}


	switch(type_of_controller.second)
	{
	case(OBSTACLE_AVOIDANCE):
		{
			if(temp_st.compare(0, 5, alpha_string)==0)
			{
				out_xml_element->SetAttribute("alpha", colon2space( XMLDeserializer::wstring2string( temp_st.substr( temp_st.find(L"=") + 1, temp_st.find(L"\n") ) ) ).c_str() );
				std::getline(data_ss, temp_st);	
				out_xml_element->SetAttribute("alphaDisplacement", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
				std::getline(data_ss, temp_st);	
			}	
			out_xml_element->SetAttribute("distanceThreshold", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str() );
			std::getline(data_ss, temp_st);	
			out_xml_element->SetAttribute("deactivationThreshold", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str() );		
		}
		break;
    case(SINGULARITY_AVOIDANCE):
        {
			out_xml_element->SetAttribute("maxVel", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str() );
			std::getline(data_ss, temp_st);	
        }
        break;
     case(JOINT_LIMIT_AVOIDANCE):
        {
			out_xml_element->SetAttribute("index", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str() );
			std::getline(data_ss, temp_st);	
            out_xml_element->SetAttribute("safetyThresh", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str() );		

        }
        break;
	case(SUBDISPLACEMENT | WITH_IMPEDANCE):
		{
			if(temp_st.compare(0, 5, alpha_string)==0)
			{
				out_xml_element->SetAttribute("alpha", colon2space( XMLDeserializer::wstring2string( temp_st.substr( temp_st.find(L"=") + 1, temp_st.find(L"\n") ) ) ).c_str() );
				std::getline(data_ss, temp_st);	
				out_xml_element->SetAttribute("alphaDisplacement", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
				std::getline(data_ss, temp_st);	
			}		
			if(temp_st.compare( 0, 4, beta_string)==0)
			{
				out_xml_element->SetAttribute("beta", colon2space(XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
				std::getline(data_ss, temp_st);	
				out_xml_element->SetAttribute("betaDisplacement",colon2space( XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")))).c_str() );
				std::getline(data_ss, temp_st);	
			}
			out_xml_element->SetAttribute("limitBody", colon2space( XMLDeserializer::wstring2string( temp_st.substr( temp_st.find(L"=") + 1, temp_st.find(L"\n") ) ) ).c_str() );
			string_type index_st;
			string_type tc_st;
			for(int i=0; i<dimension_int ; i++)
			{
				std::getline(data_ss, temp_st);	
				index_st += temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"));
				if(i!=dimension_int-1)
					index_st += L" ";

				std::getline(data_ss, temp_st);	
				tc_st += temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"));
				if(i!=dimension_int-1)
					tc_st += L" ";
			}
			out_xml_element->SetAttribute("index",XMLDeserializer::wstring2string(index_st).c_str());
			out_xml_element->SetAttribute("taskConstraints",XMLDeserializer::wstring2string(tc_st).c_str());
			std::getline(data_ss, temp_st);	
			out_xml_element->SetAttribute("distanceLimit", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str() );
			std::getline(data_ss, temp_st);	
			out_xml_element->SetAttribute("maxForce", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str() );
		}
		break;
	case(WITH_IMPEDANCE | ATTRACTOR):
		{
			out_xml_element->SetAttribute("desiredDistance", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str() );		
			std::getline(data_ss, temp_st);	
			out_xml_element->SetAttribute("maxForce", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str() );		
		}
		break;
	default:
		{
			
		}
		break;
	}
}

void MotionBehaviour::print()
{
	std::list<rxController*> controllers = control_set_->getControllers();
	string_type controllers_to_string;
	for(std::list< rxController* >::const_iterator controllers_it = controllers.begin();
		controllers_it != controllers.end() ; controllers_it ++){
			string_type controller_to_string;
			(*controllers_it)->toString(controller_to_string);

			std::wcout << controller_to_string << std::endl;

			//		int num_via_points_int = -1;
			//		std::wstringstream num_via_points_ss(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n")));
			//		num_via_points_ss >> num_via_points_int;
			//		for(int i=0; i< num_via_points_int; i++)
			//		{
			//			std::getline(data_ss, temp_st);		// Discard field "time" of via points
			//			std::getline(data_ss, temp_st);		// Discard field "type" of via points
			//			std::getline(data_ss, temp_st);		// Discard field "reuse" of via points 
			//			switch( type_of_controller.first ){
			//case JOINT_SPACE_CONTROLLER:
			//	std::getline(data_ss, temp_st);		// Discard field "dVector" of via points 
			//	break;
			//case rxController::eControlType_Displacement:
			//	std::getline(data_ss, temp_st);		// Discard field "Vector3D" of via points 
			//	break;
			//case rxController::eControlType_Orientation:
			//	std::getline(data_ss, temp_st);	
			//	//via_point_xml->SetAttribute("R", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
			//	break;
			//case rxController::eControlType_HTransform:
			//	std::getline(data_ss, temp_st);	
			//	//via_point_xml->SetAttribute("R", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
			//	std::getline(data_ss, temp_st);	
			//	//via_point_xml->SetAttribute("r", XMLDeserializer::wstring2string(temp_st.substr(temp_st.find(L"=") + 1, temp_st.find(L"\n"))).c_str());
			//	break;
			//default:
			//	break;
			//			}
			//		}

	}
}
