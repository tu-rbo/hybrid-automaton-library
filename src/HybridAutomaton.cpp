#include "hybrid_automaton/HybridAutomaton.h"

#include "hybrid_automaton/DescriptionTreeNode.h"

#include <sstream>

namespace ha {

	void HybridAutomaton::registerController(const std::string& ctrl_type, ControllerCreator cc) {
		std::map<std::string, HybridAutomaton::ControllerCreator>& controller_type_map = getControllerTypeMap();
		assert ( controller_type_map.find(ctrl_type) == controller_type_map.end() );
		controller_type_map[ctrl_type] = cc;
	}

	bool HybridAutomaton::isControllerRegistered(const std::string& ctrl_type) {
		std::map<std::string, HybridAutomaton::ControllerCreator>& controller_type_map = getControllerTypeMap();
		return ( controller_type_map.find(ctrl_type) != controller_type_map.end() );
	}

	void HybridAutomaton::unregisterController(const std::string& ctrl_type) {
		std::map<std::string, HybridAutomaton::ControllerCreator>& controller_type_map = getControllerTypeMap();
		std::map<std::string, HybridAutomaton::ControllerCreator>::iterator it = controller_type_map.find(ctrl_type);
		if (it == controller_type_map.end()) return;
		controller_type_map.erase(it);
	}

	Controller::Ptr HybridAutomaton::createController(const DescriptionTreeNode::ConstPtr& node, const System::ConstPtr& system) {
		if (node->getType() != "Controller") {
			std::stringstream ss;
			ss << "[HybridAutomaton::createController] DescriptionTreeNode must have type 'Controller', not '" << node->getType() << "'!";
			throw ss.str();
		}

		std::string ctrl_type;
		if (!node->getAttribute<std::string>("type", ctrl_type)) {
			throw "[HybridAutomaton::createController] Cannot get controller type from node";
		}

		std::map<std::string, HybridAutomaton::ControllerCreator>& controller_type_map = getControllerTypeMap();
		std::map<std::string, HybridAutomaton::ControllerCreator>::iterator it = controller_type_map.find(ctrl_type);
		if ( !isControllerRegistered(ctrl_type) ) {
			std::stringstream ss;
			ss << "[HybridAutomaton::createController] Controller type not registered: " << ctrl_type;
			throw ss.str();
		}
		return (*(it->second))(node, system);
	}

	void HybridAutomaton::registerControlSet(const std::string& ctrl_type, ControlSetCreator cc) {
		std::map<std::string, HybridAutomaton::ControlSetCreator>& controlset_type_map = getControlSetTypeMap();
		assert ( controlset_type_map.find(ctrl_type) == controlset_type_map.end() );
		controlset_type_map[ctrl_type] = cc;
	}

	bool HybridAutomaton::isControlSetRegistered(const std::string& ctrl_type) {
		std::map<std::string, HybridAutomaton::ControlSetCreator>& controlset_type_map = getControlSetTypeMap();
		return ( controlset_type_map.find(ctrl_type) != controlset_type_map.end() );
	}

	ControlSet::Ptr HybridAutomaton::createControlSet(const DescriptionTreeNode::ConstPtr& node, const System::ConstPtr& system) {
		if (node->getType() != "ControlSet") {
			std::stringstream ss;
			ss << "[HybridAutomaton::createControlSet] DescriptionTreeNode must have type 'ControlSet', not '" << node->getType() << "'!";
			throw ss.str();
		}

		std::string ctrl_type;
		if (!node->getAttribute<std::string>("type", ctrl_type)) {
			throw "[HybridAutomaton::createControlSet] Cannot get controller type from node";
		}

		std::map<std::string, HybridAutomaton::ControlSetCreator>& controlset_type_map = getControlSetTypeMap();
		std::map<std::string, HybridAutomaton::ControlSetCreator>::iterator it = controlset_type_map.find(ctrl_type);
		if ( !isControlSetRegistered(ctrl_type) ) {
			std::stringstream ss;
			ss << "[HybridAutomaton::createControlSet] ControlSet type not registered: " << ctrl_type;
			throw ss.str();
		}
		return (*(it->second))(node, system);
	}

	void HybridAutomaton::unregisterControlSet(const std::string& ctrl_type) {
		std::map<std::string, HybridAutomaton::ControlSetCreator>& controlset_type_map = getControlSetTypeMap();
		std::map<std::string, HybridAutomaton::ControlSetCreator>::iterator it = controlset_type_map.find(ctrl_type);
		if (it == controlset_type_map.end()) return;
		controlset_type_map.erase(it);
	}


	void HybridAutomaton::addControlMode(const ControlMode::Ptr& control_mode) {
		boost::add_vertex(control_mode->getName(), control_mode, _graph);
	}

	void HybridAutomaton::addControlSwitch(const std::string& source_mode, const ControlSwitch::Ptr& control_switch, const std::string& target_mode) {
		boost::add_edge_by_label(source_mode, target_mode, control_switch, _graph);
	}

	void HybridAutomaton::addControlSwitchAndMode(const std::string& source_mode, const ControlSwitch::Ptr& control_switch, const ControlMode::Ptr& target_mode) {
		boost::add_vertex(target_mode->getName(), target_mode, _graph);
		boost::add_edge_by_label(source_mode, target_mode->getName(), control_switch, _graph);
	}
	
	::Eigen::MatrixXd HybridAutomaton::step(const double& t) {
		if (_active)
		{
			// check if any out-going jump condition is true
			::std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = ::boost::out_edges(_graph.vertex(_current_control_mode->getName()), _graph);
			for(; out_edges.first != out_edges.second; ++out_edges.first) {
				SwitchHandle switch_handle = *out_edges.first;
				ControlSwitch::Ptr control_switch = _graph[switch_handle];

				control_switch->step(t);

				if (control_switch->isActive())
				{
					// switch to the next control mode
					ModeHandle mode_handle = boost::target(switch_handle, _graph);

					_current_control_mode->deactivate();
					_current_control_mode = _graph.graph()[mode_handle];
					
					_activateCurrentControlMode(t);

					// CE: We still need to encapsulate this functionality:
					//		if (!_activeMotionBehaviour->replaceControllers((MotionBehaviour*) edges[0]))
					//		{
					//			_activeMotionBehaviour = (MotionBehaviour*) edges[0];
					//		}
					break;
				}
			}
			return _current_control_mode->step(t); 
		}
		throw std::string("[HybridAutomaton::step] No current control mode defined.");
	}


	DescriptionTreeNode::Ptr HybridAutomaton::serialize(const DescriptionTree::ConstPtr& factory) const {
		DescriptionTreeNode::Ptr tree_node = factory->createNode("HybridAutomaton");
		tree_node->setAttribute<std::string>(std::string("name"), this->getName());

		if (_current_control_mode)
			tree_node->setAttribute<std::string>(std::string("current_control_mode"), _current_control_mode->getName());

		// Iterate over the vertices and serialize them
		::std::pair<ModeIterator, ModeIterator> v_pair;
		for(v_pair = ::boost::vertices(this->_graph); v_pair.first != v_pair.second; ++v_pair.first)
		{
			tree_node->addChildNode(_graph.graph()[*v_pair.first]->serialize(factory));

			for(::std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = ::boost::out_edges(_graph.vertex(_graph.graph()[*v_pair.first]->getName()), _graph); out_edges.first != out_edges.second; ++out_edges.first) 
			{
				tree_node->addChildNode(_graph.graph()[*out_edges.first]->serialize(factory));
			}
		}

		return tree_node;
	}

	void HybridAutomaton::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system){
		if (tree->getType() != "HybridAutomaton") {
			std::stringstream ss;
			ss << "[HybridAutomaton::deserialize] DescriptionTreeNode must have type 'HybridAutomaton', not '" << tree->getType() << "'!";
			throw ss.str();
		}

		tree->getAttribute<std::string>("name", _name);
 
		// control modes
		DescriptionTreeNode::ConstNodeList control_modes;
		tree->getChildrenNodes("ControlMode", control_modes);

		if (control_modes.empty()) {
			throw "[HybridAutomaton::deserialize] No control modes found!";
		}

		DescriptionTreeNode::ConstNodeList::iterator cm_it;
		for (cm_it = control_modes.begin(); cm_it != control_modes.end(); ++cm_it) {
			ControlMode::Ptr cm(new ControlMode);
			//cm->setHybridAutomaton((const HybridAutomaton*) this);
			cm->setHybridAutomaton(this);
			cm->deserialize(*cm_it, system);
			this->addControlMode(cm);
		}

		// control switches
		DescriptionTreeNode::ConstNodeList control_switches;
		tree->getChildrenNodes("ControlSwitch", control_switches);
		DescriptionTreeNode::ConstNodeList::iterator cs_it;
		for (cs_it = control_switches.begin(); cs_it != control_switches.end(); ++cs_it) {
			ControlSwitch::Ptr cs(new ControlSwitch);
			cs->deserialize(*cs_it, system);
			
			// check if source and target are in graph
			if (!existsControlMode(cs->getSourceControlMode()))
				throw std::string("[HybridAutomaton::deserialize] ERROR: Control mode '") + cs->getSourceControlMode() + "' does not exist! Cannot set source control mode.";	
			if (!existsControlMode(cs->getTargetControlMode()))
				throw std::string("[HybridAutomaton::deserialize] ERROR: Control mode '") + cs->getTargetControlMode() + "' does not exist! Cannot set target control mode.";	

			this->addControlSwitch(cs->getSourceControlMode(),  
				cs, cs->getTargetControlMode());
		}
	}

	void HybridAutomaton::setName(const std::string& name) {
		_name = name;
	}

	const std::string HybridAutomaton::getName() const {
		return _name;
	}

	void HybridAutomaton::activate(const double& t) {
		if (!_current_control_mode) {
			throw std::string("ERROR: No current control mode defined!");
		}
		_active = true;

		_activateCurrentControlMode(t);
	}

	void HybridAutomaton::deactivate() {
		if (_current_control_mode)
			_current_control_mode->deactivate();
		_active = false;

		// deactivate all outgoing edges
		::std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = ::boost::out_edges(_graph.vertex(_current_control_mode->getName()), _graph);
		for(; out_edges.first != out_edges.second; ++out_edges.first) {
			_graph[*out_edges.first]->deactivate();
		}
	}

	void HybridAutomaton::setCurrentControlMode(const std::string& control_mode) {
		/* useful code
		typedef Graph::vertex_iterator VertexIter; 
		VertexIter vertexIter, vertexEnd; 
		for (boost::tie(vertexIter, vertexEnd) = boost::vertices(_graph); vertexIter != vertexEnd; vertexIter++) 
		{ 
		std::cout << "Name " << _graph.graph()[*vertexIter]->getName() << std::endl;
		} 
		*/

		//if (!existsControlMode(control_mode))
		if (::boost::vertex_by_label(control_mode, _graph) == GraphTraits::null_vertex())
			throw std::string("[HybridAutomaton::setCurrentControlMode] ERROR: Control mode '") + control_mode + "' does not exist! Cannot set current control mode.";

		if (_current_control_mode != NULL)
			_current_control_mode->deactivate();
		_current_control_mode = _graph[control_mode];

		_activateCurrentControlMode(0.0);
	}

	ControlMode::Ptr HybridAutomaton::getCurrentControlMode() const {
		return _current_control_mode;
	}

	void HybridAutomaton::_activateCurrentControlMode(const double& t) {
		_current_control_mode->activate();

		// activate all outgoing edges
		::std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = ::boost::out_edges(_graph.vertex(_current_control_mode->getName()), _graph);
		for(; out_edges.first != out_edges.second; ++out_edges.first) {
			_graph[*out_edges.first]->activate(t);
		}
	}

	bool HybridAutomaton::existsControlMode(const std::string& control_mode)  {
		// FIXME cannot make this method const because of vertex_by_label
		return !(::boost::vertex_by_label(control_mode, _graph) == GraphTraits::null_vertex());
	}

}
