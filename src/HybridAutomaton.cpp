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

	Controller::Ptr HybridAutomaton::createController(DescriptionTreeNode::Ptr node, System::Ptr system) {
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

	ControlSet::Ptr HybridAutomaton::createControlSet(DescriptionTreeNode::Ptr node, System::Ptr system) {
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

				if (control_switch->isActive())
				{
					// switch to the next control mode
					ModeHandle mode_handle = boost::target(switch_handle, _graph);

					_current_control_mode->deactivate();
					_current_control_mode = _graph.graph()[mode_handle];
					_current_control_mode->activate();

					// CE: We still need to encapsulate this functionality:
					//		if (!_activeMotionBehaviour->replaceControllers((MotionBehaviour*) edges[0]))
					//		{
					//			_activeMotionBehaviour = (MotionBehaviour*) edges[0];
					//		}
					break;
				}
			}
		}

		return _current_control_mode->step(t); 
	}

	DescriptionTreeNode::Ptr HybridAutomaton::serialize(const DescriptionTree::ConstPtr& factory) const {
		throw std::string("not implemented");
	}

	void HybridAutomaton::deserialize(const DescriptionTreeNode::ConstPtr& tree){
		//tree.getAttribute(std::string("name"));
		//name = tree.getAttribute("name");
	}

	void HybridAutomaton::setName(const std::string& name) {
		_name = name;
	}

	const std::string& HybridAutomaton::getName() const {
		return _name;
	}

	void HybridAutomaton::activate() {
		if (!_current_control_mode) {
			throw std::string("ERROR: No current control mode defined!");
		}
		_active = true;
		_current_control_mode->activate();
	}

	void HybridAutomaton::deactivate() {
		if (_current_control_mode)
			_current_control_mode->deactivate();
		_active = false;
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

		if (::boost::vertex_by_label(control_mode, _graph) == GraphTraits::null_vertex())
			throw std::string("ERROR: Control mode '") + control_mode + "' does not exist! Cannot set current control mode.";

		if (_current_control_mode != NULL)
			_current_control_mode->deactivate();
		_current_control_mode = _graph[control_mode];
		_current_control_mode->activate();
	}

	ControlMode::Ptr HybridAutomaton::getCurrentControlMode() const {
		return _current_control_mode;
	}

}
