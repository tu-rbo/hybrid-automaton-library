#include "hybrid_automaton/HybridAutomaton.h"

#include "hybrid_automaton/DescriptionTreeNode.h"
#include "hybrid_automaton/error_handling.h"

#include <boost/graph/graphviz.hpp>

#include <sstream>

namespace ha {

	HybridAutomaton::HybridAutomaton()
		: _active(false)
	{
	}

	HybridAutomaton::~HybridAutomaton()
	{
	}

	void HybridAutomaton::registerController(const std::string& ctrl_type, ControllerCreator cc) 
	{
		std::map<std::string, HybridAutomaton::ControllerCreator>& controller_type_map = getControllerTypeMap();
		assert ( controller_type_map.find(ctrl_type) == controller_type_map.end() );
		controller_type_map[ctrl_type] = cc;
	}

	bool HybridAutomaton::isControllerRegistered(const std::string& ctrl_type) 
	{
		std::map<std::string, HybridAutomaton::ControllerCreator>& controller_type_map = getControllerTypeMap();
		return ( controller_type_map.find(ctrl_type) != controller_type_map.end() );
	}

	void HybridAutomaton::unregisterController(const std::string& ctrl_type) 
	{
		std::map<std::string, HybridAutomaton::ControllerCreator>& controller_type_map = getControllerTypeMap();
		std::map<std::string, HybridAutomaton::ControllerCreator>::iterator it = controller_type_map.find(ctrl_type);
		if (it == controller_type_map.end()) return;
		controller_type_map.erase(it);
	}

	Controller::Ptr HybridAutomaton::createController(const DescriptionTreeNode::ConstPtr& node, const System::ConstPtr& system, const HybridAutomaton* ha) 
	{
		if (node->getType() != "Controller") {
			HA_THROW_ERROR("HybridAutomaton.createController", "DescriptionTreeNode must have type 'Controller', not '" << node->getType() << "'!");
		}

		std::string ctrl_type;
		if (!node->getAttribute<std::string>("type", ctrl_type)) {
			HA_THROW_ERROR("HybridAutomaton.createController", "Cannot get controller type from node");
		}

		std::map<std::string, HybridAutomaton::ControllerCreator>& controller_type_map = getControllerTypeMap();
		std::map<std::string, HybridAutomaton::ControllerCreator>::iterator it = controller_type_map.find(ctrl_type);
		if ( !isControllerRegistered(ctrl_type) ) {
			HA_THROW_ERROR("HybridAutomaton.createController", "Controller type not registered: " << ctrl_type);
		}
		return (*(it->second))(node, system, ha);
	}

	void HybridAutomaton::registerControlSet(const std::string& ctrl_type, ControlSetCreator cc) 
	{
		std::map<std::string, HybridAutomaton::ControlSetCreator>& controlset_type_map = getControlSetTypeMap();
		assert ( controlset_type_map.find(ctrl_type) == controlset_type_map.end() );
		controlset_type_map[ctrl_type] = cc;
	}

	bool HybridAutomaton::isControlSetRegistered(const std::string& ctrl_type) 
	{
		std::map<std::string, HybridAutomaton::ControlSetCreator>& controlset_type_map = getControlSetTypeMap();
		return ( controlset_type_map.find(ctrl_type) != controlset_type_map.end() );
	}

	ControlSet::Ptr HybridAutomaton::createControlSet(const DescriptionTreeNode::ConstPtr& node, const System::ConstPtr& system, const HybridAutomaton* ha) 
	{
		if (node->getType() != "ControlSet") {
			HA_THROW_ERROR("HybridAutomaton.createControlSet", "DescriptionTreeNode must have type 'ControlSet', not '" << node->getType() << "'!");
		}

		std::string ctrl_type;
		if (!node->getAttribute<std::string>("type", ctrl_type)) {
			HA_THROW_ERROR("HybridAutomaton.createControlSet", "Cannot get controller type from node");
		}

		std::map<std::string, HybridAutomaton::ControlSetCreator>& controlset_type_map = getControlSetTypeMap();
		std::map<std::string, HybridAutomaton::ControlSetCreator>::iterator it = controlset_type_map.find(ctrl_type);
		if ( !isControlSetRegistered(ctrl_type) ) {
			HA_THROW_ERROR("HybridAutomaton.createControlSet", "ControlSet type not registered: " << ctrl_type);
		}
		return (*(it->second))(node, system, ha);
	}

	void HybridAutomaton::unregisterControlSet(const std::string& ctrl_type) {
		std::map<std::string, HybridAutomaton::ControlSetCreator>& controlset_type_map = getControlSetTypeMap();
		std::map<std::string, HybridAutomaton::ControlSetCreator>::iterator it = controlset_type_map.find(ctrl_type);
		if (it == controlset_type_map.end()) return;
		controlset_type_map.erase(it);
	}

	void HybridAutomaton::registerSensor(const std::string& sensor_type, SensorCreator sc)
	{
		std::map<std::string, HybridAutomaton::SensorCreator>& sensor_type_map = getSensorTypeMap();
		assert ( sensor_type_map.find(sensor_type) == sensor_type_map.end() );
		sensor_type_map[sensor_type] = sc;
	}

	bool HybridAutomaton::isSensorRegistered(const std::string& sensor_type)
	{
		std::map<std::string, HybridAutomaton::SensorCreator>& sensor_type_map = getSensorTypeMap();
		return ( sensor_type_map.find(sensor_type) != sensor_type_map.end() );
	}

	void HybridAutomaton::unregisterSensor(const std::string& sensor_type)
	{
		std::map<std::string, HybridAutomaton::SensorCreator>& sensor_type_map = getSensorTypeMap();
		std::map<std::string, HybridAutomaton::SensorCreator>::iterator it = sensor_type_map.find(sensor_type);
		if (it == sensor_type_map.end()) return;
		sensor_type_map.erase(it);
	}

	Sensor::Ptr HybridAutomaton::createSensor(const DescriptionTreeNode::ConstPtr& node, const System::ConstPtr& system, const HybridAutomaton* ha) 
	{
		if (node->getType() != "Sensor") {
			HA_THROW_ERROR("HybridAutomaton.createSensor", "DescriptionTreeNode must have type 'Sensor', not '" << node->getType() << "'!");
		}

		std::string sensor_type;
		if (!node->getAttribute<std::string>("type", sensor_type)) {
			HA_THROW_ERROR("HybridAutomaton.createSensor", "Cannot get sensor type from node");
		}

		std::map<std::string, HybridAutomaton::SensorCreator>& sensor_type_map = getSensorTypeMap();
		std::map<std::string, HybridAutomaton::SensorCreator>::iterator it = sensor_type_map.find(sensor_type);
		if ( !isSensorRegistered(sensor_type) ) {
			HA_THROW_ERROR("HybridAutomaton.createSensor", "Sensor type not registered: " << sensor_type);
		}
		return (*(it->second))(node, system, ha);
	}


    void HybridAutomaton::addControlMode(const ControlMode::Ptr& control_mode)
	{
        AdjacencyListGraph& g = _graph.graph();
        ModeHandle mh = boost::add_vertex(control_mode->getName(), control_mode, _graph);
        g[mh] = control_mode;
	}

	void HybridAutomaton::addControlSwitch(const std::string& source_mode, const ControlSwitch::Ptr& control_switch, const std::string& target_mode) 
	{
        control_switch->setHybridAutomaton(this);

        AdjacencyListGraph& g = _graph.graph();
        SwitchHandle sh = boost::add_edge_by_label(source_mode, target_mode, control_switch, _graph).first;
        g[sh] = control_switch;

		_switchMap.insert(std::pair<std::string, SwitchHandle>(control_switch->getName(), sh));
	}

	void HybridAutomaton::addControlSwitchAndMode(const std::string& source_mode, const ControlSwitch::Ptr& control_switch, const ControlMode::Ptr& target_mode) 
	{
		addControlMode(target_mode);
		addControlSwitch(source_mode, control_switch, target_mode->getName());
		//boost::add_vertex(target_mode->getName(), target_mode, _graph);
		//boost::add_edge_by_label(source_mode, target_mode->getName(), control_switch, _graph);
	}

	::Eigen::MatrixXd HybridAutomaton::step(const double& t) 
	{
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

					_current_control_mode->terminate();
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
		HA_THROW_ERROR("HybridAutomaton.step", "No current control mode defined.");
	}


	DescriptionTreeNode::Ptr HybridAutomaton::serialize(const DescriptionTree::ConstPtr& factory) const 
	{
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

	void HybridAutomaton::deserialize(const DescriptionTreeNode::ConstPtr& tree, const System::ConstPtr& system, const HybridAutomaton* ha)
	{
		if (tree->getType() != "HybridAutomaton") {
			HA_THROW_ERROR("HybridAutomaton.deserialize", "DescriptionTreeNode must have type 'HybridAutomaton', not '" << tree->getType() << "'!");
		}

		tree->getAttribute<std::string>("name", _name);

		// control modes
		DescriptionTreeNode::ConstNodeList control_modes;
		tree->getChildrenNodes("ControlMode", control_modes);

		if (control_modes.empty()) 
		{
			HA_THROW_ERROR("HybridAutomaton.deserialize", "No control modes found!");
		}

		DescriptionTreeNode::ConstNodeList::iterator cm_it;
		for (cm_it = control_modes.begin(); cm_it != control_modes.end(); ++cm_it) 
		{
			ControlMode::Ptr cm(new ControlMode);
			cm->deserialize(*cm_it, system, this);
			this->addControlMode(cm);
		}

		// control switches
		DescriptionTreeNode::ConstNodeList control_switches;
		tree->getChildrenNodes("ControlSwitch", control_switches);
		DescriptionTreeNode::ConstNodeList::iterator cs_it;
		for (cs_it = control_switches.begin(); cs_it != control_switches.end(); ++cs_it) 
		{
			ControlSwitch::Ptr cs(new ControlSwitch);
			std::string cs_name;
			(*cs_it)->getAttribute<std::string>("name", cs_name, "");
			cs->setName(cs_name);

			// check if source and target are in graph
			std::string source, target;
			(*cs_it)->getAttribute<std::string>("source", source, "");
			(*cs_it)->getAttribute<std::string>("target", target, "");

			if (!existsControlMode(source))
				HA_THROW_ERROR("HybridAutomaton.deserialize", "Control mode '" << source << "' does not exist! Cannot set source control mode.");	
			if (!existsControlMode(target))
				HA_THROW_ERROR("HybridAutomaton.deserialize", "Control mode '" << target << "' does not exist! Cannot set target control mode.");	

			this->addControlSwitch(source, cs, target);

			cs->deserialize(*cs_it, system, this);
		}
		
		std::string current_control_mode_name;
		if (tree->getAttribute<std::string>("current_control_mode", current_control_mode_name))
		{
			this->setCurrentControlMode(current_control_mode_name);
		}
		else
			HA_THROW_ERROR("HybridAutomaton.deserialize", "Control mode '" << current_control_mode_name << "' does not exist! Cannot set current control mode.");	
	}

	void HybridAutomaton::setName(const std::string& name) 
	{
		_name = name;
	}

	const std::string HybridAutomaton::getName() const 
	{
		return _name;
	}

	void HybridAutomaton::initialize(const double& t) 
	{
		if (!_current_control_mode) {
			HA_THROW_ERROR("HybridAutomaton.initialize", "No current control mode defined!");
		}
		_activateCurrentControlMode(t);
		_active = true;
	}

	void HybridAutomaton::terminate() 
	{
		if (_current_control_mode)
			_current_control_mode->terminate();
		_active = false;

		// deactivate all outgoing edges
		::std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = ::boost::out_edges(_graph.vertex(_current_control_mode->getName()), _graph);
		for(; out_edges.first != out_edges.second; ++out_edges.first) {
			_graph[*out_edges.first]->terminate();
		}
	}

	void HybridAutomaton::setCurrentControlMode(const std::string& control_mode) 
	{
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
			HA_THROW_ERROR("HybridAutomaton.setCurrentControlMode", "Control mode '" << control_mode << "' does not exist! Cannot set current control mode.");

		if (_current_control_mode != NULL)
			_current_control_mode->terminate();
		_current_control_mode = _graph[control_mode];
	}

	ControlMode::Ptr HybridAutomaton::getCurrentControlMode() const 
	{
		return _current_control_mode;
	}

	void HybridAutomaton::_activateCurrentControlMode(const double& t) 
	{
		_current_control_mode->initialize();

		// initialize all outgoing edges
		::std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = ::boost::out_edges(_graph.vertex(_current_control_mode->getName()), _graph);
		for(; out_edges.first != out_edges.second; ++out_edges.first) {
			_graph[*out_edges.first]->initialize(t);
		}
	}

	bool HybridAutomaton::existsControlMode(const std::string& control_mode) const 
	{
		return !(_graph.vertex(control_mode) == GraphTraits::null_vertex());
	}

	bool HybridAutomaton::existsControlSwitch(const std::string& control_switch) const 
	{
		return (_switchMap.find(control_switch) != _switchMap.end());
	}

	ControlMode::ConstPtr HybridAutomaton::getControlModeByName(const std::string& control_mode_name) const
	{
		if (!existsControlMode(control_mode_name))
			HA_THROW_ERROR("HybridAutomaton.getControlModeByName", "Control mode " << control_mode_name << " does not exist");

		return _graph[control_mode_name];
	}

	ControlSwitch::ConstPtr HybridAutomaton::getControlSwitchByName(const std::string& control_switch_name) const
	{
		if (!existsControlSwitch(control_switch_name))
			HA_THROW_ERROR("HybridAutomaton.getControlSwitchByName", "Control switch " << control_switch_name << " does not exist");
		
		SwitchHandle sh = _switchMap.find(control_switch_name)->second;
		return _graph.graph()[sh];
	}

	Controller::ConstPtr HybridAutomaton::getControllerByName(const std::string& control_mode_name, const std::string& controller_name) const 
	{
		if (!existsControlMode(control_mode_name))
			HA_THROW_ERROR("HybridAutomaton.getControllerByName", "Control mode " << control_mode_name << " does not exist");

		ControlMode::Ptr cm = _graph[control_mode_name];

		return cm->getControllerByName(controller_name);
	}

	ControlMode::ConstPtr HybridAutomaton::getSourceControlMode(const std::string& controlSwitch) const
	{
		SwitchHandle sh = _switchMap.find(controlSwitch)->second;
		ModeHandle mh = ::boost::source(sh, _graph);
		return _graph.graph()[mh];
	}

	ControlMode::ConstPtr HybridAutomaton::getTargetControlMode(const std::string& controlSwitch) const
	{
		SwitchHandle sh = _switchMap.find(controlSwitch)->second;
		ModeHandle mh = ::boost::target(sh, _graph);
		return _graph.graph()[mh];
	}

//    template < class Name >
//    class label_writer {
//    public:
//        //label_writer(Name _name) : name(_name) {}
//        label_writer(Name _name) {}

//        //void operator()(std::ostream& out, const VertexOrEdge& v) const {
////        template
//        void operator()(std::ostream& out, const ControlMode::Ptr& v) const {

//        out << "[label=\"" << "hort" << "\"]";
//    }
////        private:
////        Name name;
//    };

    template <class Graph>
    struct vertex_writer {
    private:
        Graph _g;
    public:
        vertex_writer(Graph& g) : _g(g) {}

        template <class V>
        void operator()(std::ostream& out, const V& v) const {
            ControlMode::Ptr cm = _g[v];
            if (!cm) {
                HA_ERROR("HybridAutomaton.visualizeGraph", "Unable to obtain ControlMode for vertex " << v << " - is your graph correct?");
                return;
            }

//            out << " [label=\"" << cm->getName() << "\"]";

            // HACK: move backwards as many steps as necessary to remove leading node id
            for (int i=0; i <= v/10; i++) out <<'\b';

            out << "subgraph " << "cluster" << v << " {" << std::endl;
            out << v << " [label=\"" << cm->getName() << "\"];" <<std::endl;
            ControlSetPtr cs = cm->getControlSet();
            const std::map<std::string, Controller::Ptr>& controllers = cs->getControllers();
            std::map<std::string, Controller::Ptr>::const_iterator it;
            for (it = controllers.begin(); it != controllers.end(); it++) {
                std::cout << it->first << ";" << std::endl;
                out << it->first << ";" << std::endl;
            }
            out << "}" << std::endl;

        }
    };

    template <class Graph>
    struct edge_writer {
    private:
        Graph _g;
    public:
        edge_writer(Graph& g) : _g(g) {}

        template <class E>
        void operator()(std::ostream& out, const E& e) const {
            ControlSwitch::Ptr cs = _g[e];
            if (!cs) {
                HA_ERROR("HybridAutomaton.visualizeGraph", "Unable to obtain ControlSwitch for vertex " << e << " - is your graph correct?");
                return;
            }
            out << " [label=\"" << cs->getName() << "\"]";
        }
    };

    template <class Graph>
    struct graph_writer {
    private:
        Graph _g;
    public:
        graph_writer(Graph& g) : _g(g) {}

        void operator()(std::ostream& out) const {
            out << "edge[style=\"dotted\"];" << std::endl;
            out << "compound=true;" << std::endl;
        }
    };

    void HybridAutomaton::visualizeGraph(const std::string& filename) {
        // write the dot file
        std::ofstream dotfile (filename.c_str ());

//        AdjacencyListGraph& g = _graph.graph();

//        boost::dynamic_properties dp;
//        dp.property("name", vertex_writer<AdjacencyListGraph>(_graph.graph()));
//        dp.property("node_id", vertex_writer<AdjacencyListGraph>(_graph.graph());

//        boost::write_graphviz_dp(dotfile, this->_graph, dp);
        boost::write_graphviz( dotfile, this->_graph,
                               vertex_writer<AdjacencyListGraph>(_graph.graph()),
                               edge_writer<AdjacencyListGraph>(_graph.graph()),
                               graph_writer<AdjacencyListGraph>(_graph.graph())
                );
    }
}
