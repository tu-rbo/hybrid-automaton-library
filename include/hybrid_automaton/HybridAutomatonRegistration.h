/**
 * @brief Macro for registering controllers with HybridAutomaton
 *
 * Required to enable deserialization of this controller.
 * Invoke this macro in the header of your controller.
 *
 * The macro will create (amongst other things) a factory method which is passed
 * a ha::DescriptionTreeNode::Ptr and a ha::System::Ptr.
 * You are free to choose the name of these parameters and pass
 * them as arguments for the macro.
 *
 * Attention: Be sure to make this factory method public!
 *
 * Inside the factory method you should create a new instance of your controller
 * and pass to it the system and deserialize it from the
 * passed node (both things are in principle optional, but the standard
 * use case).
 *
 * Example:
 * @code
 *   public:
 *   HA_CONTROLLER_INSTANCE(node, system, ha) {
 *		YourController::Ptr mc(new YourController);
 *		mc->setSystem(system);
 *		mc->deserialize(node, system, ha);
 *		return mc;
 *   }
 * @endcode
 *
 * Why do we do it like this? It could be that YourController requires
 * special arguments in the constructor, which you can pass here.
 *
 * Internally, this factory method will be called by HybridAutomaton when
 * deserializing a HybridAutomaton.
 *
 * @see HA_CONTROLLER_REGISTER
 */
#define HA_CONTROLLER_INSTANCE(NODE_NAME, SYSTEM_NAME, HA_NAME) \
	class Initializer {\
	public:\
	Initializer();\
	};\
	static Initializer initializer;\
	virtual const std::string getType() const;\
	static std::string type();\
	static ::ha::Controller::Ptr instance(const ::ha::DescriptionTreeNode::ConstPtr NODE_NAME, const ::ha::System::ConstPtr SYSTEM_NAME, const ::ha::HybridAutomaton* HA_NAME)

/**
 * @brief Registration method for registering your controller with the HybridAutomaton.
 *
 * The first parameter specifies the name that will be used
 * in the serialization, the second is the class name
 * of your controller.
 *
 * Invoke this in the cpp file of your controller!
 *
 * Example:
 * @code
 *   HA_CONTROLLER_REGISTER("JointController", rlabJointController)
 * @endcode
 */
#define HA_CONTROLLER_REGISTER(STR, NAME) NAME::Initializer NAME::initializer;\
	NAME::Initializer::Initializer() { ::ha::HybridAutomaton::registerController(STR, &NAME::instance); }\
	const std::string NAME::getType() const { return STR; }\
	std::string NAME::type() { return STR; }


/**
 * @brief Macro for registering control sets with HybridAutomaton
 *
 * Required to enable deserialization of this control set.
 * Invoke this in the header of your control set.
 *
 * Attention: Be sure to make this factory method public!
 *
 * It will create (amongst other things) a factory method which is passed
 * a ha::DescriptionTreeNode::Ptr and a ha::System::Ptr.
 * You are free to choose the name of these parameters and pass
 * them as arguments for the macro.
 *
 * Inside the factory method you should create a new instance of your control set
 * and pass to it the system and deserialize it from the
 * passed node (both things are in principle optional, but the standard
 * use case).
 *
 * Example:
 * @code
 *   public:
 *   HA_CONTROLSET_INSTANCE(node, system, ha) {
 *		YourControlSet::Ptr mc(new YourControlSet);
 *		mc->setSystem(system);
 *		mc->deserialize(node, system, ha);
 *		return mc;
 *   }
 * @endcode
 *
 * Why do we do it like this? It could be that YourControlSet requires
 * special arguments in the constructor, which you can pass here.
 *
 * Internally, this factory method will be called by HybridAutomaton when
 * deserializing a HybridAutomaton.
 */
#define HA_CONTROLSET_INSTANCE(NODE_NAME, SYSTEM_NAME, HA_NAME) \
	class Initializer {\
	public:\
	Initializer();\
	static void ping() {};\
	};\
	static Initializer initializer;\
	virtual const std::string getType() const;\
	static std::string type();\
	static ::ha::ControlSet::Ptr instance(const ::ha::DescriptionTreeNode::ConstPtr NODE_NAME, const ::ha::System::ConstPtr SYSTEM_NAME, const ::ha::HybridAutomaton* HA_NAME)

/**
 * @brief Registration method for registering your control set with the HybridAutomaton.
 *
 * The first parameter specifies the name that will be used
 * in the serialization, the second is the class name
 * of your control set.
 *
 * Invoke this in the cpp file of your control set.
 *
 * Example:
 * @code
 *   HA_CONTROLSET_REGISTER("MyControlSet", rlabMyControlSet)
 * @endcode
 */
#define HA_CONTROLSET_REGISTER(STR, NAME) NAME::Initializer NAME::initializer;\
	NAME::Initializer::Initializer() { ha::HybridAutomaton::registerControlSet(STR, &NAME::instance); }\
	const std::string NAME::getType() const { return STR; }\
	std::string NAME::type() { return STR; }


/**
 * @brief Macro for registering sensors with HybridAutomaton
 *
 * Invoke this in the header file of your sensor.
 *
 */
#define HA_SENSOR_INSTANCE(NODE_NAME, SYSTEM_NAME, HA_NAME) \
	class Initializer {\
	public:\
	Initializer();\
	static void ping() {};\
	};\
	static Initializer initializer;\
	virtual const std::string getType() const;\
	static std::string type();\
	static Sensor::Ptr instance(const ::ha::DescriptionTreeNode::ConstPtr NODE_NAME, const ::ha::System::ConstPtr SYSTEM_NAME, const ::ha::HybridAutomaton* HA_NAME)

/**
 * @brief Macro for registering sensors with HybridAutomaton
 *
 * Invoke this in the cpp file of your sensor.
 *
 */
#define HA_SENSOR_REGISTER(STR, NAME) NAME::Initializer NAME::initializer;\
	NAME::Initializer::Initializer() { ha::HybridAutomaton::registerSensor(STR, &NAME::instance); }\
	const std::string NAME::getType() const { return STR; }\
	std::string NAME::type() { return STR; }