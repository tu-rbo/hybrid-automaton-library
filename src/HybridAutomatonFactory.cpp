//#include "hybrid_automaton/HybridAutomatonFactory.h"
//
//#define DEFAULT_NUM_DOF_ARM 7
//#define DEFAULT_NUM_DOF_BASE 3
//
//#define DEFAULT_MAX_VEL_JS_ARM 0.3
//#define DEFAULT_MAX_VEL_JS_BASE 0.15
//
//#define DEFAULT_KP_JS_ARM 0.0
//#define DEFAULT_KV_JS_ARM 0.0
//#define DEFAULT_KP_JS_BASE 40.0
//#define DEFAULT_KV_JS_BASE 7.0
//
//#define DEFAULT_KP_JS_NAKAMURA_ARM 15.0
//#define DEFAULT_KP_JS_NAKAMURA_BASE 0.0
//#define DEFAULT_KV_JS_NAKAMURA_ARM 0.5
//#define DEFAULT_KV_JS_NAKAMURA_BASE 5.0
//
//#define DEFAULT_JOINT_WEIGHTS_NAKAMURA_ARM 1.0
//#define DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE 0.5
//#define DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE_LITTLE_MOTION 0.01
//
//#define DEFAULT_KP_OS_LINEAR 0.0
//#define DEFAULT_KP_OS_ANGULAR 0.0
//#define DEFAULT_KV_OS_LINEAR 10.0
//#define DEFAULT_KV_OS_ANGULAR 10.0
//
//#define DEFAULT_HOME_CONFIG_JS_ARM 0.0
//#define DEFAULT_HOME_CONFIG_JS_BASE 0.0
//
//#define DEFAULT_POS_EPSILON_JS_ARM 0.13
//#define DEFAULT_POS_EPSILON_JS_BASE 0.05
//#define DEFAULT_VEL_EPSILON_JS_ARM 0.01
//#define DEFAULT_VEL_EPSILON_JS_BASE 0.001
//
//#define DEFAULT_POS_EPSILON_OS_LINEAR 0.01
//#define DEFAULT_POS_EPSILON_OS_ANGULAR 0.01
//#define DEFAULT_VEL_EPSILON_OS_LINEAR 0.01
//#define DEFAULT_VEL_EPSILON_OS_ANGULAR 0.01
//
//namespace ha
//{
//	HybridAutomatonFactory::HybridAutomatonFactory()
//		: HybridAutomatonFactory(DEFAULT_NUM_DOF_ARM, DEFAULT_NUM_DOF_BASE)
//	{
//
//	}
//
//	HybridAutomatonFactory::HybridAutomatonFactory(const int& num_dof_arm, const int& num_dof_base)
//		:_num_dof_arm(num_dof_arm), _num_dof_base(num_dof_base)
//	{
//
//		_index_vec_arm.resize(_num_dof_arm, 0.0);
//		std::stringstream index_arm_ss;
//		index_arm_ss << "[" << _num_dof_arm << ",1]";
//		const char* separator_arm = "";
//		for(int idx_arm=0; idx_arm<_num_dof_arm; idx_arm++)
//		{
//			_index_vec_arm.push_back(idx_arm);
//			index_arm_ss << separator_arm << idx_arm;
//			separator_arm = ";";
//		}
//		_index_str_arm = index_arm_ss.str();
//
//		_index_vec_base.resize(_num_dof_base, 0.0);
//		std::stringstream index_base_ss;
//		index_base_ss << "[" << _num_dof_base << ",1]";
//		const char* separator_base = "";
//		for(int idx_base=_num_dof_arm; idx_base<_num_dof_arm+_num_dof_base; idx_base++)
//		{
//			_index_vec_base.push_back(idx_base);
//			index_base_ss << separator_base << idx_base;
//			separator_base = ";";
//		}
//		_index_str_base = index_base_ss.str();
//
//		_max_vel_js_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_MAX_VEL_JS_ARM);
//		_max_vel_js_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_MAX_VEL_JS_BASE);
//
//		_kp_js_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_KP_JS_ARM);
//		if(_num_dof_arm == DEFAULT_NUM_DOF_ARM)
//		{
//			_kp_js_arm << 300.0, 200.0, 150.0, 120.0, 10.0, 10.0, 10.0;
//		}
//		_kp_js_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_KP_JS_BASE);
//
//		_kv_js_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_KV_JS_ARM);
//		if(_num_dof_arm == DEFAULT_NUM_DOF_ARM)
//		{
//			_kv_js_arm << 2.0, 4.0, 2.0, 1.2, 0.2, 0.3, 0.02;
//		}
//		_kv_js_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_KV_JS_BASE);
//
//		_kp_os_linear = Eigen::MatrixXd::Constant(3, 1, DEFAULT_KP_OS_LINEAR);
//		_kp_os_angular = Eigen::MatrixXd::Constant(3, 1, DEFAULT_KP_OS_ANGULAR);
//		_kv_os_linear = Eigen::MatrixXd::Constant(3, 1, DEFAULT_KV_OS_LINEAR);
//		_kv_os_angular = Eigen::MatrixXd::Constant(3, 1, DEFAULT_KV_OS_ANGULAR);
//
//		_kp_js_nakamura_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_KP_JS_NAKAMURA_ARM);
//		if(_num_dof_arm == DEFAULT_NUM_DOF_ARM)
//		{
//			_kp_js_nakamura_arm << 30.0, 20.0, 15.0, 20.0, 10.0, 10.0, 10.0;
//		}
//		_kp_js_nakamura_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_KP_JS_NAKAMURA_BASE);
//
//		_kv_js_nakamura_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_KV_JS_NAKAMURA_ARM);
//		if(_num_dof_arm == DEFAULT_NUM_DOF_ARM)
//		{
//			_kv_js_nakamura_arm << 1.0, 2.0, 1.0, 0.4, 0.1, 0.1, 0.01;
//		}
//		_kv_js_nakamura_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_KV_JS_NAKAMURA_BASE);
//		if(_num_dof_base== DEFAULT_NUM_DOF_BASE)
//		{
//			_kv_js_nakamura_base << 10.0, 10.0, 2.0;
//		}
//
//		_joint_weights_nakamura_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_JOINT_WEIGHTS_NAKAMURA_ARM);
//		_joint_weights_nakamura_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE);
//		_joint_weights_nakamura_base_no_rotation = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE);
//		if(_num_dof_base== DEFAULT_NUM_DOF_BASE)
//		{
//			_joint_weights_nakamura_base_no_rotation << DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE, DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE, DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE_LITTLE_MOTION;
//		}
//		_joint_weights_nakamura_base_little_motion = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_JOINT_WEIGHTS_NAKAMURA_BASE_LITTLE_MOTION);
//
//		_home_config_js_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_HOME_CONFIG_JS_ARM);
//		if(_num_dof_arm == DEFAULT_NUM_DOF_ARM)
//		{
//			_home_config_js_arm << 0.0, -0.14, 0.0, 2.18, 0.0, 0.2, -0.13;
//		}
//		_home_config_js_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_HOME_CONFIG_JS_BASE);
//
//		_pos_epsilon_js_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_POS_EPSILON_JS_ARM);
//		_pos_epsilon_js_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_POS_EPSILON_JS_BASE);
//
//		_vel_epsilon_js_arm = Eigen::MatrixXd::Constant(_num_dof_arm, 1, DEFAULT_VEL_EPSILON_JS_ARM);
//		_vel_epsilon_js_base = Eigen::MatrixXd::Constant(_num_dof_base, 1, DEFAULT_VEL_EPSILON_JS_BASE);
//
//		_pos_epsilon_os_linear = Eigen::MatrixXd::Constant(3, 1, DEFAULT_POS_EPSILON_OS_LINEAR);
//		_pos_epsilon_os_angular = Eigen::MatrixXd::Constant(3, 1, DEFAULT_POS_EPSILON_OS_ANGULAR);
//
//		_vel_epsilon_os_linear = Eigen::MatrixXd::Constant(3, 1, DEFAULT_VEL_EPSILON_OS_LINEAR);
//		_vel_epsilon_os_angular = Eigen::MatrixXd::Constant(3, 1, DEFAULT_VEL_EPSILON_OS_ANGULAR);
//
//	}
//
//	HybridAutomatonFactory::~HybridAutomatonFactory()
//	{
//
//	}
//
//	HybridAutomatonFactory::HybridAutomatonFactory(const HybridAutomatonFactory& haf)
//	{
//
//	}
//
//	Eigen::MatrixXd HybridAutomatonFactory::_combineArmAndBase(const Eigen::MatrixXd& arm_vector, const Eigen::MatrixXd base_vector)
//	{
//		Eigen::MatrixXd combined_vector(arm_vector.rows()+base_vector.rows(), arm_vector.cols());
//		combined_vector <<  arm_vector,
//			base_vector;
//		return combined_vector;
//	}
//
//}
