#include "hybrid_automaton/HybridAutomatonFactory.h"

#define DEFAULT_NUM_DOF_ARM 7
#define DEFAULT_NUM_DOF_BASE 3

namespace ha
{
HybridAutomatonFactory::HybridAutomatonFactory()
    : HybridAutomatonFactory(DEFAULT_NUM_DOF_ARM,DEFAULT_NUM_DOF_BASE)
{

}

HybridAutomatonFactory::HybridAutomatonFactory(const int& num_dof_arm, const int& num_dof_base)
    :_num_dof_arm(num_dof_arm), _num_dof_base(num_dof_base)
{

    _index_vec_arm.resize(_num_dof_arm, 0.0);
    std::stringstream index_arm_ss;
    index_arm_ss << "[" << _num_dof_arm << ",1]";
    const char* separator_arm = "";
    for(int idx_arm=0; idx_arm<_num_dof_arm; idx_arm++)
    {
        _index_vec_arm.push_back(idx_arm);
        index_arm_ss << separator_arm << idx_arm;
        separator_arm = ";";
    }
    _index_str_arm = index_arm_ss.str();

    _index_vec_base.resize(_num_dof_base, 0.0);
    std::stringstream index_base_ss;
    index_base_ss << "[" << _num_dof_base << ",1]";
    const char* separator_base = "";
    for(int idx_base=_num_dof_arm; idx_base<_num_dof_arm+_num_dof_base; idx_base++)
    {
        _index_vec_base.push_back(idx_base);
        index_base_ss << separator_base << idx_base;
        separator_base = ";";
    }
    _index_str_base = index_base_ss.str();

    _max_vel_js_arm.resize(_num_dof_arm, 1);
    _max_vel_js_base.resize(_num_dof_base, 1);

    _kp_js_arm.resize(_num_dof_arm, 1);
    _kp_js_base.resize(_num_dof_base, 1);
    _kv_js_arm.resize(_num_dof_arm, 1);
    _kv_js_base.resize(_num_dof_base, 1);

    _kp_os_linear.resize(3, 1);
    _kp_os_angular.resize(3, 1);
    _kv_os_linear.resize(3, 1);
    _kv_os_angular.resize(3, 1);

    _kp_js_nakamura_arm.resize(_num_dof_arm, 1);
    _kp_js_nakamura_base.resize(_num_dof_base, 1);
    _kv_js_nakamura_arm.resize(_num_dof_arm, 1);
    _kv_js_nakamura_base.resize(_num_dof_base, 1);

    _joint_weights_nakamura_arm.resize(_num_dof_arm, 1);
    _joint_weights_nakamura_base.resize(_num_dof_base, 1);

    _home_config_arm.resize(_num_dof_arm, 1);
    _home_config_base.resize(_num_dof_base, 1);

    _pos_epsilon_js_arm.resize(_num_dof_arm, 1);
    _pos_epsilon_js_base.resize(_num_dof_base, 1);

    _vel_epsilon_js_arm.resize(_num_dof_arm, 1);
    _vel_epsilon_js_base.resize(_num_dof_base, 1);

    _pos_epsilon_os_linear.resize(3, 1);
    _pos_epsilon_os_angular.resize(3, 1);

    _vel_epsilon_os_linear.resize(3, 1);
    _vel_epsilon_os_angular.resize(3, 1);

}

HybridAutomatonFactory::~HybridAutomatonFactory()
{

}

HybridAutomatonFactory::HybridAutomatonFactory(const HybridAutomatonFactory& haf)
{

}

Eigen::MatrixXd HybridAutomatonFactory::_combineArmAndBase(const Eigen::MatrixXd& arm_vector, const Eigen::MatrixXd base_vector)
{
    Eigen::MatrixXd combined_vector(arm_vector.rows()+base_vector.rows(), arm_vector.cols());
    combined_vector <<  arm_vector,
                        base_vector;
    return combined_vector;
}

}
