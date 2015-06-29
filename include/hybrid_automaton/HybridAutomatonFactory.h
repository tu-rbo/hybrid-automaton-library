//#ifndef HYBRID_AUTOMATON_FACTORY_H
//#define HYBRID_AUTOMATON_FACTORY_H
//
//#include "hybrid_automaton/HybridAutomaton.h"
//
//#include <boost/shared_ptr.hpp>
//
//namespace ha {
//
//    class HybridAutomatonFactory;
//    typedef boost::shared_ptr<HybridAutomatonFactory> HybridAutomatonFactoryPtr;
//    typedef boost::shared_ptr<const HybridAutomatonFactory> HybridAutomatonFactoryConstPtr;
//
//    /**
//     * @brief A class to easily generate basic hybrid automata, control modes, control switches, jump conditions...
//     *
//     */
//    class HybridAutomatonFactory
//    {
//    public:
//
//        typedef boost::shared_ptr<HybridAutomatonFactory> Ptr;
//        typedef boost::shared_ptr<const HybridAutomatonFactory> ConstPtr;
//
//        /**
//         * @brief Default constructor
//         *
//         */
//        HybridAutomatonFactory();
//
//        /**
//         * @brief Constructor
//         *
//         * @param num_dof_arm Number of degrees of freedom of the arm
//         * @param num_dof_base Number of degrees of freedom of the base
//         */
//        HybridAutomatonFactory(const int& num_dof_arm, const int& num_dof_base);
//
//        /**
//         * @brief Destructor
//         *
//         */
//        virtual ~HybridAutomatonFactory();
//
//        /**
//         * @brief Copy constructor
//         *
//         * @param haf Object to make a copy from
//         */
//        HybridAutomatonFactory(const HybridAutomatonFactory& haf);
//
//        /**
//         * @brief Clone function
//         *
//         * @return HybridAutomatonFactoryPtr Pointer to the generated clone
//         */
//        HybridAutomatonFactoryPtr clone() const
//        {
//            return (HybridAutomatonFactoryPtr(_doClone()));
//        }
//
//    protected:
//
//        /**
//         * @brief Performs the cloning operation (this solves some issues of inheritance and smart pointers)
//         *
//         * @return HybridAutomatonFactory Pointer to the generated clone
//         */
//        virtual HybridAutomatonFactory* _doClone() const
//        {
//            return (new HybridAutomatonFactory(*this));
//        }
//
//        /**
//         * @brief Concatenates two vectors, one for the arm and one for the base
//         *
//         * @param arm_vector Vector for the arm. It will be the first _num_dof_arm values of the resulting vector
//         * @param base_vector Vector for the base. It will be the last _num_dof_base of the resulting vector
//         * @return Eigen::MatrixXd Resulting vector that contains both input vectors
//         */
//        virtual Eigen::MatrixXd _combineArmAndBase(const Eigen::MatrixXd& arm_vector, const Eigen::MatrixXd base_vector);
//
//        /// Number of degrees of freedom of the arm
//        int _num_dof_arm;
//
//        /// Number of degrees of freedom of the base
//        int _num_dof_base;
//
//        /// Indices that define the convention for the joints of the arm and the base
//        std::string _index_str_base;
//        std::string _index_str_arm;
//        std::vector<int> _index_vec_base;
//        std::vector<int> _index_vec_arm;
//
//        /// Maximum joint space (js) velocity
//        Eigen::MatrixXd _max_vel_js_arm;
//        Eigen::MatrixXd _max_vel_js_base;
//
//        /// Joint space (js) gains
//        Eigen::MatrixXd _kp_js_arm;
//        Eigen::MatrixXd _kp_js_base;
//        Eigen::MatrixXd _kv_js_arm;
//        Eigen::MatrixXd _kv_js_base;
//
//        /// Operational space gains (3x orientation + 3x position)
//        Eigen::MatrixXd _kp_os_linear;
//        Eigen::MatrixXd _kp_os_angular;
//        Eigen::MatrixXd _kv_os_linear;
//        Eigen::MatrixXd _kv_os_angular;
//
//        /// Joint space integration gains for operational space Nakamura control set
//        Eigen::MatrixXd _kp_js_nakamura_arm;
//        Eigen::MatrixXd _kp_js_nakamura_base;
//        Eigen::MatrixXd _kv_js_nakamura_arm;
//        Eigen::MatrixXd _kv_js_nakamura_base;
//
//        /// Weights of each joint for operational space Nakamura control set
//        Eigen::MatrixXd _joint_weights_nakamura_arm;
//        Eigen::MatrixXd _joint_weights_nakamura_base;
//        Eigen::MatrixXd _joint_weights_nakamura_base_no_rotation;
//        Eigen::MatrixXd _joint_weights_nakamura_base_little_motion;
//
//        /// Home configuration - usually a good initial position to begin the interaction and/or a safe
//        /// position to return to
//        Eigen::MatrixXd _home_config_js_arm;
//        Eigen::MatrixXd _home_config_js_base;
//
//        /// Convergence radius of a joint space (js) controller
//        Eigen::MatrixXd _pos_epsilon_js_arm;
//        Eigen::MatrixXd _pos_epsilon_js_base;
//
//        /// Convergence velocity of a joint space (js) controller
//        Eigen::MatrixXd _vel_epsilon_js_arm;
//        Eigen::MatrixXd _vel_epsilon_js_base;
//
//        /// Convergence radius of an operational space (os) controller
//        Eigen::MatrixXd _pos_epsilon_os_linear;
//        Eigen::MatrixXd _pos_epsilon_os_angular;
//
//        /// Convergence velocity of an operational space (os) controller
//        Eigen::MatrixXd _vel_epsilon_os_linear;
//        Eigen::MatrixXd _vel_epsilon_os_angular;
//
//    };
//
//}
//
//#endif
