#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "hybrid_automaton/HybridAutomaton.h"
#include "hybrid_automaton/Controller.h"
#include "hybrid_automaton/ForceTorqueSensor.h"

#include "tests/MockDescriptionTree.h"
#include "tests/MockDescriptionTreeNode.h"

using namespace std;
using namespace ha;

using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::AtLeast;
using ::testing::_;


namespace ForceTorqueSensorTest {
    class MockSystem : public ha::System {
    public:
        MOCK_CONST_METHOD0(getDof, int () );
        MOCK_CONST_METHOD0(getJointConfiguration, ::Eigen::MatrixXd () );
        MOCK_CONST_METHOD0(getJointVelocity, ::Eigen::MatrixXd () );
        MOCK_CONST_METHOD1(getForceTorqueMeasurement, ::Eigen::MatrixXd (const std::string& frame_id) );
        MOCK_CONST_METHOD0(getCurrentTime, ::Eigen::MatrixXd () );
        MOCK_CONST_METHOD1(getFramePose, ::Eigen::MatrixXd (const std::string& frame_id) );
    };
}


TEST(ForceTorqueSensor, FrameId) {
    using ::testing::Invoke;

    ForceTorqueSensorTest::MockSystem* _ms = new ForceTorqueSensorTest::MockSystem();
    System::ConstPtr ms(_ms);

    //The sensor reading of the system - we will mock it as constant
    ::Eigen::MatrixXd ftSensorMat(6,1);
    ftSensorMat<<1.0,2.0,3.0,4.0,5.0,6.0;
    EXPECT_CALL(*_ms, getForceTorqueMeasurement("ee"))
        .WillRepeatedly(Return(ftSensorMat));

    ::Eigen::MatrixXd poseMat(4,4);
    poseMat.row(0) << 0.0,1.0,0.0,   0.; //1.0;
    poseMat.row(1) << 1.0,0.0,0.0,   0.; //2.0;
    poseMat.row(2) << 0.0,0.0,1.0,   0.; //3.0;
    poseMat.row(3) << 0.0,0.0,0.0,   1.0;

    EXPECT_CALL(*_ms, getFramePose(_))
        .WillRepeatedly(Return(poseMat));


    ::Eigen::MatrixXd ftSensorMatTransformed(6,1);
    ftSensorMatTransformed << 2.0,1.0,3.0,5.0,4.0,6.0;


    //std::map<std::string, std::string> add_args;
    //fillMap(add_args);

    // FIRST TEST: no frame ID

    {
    // Mocked description used to deserialize force torque sensor
    MockDescriptionTreeNode::Ptr ft_node(new MockDescriptionTreeNode);

    ForceTorqueSensor::Ptr ftsensor(new ForceTorqueSensor);
    EXPECT_CALL(*ft_node, getType()).WillRepeatedly(Return("Sensor"));
    EXPECT_CALL(*ft_node, getAttributeString(_, _))
        .WillRepeatedly(Return(false));
    EXPECT_CALL(*ft_node, getAttributeString(std::string("type"), _))
        .WillRepeatedly(DoAll(SetArgReferee<1>("ForceTorqueSensor"),Return(true)));
    EXPECT_CALL(*ft_node, getAttributeString(std::string("frame_id"), _))
        .WillRepeatedly(DoAll(SetArgReferee<1>(""),Return(true)));

    HybridAutomaton ha;
    ftsensor->deserialize(ft_node, ms, &ha);

    EXPECT_EQ("ForceTorqueSensor", ftsensor->getType());

    ::Eigen::MatrixXd ret = ftsensor->getCurrentValue();
    EXPECT_EQ(ftSensorMat, ret);

    }

    // SECOND TEST: now check transformation
    {
    // Mocked description used to deserialize force torque sensor
    MockDescriptionTreeNode::Ptr ft_node(new MockDescriptionTreeNode);

    ForceTorqueSensor::Ptr ftsensor(new ForceTorqueSensor);
    EXPECT_CALL(*ft_node, getType()).WillRepeatedly(Return("Sensor"));
    EXPECT_CALL(*ft_node, getAttributeString(_, _))
        .WillRepeatedly(Return(false));
    EXPECT_CALL(*ft_node, getAttributeString(std::string("type"), _))
        .WillRepeatedly(DoAll(SetArgReferee<1>("ForceTorqueSensor"),Return(true)));
    EXPECT_CALL(*ft_node, getAttributeString(std::string("frame_id"), _))
        .WillRepeatedly(DoAll(SetArgReferee<1>("base"),Return(true)));

    HybridAutomaton ha;
    ftsensor->deserialize(ft_node, ms, &ha);

    EXPECT_EQ("ForceTorqueSensor", ftsensor->getType());

    ::Eigen::MatrixXd ret = ftsensor->getCurrentValue();
    //cout << ret << endl;
    EXPECT_EQ(ftSensorMatTransformed, ret);

    }


//    std::string val;
//    ctrl->getArgument("extra_arg", val);
//    EXPECT_EQ(add_args["extra_arg"], val);




}
