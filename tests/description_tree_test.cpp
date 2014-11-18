#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <string>

#include "hybrid_automaton/DescriptionTreeNode.h"
#include "hybrid_automaton/tests/MockDescriptionTreeNode.h"

using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::_;

using namespace ::ha;

TEST(DescriptionTree, setAttribute) {
	using namespace ha;
	using ::testing::Return;

	MockDescriptionTreeNode mock_dtn;

	int dimensionality = 6;
	std::string dimensionality_exp = "6";

	EXPECT_CALL(mock_dtn, setAttributeString(std::string("dimensionality"), dimensionality_exp));

	mock_dtn.setAttribute<int>("dimensionality", dimensionality);
}

TEST(DescriptionTree, getAttribute) {
	using namespace ha;

	MockDescriptionTreeNode mock_dtn;

	int dimensionality = 6;
	std::string dimensionality_exp = "6";

	EXPECT_CALL(mock_dtn, getAttributeString(std::string("dimensionality"), _)).WillOnce(DoAll(SetArgReferee<1>(dimensionality_exp), Return(true)));

	int dimensionality_result;
	mock_dtn.getAttribute<int>("dimensionality", dimensionality_result);

	EXPECT_EQ(dimensionality, dimensionality_result);
}

TEST(DescriptionTree, setAttributeVector) {
	using namespace ha;
	using ::testing::Return;

	MockDescriptionTreeNode mock_dtn;

	::Eigen::MatrixXd goal(5,1);
	goal << 1.,
		2.,
		3.,
		4.,
		5.;
	std::string goal_exp = "[5,1]1;2;3;4;5";

	EXPECT_CALL(mock_dtn, setAttributeString(std::string("goal"), goal_exp));

	mock_dtn.setAttribute<::Eigen::MatrixXd>("goal", goal);
}

TEST(DescriptionTree, getAttributeVector) {
	using namespace ha;

	MockDescriptionTreeNode mock_dtn;


	std::string goal_exp = "[5,1]1;2;3;4;5";
	EXPECT_CALL(mock_dtn, getAttributeString(std::string("goal"), _)).WillOnce(DoAll(SetArgReferee<1>(goal_exp), Return(true)));

	::Eigen::MatrixXd goal_result(1,1);
	goal_result << 1.;
	mock_dtn.getAttribute<::Eigen::MatrixXd>("goal", goal_result);

	::Eigen::MatrixXd goal_true(5,1);
	goal_true << 1., 2., 3., 4., 5.;
	EXPECT_EQ(goal_result, goal_true);

	::Eigen::MatrixXd goal_false(5,1);
	goal_false << 2., 2., 2., 2., 2.;
	EXPECT_NE(goal_result, goal_false);
}

TEST(DescriptionTree, setAttributeMatrix) {
	using namespace ha;
	using ::testing::Return;

	MockDescriptionTreeNode mock_dtn;

	::Eigen::MatrixXd goal(5,2);
	goal << 1.,2.,3.,4.,5, 6.,7.,8.,9.,10.;
	std::string goal_exp = "[5,2]1,2;3,4;5,6;7,8;9,10";

	EXPECT_CALL(mock_dtn, setAttributeString(std::string("goal"), goal_exp));

	mock_dtn.setAttribute<::Eigen::MatrixXd>("goal", goal);
}

TEST(DescriptionTree, getAttributeMatrix) {
	using namespace ha;

	MockDescriptionTreeNode mock_dtn;


	std::string goal_exp = "[5,2]1,2;3,4;5,6;7,8;9,10";
	EXPECT_CALL(mock_dtn, getAttributeString(std::string("goal"), _)).WillOnce(DoAll(SetArgReferee<1>(goal_exp), Return(true)));

	::Eigen::MatrixXd goal_result(1,1);
	goal_result << 1.;
	mock_dtn.getAttribute<::Eigen::MatrixXd>("goal", goal_result);

	::Eigen::MatrixXd goal_true(5,2);
	goal_true << 1.,2.,3.,4.,5, 6.,7.,8.,9.,10.;
	EXPECT_EQ(goal_result, goal_true);

	::Eigen::MatrixXd goal_false(5,2);
	goal_false << 2., 2., 2., 2., 2., 2., 2., 2., 2., 2.;
	EXPECT_NE(goal_result, goal_false);
}