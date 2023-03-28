#include "mock_common.h"
#define _FUNC(m, n, method, ...) EQ##n(m, method)
#define _INST(m, n, method, ...) EQ##n(m, method)
RobotAlgorithm_DECLARES

    /*TEST_F(MockInterfaceTest, rpyToQuaternion)
    {
        auto robot = cli->getRobotInterface(cli->getRobotNames().front());
        std::vector rpy(3, 0.1);
        std::vector tmp = { 0.99638, 0.0473595, 0.0523491, 0.0473595 };
        EXPECT_NEAR(robot->getRobotAlgorithm()->rpyToQuaternion(rpy).at(0),
                    tmp.at(0), 0.001);
        EXPECT_NEAR(robot->getRobotAlgorithm()->rpyToQuaternion(rpy).at(1),
                    tmp.at(1), 0.001);
        EXPECT_NEAR(robot->getRobotAlgorithm()->rpyToQuaternion(rpy).at(2),
                    tmp.at(2), 0.001);
        EXPECT_NEAR(robot->getRobotAlgorithm()->rpyToQuaternion(rpy).at(3),
                    tmp.at(3), 0.001);
    }

    TEST_F(MockInterfaceTest, quaternionToRpy)
    {
        auto robot = cli->getRobotInterface(cli->getRobotNames().front());
        std::vector quat(4, 0.1);
        std::vector<double> tmp = { 3.09995, -0, 3.09995 };
        EXPECT_NEAR(robot->getRobotAlgorithm()->quaternionToRpy(quat).at(0),
                    tmp.at(0), 0.001);
        EXPECT_NEAR(robot->getRobotAlgorithm()->quaternionToRpy(quat).at(1),
                    tmp.at(1), 0.001);
        EXPECT_NEAR(robot->getRobotAlgorithm()->quaternionToRpy(quat).at(2),
                    tmp.at(2), 0.001);
    }*/
