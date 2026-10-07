#include <gtest/gtest.h>

#include "robot_controller/core/Robot.hpp"

class RobotTests : public ::testing::Test
{
protected:
    Robot robot;
};

// A new robot should contain six joints at zero state.
TEST_F(RobotTests, StartsWithSixZeroedJoints)
{
    const RobotState state = robot.state();

    EXPECT_EQ(state.size(), joint_count);

    for (const JointState& joint : state)
    {
        EXPECT_DOUBLE_EQ(joint.position.degrees(), 0.0);
        EXPECT_DOUBLE_EQ(
            joint.velocity.degreesPerSecond(),
            0.0);
    }
}

// A valid joint position should be forwarded to the selected joint.
TEST_F(RobotTests, InitializesJointPosition)
{
    const bool initialized =
        robot.initializeJointPosition(
            JointIndex{1},
            Angle{45.0});

    EXPECT_TRUE(initialized);

    const RobotState state = robot.state();

    EXPECT_DOUBLE_EQ(
        state[1].position.degrees(),
        45.0);
}

// An invalid joint index should be rejected.
TEST_F(RobotTests, RejectsInvalidJointIndex)
{
    const bool initialized =
        robot.initializeJointPosition(
            JointIndex{joint_count},
            Angle{45.0});

    EXPECT_FALSE(initialized);

    const RobotState state = robot.state();

    for (const JointState& joint : state)
    {
        EXPECT_DOUBLE_EQ(joint.position.degrees(), 0.0);
    }
}

// An invalid joint index should reject the target command.
TEST_F(RobotTests, RejectsInvalidTargetPositionIndex)
{
    const bool accepted =
        robot.setJointTargetPosition(
            JointIndex{joint_count},
            Angle{60.0});

    EXPECT_FALSE(accepted);
}

// A valid acceleration command should be accepted.
TEST_F(RobotTests, AcceptsJointAcceleration)
{
    const bool accepted =
        robot.setJointAcceleration(
            JointIndex{2},
            AngularAcceleration{20.0});

    EXPECT_TRUE(accepted);
}

// An invalid joint index should reject the acceleration command.
TEST_F(RobotTests, RejectsInvalidAccelerationIndex)
{
    const bool accepted =
        robot.setJointAcceleration(
            JointIndex{joint_count},
            AngularAcceleration{20.0});

    EXPECT_FALSE(accepted);
}

// An acceleration above the joint limit should be rejected.
TEST_F(RobotTests, RejectsJointAccelerationAboveLimit)
{
    const bool accepted =
        robot.setJointAcceleration(
            JointIndex{2},
            AngularAcceleration{31.0});

    EXPECT_FALSE(accepted);
}

// The configured acceleration limit should be accepted.
TEST_F(RobotTests, AcceptsJointAccelerationAtLimit)
{
    const bool accepted =
        robot.setJointAcceleration(
            JointIndex{2},
            AngularAcceleration{30.0});

    EXPECT_TRUE(accepted);
}

// The controller should move a joint toward its target.
TEST_F(RobotTests, UpdatesJointTowardTarget)
{
    robot.setJointTargetPosition(
        JointIndex{0},
        Angle{60.0});

    robot.update(Duration{1.0});

    const RobotState state = robot.state();

    EXPECT_DOUBLE_EQ(
        state[0].velocity.degreesPerSecond(),
        30.0);

    EXPECT_DOUBLE_EQ(
        state[0].position.degrees(),
        30.0);
}

// The simulator should stop a joint at its position limit.
TEST_F(RobotTests, StopsAtMaximumJointPosition)
{
    robot.initializeJointPosition(
        JointIndex{1},
        Angle{89.0});

    robot.setJointTargetPosition(
        JointIndex{1},
        Angle{180.0});

    robot.update(Duration{1.0});

    const RobotState state = robot.state();

    EXPECT_DOUBLE_EQ(
        state[1].position.degrees(),
        90.0);

    EXPECT_DOUBLE_EQ(
        state[1].velocity.degreesPerSecond(),
        0.0);
}

// The simulator should stop a joint at its minimum position.
TEST_F(RobotTests, StopsAtMinimumJointPosition)
{
    robot.initializeJointPosition(
        JointIndex{1},
        Angle{-89.0});

    robot.setJointTargetPosition(
        JointIndex{1},
        Angle{-180.0});

    robot.update(Duration{1.0});

    const RobotState state = robot.state();

    EXPECT_DOUBLE_EQ(
        state[1].position.degrees(),
        -90.0);

    EXPECT_DOUBLE_EQ(
        state[1].velocity.degreesPerSecond(),
        0.0);
}

// The simulator should limit the joint to its maximum velocity.
TEST_F(RobotTests, LimitsJointVelocity)
{
    robot.setJointTargetPosition(
        JointIndex{0},
        Angle{180.0});

    robot.update(Duration{3.0});

    const RobotState state = robot.state();

    EXPECT_DOUBLE_EQ(
        state[0].velocity.degreesPerSecond(),
        60.0);
}

// A joint at its target should remain stationary.
TEST_F(RobotTests, HoldsPositionWhenTargetIsReached)
{
    robot.initializeJointPosition(
        JointIndex{0},
        Angle{45.0});

    robot.setJointTargetPosition(
        JointIndex{0},
        Angle{45.0});

    robot.update(Duration{1.0});

    const RobotState state = robot.state();

    EXPECT_DOUBLE_EQ(
        state[0].position.degrees(),
        45.0);

    EXPECT_DOUBLE_EQ(
        state[0].velocity.degreesPerSecond(),
        0.0);
}