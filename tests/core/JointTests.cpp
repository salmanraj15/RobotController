#include <gtest/gtest.h>

#include "robot_controller/core/Joint.hpp"
#include "robot_controller/motor/SimulatedMotor.hpp"

class JointTests : public ::testing::Test
{
protected:
    SimulatedMotor motor;

    Joint joint{
        Angle{-90.0},
        Angle{90.0},
        AngularVelocity{180.0},
        AngularAcceleration{360.0},
        motor};
};

// A new joint should start from a known safe state.
TEST_F(JointTests, StartsWithZeroPositionAndVelocity)
{
    EXPECT_DOUBLE_EQ(joint.position().degrees(), 0.0);
    EXPECT_DOUBLE_EQ(
        joint.velocity().degreesPerSecond(),
        0.0);
}

// A position inside the configured limits should be accepted.
TEST_F(JointTests, AcceptsPositionInsideLimits)
{
    const bool initialized =
        joint.initializePosition(Angle{45.0});

    EXPECT_TRUE(initialized);
    EXPECT_DOUBLE_EQ(joint.position().degrees(), 45.0);
}

// A position outside the limits should be rejected.
TEST_F(JointTests, RejectsPositionOutsideLimits)
{
    const bool initialized =
        joint.initializePosition(Angle{100.0});

    EXPECT_FALSE(initialized);

    // Rejected initialization must not change the current position.
    EXPECT_DOUBLE_EQ(joint.position().degrees(), 0.0);
}

// Setting a target should update the joint command.
TEST_F(JointTests, StoresTargetPosition)
{
    joint.setTargetPosition(Angle{45.0});

    EXPECT_DOUBLE_EQ(joint.targetPosition().degrees(), 45.0);
}

// An acceleration within the configured limit should be accepted.
TEST_F(JointTests, AcceptsAccelerationWithinLimit)
{
    const bool accepted =
        joint.setAcceleration(AngularAcceleration{180.0});

    EXPECT_TRUE(accepted);
    EXPECT_DOUBLE_EQ(
        joint.motorAcceleration().degreesPerSecondSquared(),
        180.0);
}

// The exact acceleration limit should still be accepted.
TEST_F(JointTests, AcceptsAccelerationAtLimit)
{
    const bool accepted =
        joint.setAcceleration(AngularAcceleration{360.0});

    EXPECT_TRUE(accepted);
    EXPECT_DOUBLE_EQ(
        joint.motorAcceleration().degreesPerSecondSquared(),
        360.0);
}

// An acceleration above the configured limit should be rejected.
TEST_F(JointTests, RejectsAccelerationAboveLimit)
{
    const bool accepted =
        joint.setAcceleration(AngularAcceleration{361.0});

    EXPECT_FALSE(accepted);

    // A rejected command must not reach the motor.
    EXPECT_DOUBLE_EQ(
        joint.motorAcceleration().degreesPerSecondSquared(),
        0.0);
}

// Negative acceleration within the limit should be accepted.
TEST_F(JointTests, AcceptsNegativeAccelerationWithinLimit)
{
    const bool accepted =
        joint.setAcceleration(AngularAcceleration{-180.0});

    EXPECT_TRUE(accepted);
    EXPECT_DOUBLE_EQ(
        joint.motorAcceleration().degreesPerSecondSquared(),
        -180.0);
}

// The negative acceleration limit should also be accepted.
TEST_F(JointTests, AcceptsNegativeAccelerationAtLimit)
{
    const bool accepted =
        joint.setAcceleration(AngularAcceleration{-360.0});

    EXPECT_TRUE(accepted);
    EXPECT_DOUBLE_EQ(
        joint.motorAcceleration().degreesPerSecondSquared(),
        -360.0);
}

// Negative acceleration beyond the limit should be rejected.
TEST_F(JointTests, RejectsNegativeAccelerationBelowLimit)
{
    const bool accepted =
        joint.setAcceleration(AngularAcceleration{-361.0});

    EXPECT_FALSE(accepted);

    // A rejected command must not reach the motor.
    EXPECT_DOUBLE_EQ(
        joint.motorAcceleration().degreesPerSecondSquared(),
        0.0);
}

// Simulation should update the joint's physical state.
TEST_F(JointTests, UpdatesSimulatedState)
{
    joint.simulate(
        AngularVelocity{30.0},
        Angle{45.0});

    EXPECT_DOUBLE_EQ(joint.position().degrees(), 45.0);
    EXPECT_DOUBLE_EQ(
        joint.velocity().degreesPerSecond(),
        30.0);
}

// The state snapshot should contain the current position and velocity.
TEST_F(JointTests, ReturnsCurrentState)
{
    joint.simulate(
        AngularVelocity{30.0},
        Angle{45.0});

    const JointState state = joint.state();

    EXPECT_DOUBLE_EQ(state.position.degrees(), 45.0);
    EXPECT_DOUBLE_EQ(
        state.velocity.degreesPerSecond(),
        30.0);
}

// The joint should expose its configured limits.
TEST_F(JointTests, ReturnsConfiguredLimits)
{
    EXPECT_DOUBLE_EQ(joint.minPosition().degrees(), -90.0);
    EXPECT_DOUBLE_EQ(joint.maxPosition().degrees(), 90.0);
    EXPECT_DOUBLE_EQ(
        joint.maxVelocity().degreesPerSecond(),
        180.0);
    EXPECT_DOUBLE_EQ(
        joint.maxAcceleration().degreesPerSecondSquared(),
        360.0);
}