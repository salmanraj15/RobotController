#include <gtest/gtest.h>

#include <array>

#include "robot_controller/motor/SimulatedMotor.hpp"
#include "robot_controller/simulation/RobotSimulator.hpp"

class RobotSimulatorTests : public ::testing::Test
{
protected:
    SimulatedMotor motor;

    std::array<Joint, 1> joints{
        Joint{
            Angle{-90.0},
            Angle{90.0},
            AngularVelocity{60.0},
            AngularAcceleration{30.0},
            motor}};

    RobotSimulator simulator{joints};
};

// A stationary joint should remain stationary without acceleration.
TEST_F(RobotSimulatorTests, KeepsStationaryJointAtRest)
{
    simulator.update(Duration{1.0});

    const JointState state = joints[0].state();

    EXPECT_DOUBLE_EQ(state.position.degrees(), 0.0);
    EXPECT_DOUBLE_EQ(
        state.velocity.degreesPerSecond(),
        0.0);
}

// Acceleration should update velocity and then position.
TEST_F(RobotSimulatorTests, IntegratesPositiveAcceleration)
{
    ASSERT_TRUE(
        joints[0].setAcceleration(
            AngularAcceleration{10.0}));

    simulator.update(Duration{0.5});

    const JointState state = joints[0].state();

    EXPECT_DOUBLE_EQ(
        state.velocity.degreesPerSecond(),
        5.0);

    EXPECT_DOUBLE_EQ(
        state.position.degrees(),
        1.25);
}

// Velocity should not exceed the joint's configured maximum.
TEST_F(RobotSimulatorTests, LimitsPositiveVelocity)
{
    ASSERT_TRUE(
        joints[0].initializePosition(Angle{-80.0}));

    ASSERT_TRUE(
        joints[0].setAcceleration(
            AngularAcceleration{30.0}));

    simulator.update(Duration{2.0});

    const JointState state = joints[0].state();

    EXPECT_DOUBLE_EQ(
        state.velocity.degreesPerSecond(),
        60.0);
}

// Velocity should not exceed the configured limit in the negative direction.
TEST_F(RobotSimulatorTests, LimitsNegativeVelocity)
{
    ASSERT_TRUE(
        joints[0].initializePosition(Angle{80.0}));

    ASSERT_TRUE(
        joints[0].setAcceleration(
            AngularAcceleration{-30.0}));

    simulator.update(Duration{2.0});

    const JointState state = joints[0].state();

    EXPECT_DOUBLE_EQ(
        state.velocity.degreesPerSecond(),
        -60.0);
}

// The joint should stop at its maximum position limit.
TEST_F(RobotSimulatorTests, EnforcesMaximumPositionLimit)
{
    ASSERT_TRUE(
        joints[0].initializePosition(Angle{80.0}));

    ASSERT_TRUE(
        joints[0].setAcceleration(
            AngularAcceleration{30.0}));

    simulator.update(Duration{1.0});

    const JointState state = joints[0].state();

    EXPECT_DOUBLE_EQ(state.position.degrees(), 90.0);
    EXPECT_DOUBLE_EQ(state.velocity.degreesPerSecond(), 0.0);
}

// The joint should stop at its minimum position limit.
TEST_F(RobotSimulatorTests, EnforcesMinimumPositionLimit)
{
    ASSERT_TRUE(
        joints[0].initializePosition(Angle{-80.0}));

    ASSERT_TRUE(
        joints[0].setAcceleration(
            AngularAcceleration{-30.0}));

    simulator.update(Duration{1.0});

    const JointState state = joints[0].state();

    EXPECT_DOUBLE_EQ(state.position.degrees(), -90.0);
    EXPECT_DOUBLE_EQ(state.velocity.degreesPerSecond(), 0.0);
}

// A zero-duration update should not change the joint state.
TEST_F(RobotSimulatorTests, ZeroDurationDoesNotChangeState)
{
    ASSERT_TRUE(
        joints[0].initializePosition(Angle{20.0}));

    ASSERT_TRUE(
        joints[0].setAcceleration(
            AngularAcceleration{10.0}));

    simulator.update(Duration{0.0});

    const JointState state = joints[0].state();

    EXPECT_DOUBLE_EQ(state.position.degrees(), 20.0);
    EXPECT_DOUBLE_EQ(state.velocity.degreesPerSecond(), 0.0);
}

// A negative duration should leave the joint state unchanged.
TEST_F(RobotSimulatorTests, NegativeDurationDoesNotChangeState)
{
    ASSERT_TRUE(
        joints[0].initializePosition(Angle{20.0}));

    ASSERT_TRUE(
        joints[0].setAcceleration(
            AngularAcceleration{10.0}));

    simulator.update(Duration{-1.0});

    const JointState state = joints[0].state();

    EXPECT_DOUBLE_EQ(state.position.degrees(), 20.0);
    EXPECT_DOUBLE_EQ(state.velocity.degreesPerSecond(), 0.0);
}

// A large time step should still respect the joint's position limit.
TEST_F(RobotSimulatorTests, LargeTimeStepRespectsPositionLimit)
{
    ASSERT_TRUE(
        joints[0].initializePosition(Angle{0.0}));

    ASSERT_TRUE(
        joints[0].setAcceleration(
            AngularAcceleration{30.0}));

    simulator.update(Duration{10.0});

    const JointState state = joints[0].state();

    EXPECT_DOUBLE_EQ(state.position.degrees(), 90.0);
    EXPECT_DOUBLE_EQ(state.velocity.degreesPerSecond(), 0.0);
}

// Hitting a position limit should also clear the motor acceleration.
TEST_F(RobotSimulatorTests, ClearsAccelerationAtPositionLimit)
{
    ASSERT_TRUE(
        joints[0].initializePosition(Angle{80.0}));

    ASSERT_TRUE(
        joints[0].setAcceleration(
            AngularAcceleration{30.0}));

    simulator.update(Duration{1.0});

    const JointState state = joints[0].state();

    EXPECT_DOUBLE_EQ(state.position.degrees(), 90.0);
    EXPECT_DOUBLE_EQ(state.velocity.degreesPerSecond(), 0.0);

    // The motor should stop accelerating after hitting the limit.
    EXPECT_DOUBLE_EQ(
        joints[0].motorAcceleration().degreesPerSecondSquared(),
        0.0);
}

// Position should be integrated correctly when velocity reaches its limit
// partway through a time step.
TEST_F(RobotSimulatorTests, IntegratesPositionWhenVelocityLimitIsReached)
{
    // Start at 55 degrees per second, just below the 60 degrees per
    // second velocity limit.
    joints[0].simulate(
        AngularVelocity{55.0},
        Angle{0.0});

    ASSERT_TRUE(
        joints[0].setAcceleration(
            AngularAcceleration{30.0}));

    // The joint accelerates until it reaches 60 degrees per second,
    // then continues moving at that maximum speed for the rest of the second.
    simulator.update(Duration{1.0});

    const JointState state = joints[0].state();

    // The velocity must remain within the configured limit.
    EXPECT_DOUBLE_EQ(
        state.velocity.degreesPerSecond(),
        60.0);

    // Expected displacement:
    //  - Accelerating from 55 to 60 degrees per second for 1/6 second.
    //  - Moving at 60 degrees per second for the remaining 5/6 second.
    EXPECT_NEAR(
        state.position.degrees(),
        59.583333333333336,
        1e-9);
}

// A joint already at maximum velocity should not gain extra speed
// or displacement from acceleration beyond that velocity limit.
TEST_F(RobotSimulatorTests, HoldsMaximumVelocityWhenAcceleratingFurther)
{
    // Start at the configured maximum velocity, away from position limits.
    ASSERT_TRUE(
        joints[0].initializePosition(Angle{0.0}));

    joints[0].simulate(
        AngularVelocity{60.0},
        Angle{0.0});

    ASSERT_TRUE(
        joints[0].setAcceleration(
            AngularAcceleration{30.0}));

    // Even with further positive acceleration, velocity stays capped.
    simulator.update(Duration{1.0});

    const JointState state = joints[0].state();

    EXPECT_DOUBLE_EQ(
        state.velocity.degreesPerSecond(),
        60.0);

    // At 60 degrees per second for one second, the joint travels 60 degrees.
    EXPECT_DOUBLE_EQ(
        state.position.degrees(),
        60.0);
}

// A joint already at maximum negative speed should not accelerate
// beyond its negative velocity limit.
TEST_F(RobotSimulatorTests, HoldsMinimumVelocityWhenAcceleratingFurther)
{
    // Start at the configured negative velocity limit.
    ASSERT_TRUE(
        joints[0].initializePosition(Angle{0.0}));

    joints[0].simulate(
        AngularVelocity{-60.0},
        Angle{0.0});

    ASSERT_TRUE(
        joints[0].setAcceleration(
            AngularAcceleration{-30.0}));

    // The joint should keep moving at -60 degrees per second.
    simulator.update(Duration{1.0});

    const JointState state = joints[0].state();

    EXPECT_DOUBLE_EQ(
        state.velocity.degreesPerSecond(),
        -60.0);

    // At -60 degrees per second for one second, position becomes -60 degrees.
    EXPECT_DOUBLE_EQ(
        state.position.degrees(),
        -60.0);
}
