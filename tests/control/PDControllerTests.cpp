#include <gtest/gtest.h>

#include "robot_controller/control/PDController.hpp"

class PDControllerTests : public ::testing::Test
{
protected:
    PDController controller{
        PDControllerConfig{
            1.0,
            0.5,
            AngularAcceleration{30.0}}};
};

// With no position or velocity error, no acceleration should be requested.
TEST_F(PDControllerTests, ReturnsZeroAccelerationAtTarget)
{
    const JointCommand command{
        Angle{45.0}};

    const JointState actual_state{
        Angle{45.0},
        AngularVelocity{0.0}};

    const AngularAcceleration acceleration =
        controller.calculate(command, actual_state);

    EXPECT_DOUBLE_EQ(
        acceleration.degreesPerSecondSquared(),
        0.0);
}

// Position error should produce proportional acceleration.
TEST_F(PDControllerTests, CalculatesProportionalAcceleration)
{
    const JointCommand command{
        Angle{50.0}};

    const JointState actual_state{
        Angle{40.0},
        AngularVelocity{0.0}};

    const AngularAcceleration acceleration =
        controller.calculate(command, actual_state);

    EXPECT_DOUBLE_EQ(
        acceleration.degreesPerSecondSquared(),
        10.0);
}

// Negative position error should produce negative acceleration.
TEST_F(PDControllerTests, CalculatesNegativeProportionalAcceleration)
{
    const JointCommand command{
        Angle{30.0}};

    const JointState actual_state{
        Angle{40.0},
        AngularVelocity{0.0}};

    const AngularAcceleration acceleration =
        controller.calculate(command, actual_state);

    EXPECT_DOUBLE_EQ(
        acceleration.degreesPerSecondSquared(),
        -10.0);
}

// Positive velocity should reduce the requested acceleration.
TEST_F(PDControllerTests, AppliesVelocityFeedback)
{
    const JointCommand command{
        Angle{50.0}};

    const JointState actual_state{
        Angle{40.0},
        AngularVelocity{4.0}};

    const AngularAcceleration acceleration =
        controller.calculate(command, actual_state);

    EXPECT_DOUBLE_EQ(
        acceleration.degreesPerSecondSquared(),
        8.0);
}

// Negative velocity should increase the requested acceleration.
TEST_F(PDControllerTests, AppliesNegativeVelocityFeedback)
{
    const JointCommand command{
        Angle{50.0}};

    const JointState actual_state{
        Angle{40.0},
        AngularVelocity{-4.0}};

    const AngularAcceleration acceleration =
        controller.calculate(command, actual_state);

    EXPECT_DOUBLE_EQ(
        acceleration.degreesPerSecondSquared(),
        12.0);
}

// Large positive error should be limited to the configured maximum.
TEST_F(PDControllerTests, LimitsPositiveAcceleration)
{
    const JointCommand command{
        Angle{100.0}};

    const JointState actual_state{
        Angle{0.0},
        AngularVelocity{0.0}};

    const AngularAcceleration acceleration =
        controller.calculate(command, actual_state);

    EXPECT_DOUBLE_EQ(
        acceleration.degreesPerSecondSquared(),
        30.0);
}

// Large negative error should be limited to the configured maximum.
TEST_F(PDControllerTests, LimitsNegativeAcceleration)
{
    const JointCommand command{
        Angle{-100.0}};

    const JointState actual_state{
        Angle{0.0},
        AngularVelocity{0.0}};

    const AngularAcceleration acceleration =
        controller.calculate(command, actual_state);

    EXPECT_DOUBLE_EQ(
        acceleration.degreesPerSecondSquared(),
        -30.0);
}

// Acceleration exactly at the positive limit should be unchanged.
TEST_F(PDControllerTests, KeepsPositiveAccelerationAtLimit)
{
    const JointCommand command{
        Angle{30.0}};

    const JointState actual_state{
        Angle{0.0},
        AngularVelocity{0.0}};

    const AngularAcceleration acceleration =
        controller.calculate(command, actual_state);

    EXPECT_DOUBLE_EQ(
        acceleration.degreesPerSecondSquared(),
        30.0);
}

// Acceleration exactly at the negative limit should be unchanged.
TEST_F(PDControllerTests, KeepsNegativeAccelerationAtLimit)
{
    const JointCommand command{
        Angle{-30.0}};

    const JointState actual_state{
        Angle{0.0},
        AngularVelocity{0.0}};

    const AngularAcceleration acceleration =
        controller.calculate(command, actual_state);

    EXPECT_DOUBLE_EQ(
        acceleration.degreesPerSecondSquared(),
        -30.0);
}

// Position error and velocity feedback should both contribute
// to the final acceleration command.
TEST_F(PDControllerTests, CombinesPositionErrorAndVelocityFeedback)
{
    const JointCommand command{
        Angle{50.0}};

    const JointState actual_state{
        Angle{40.0},
        AngularVelocity{4.0}};

    const AngularAcceleration acceleration =
        controller.calculate(command, actual_state);

    // Position contribution: 1.0 * (50 - 40) = 10.
    // Velocity contribution: 0.5 * 4 = 2.
    // Final acceleration: 10 - 2 = 8 degrees per second squared.
    EXPECT_DOUBLE_EQ(
        acceleration.degreesPerSecondSquared(),
        8.0);
}

// Combined positive feedback above the limit should be clamped
// to the configured maximum acceleration.
TEST_F(PDControllerTests, ClampsCombinedPositiveAcceleration)
{
    const JointCommand command{
        Angle{100.0}};

    const JointState actual_state{
        Angle{0.0},
        AngularVelocity{-10.0}};

    const AngularAcceleration acceleration =
        controller.calculate(command, actual_state);

    // Position contribution: 1.0 * (100 - 0) = 100.
    // Negative velocity contributes another +5.
    // The result is 105, so the controller must clamp it to +30.
    EXPECT_DOUBLE_EQ(
        acceleration.degreesPerSecondSquared(),
        30.0);
}

// Combined negative feedback below the limit should be clamped
// to the configured minimum acceleration.
TEST_F(PDControllerTests, ClampsCombinedNegativeAcceleration)
{
    const JointCommand command{
        Angle{-100.0}};

    const JointState actual_state{
        Angle{0.0},
        AngularVelocity{10.0}};

    const AngularAcceleration acceleration =
        controller.calculate(command, actual_state);

    // Position contribution: 1.0 * (-100 - 0) = -100.
    // Positive velocity contributes another -5.
    // The result is -105, so the controller must clamp it to -30.
    EXPECT_DOUBLE_EQ(
        acceleration.degreesPerSecondSquared(),
        -30.0);
}
