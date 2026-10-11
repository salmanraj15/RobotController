#include <gtest/gtest.h>
#include <limits>

#include "robot_controller/safety/SafetyLayer.hpp"
#include "robot_controller/motor/SimulatedMotor.hpp"

class SafetyLayerTests : public ::testing::Test
{
protected:
    SimulatedMotor motor;

    Joint joint{
        Angle{-90.0},
        Angle{90.0},
        AngularVelocity{180.0},
        AngularAcceleration{30.0},
        motor};

    SafetyLayer safety_layer;
};

// Acceleration inside the joint's limit should be accepted.
TEST_F(SafetyLayerTests, AcceptsAccelerationWithinLimit)
{
    EXPECT_TRUE(
        safety_layer.validate(
            joint,
            AngularAcceleration{20.0}));
}

// Acceleration exactly at the positive limit should be accepted.
TEST_F(SafetyLayerTests, AcceptsPositiveAccelerationAtLimit)
{
    EXPECT_TRUE(
        safety_layer.validate(
            joint,
            AngularAcceleration{30.0}));
}

// Acceleration above the positive limit should be rejected.
TEST_F(SafetyLayerTests, RejectsAccelerationAboveLimit)
{
    EXPECT_FALSE(
        safety_layer.validate(
            joint,
            AngularAcceleration{31.0}));
}

// Negative acceleration within the limit should be accepted.
TEST_F(SafetyLayerTests, AcceptsNegativeAccelerationWithinLimit)
{
    EXPECT_TRUE(
        safety_layer.validate(
            joint,
            AngularAcceleration{-20.0}));
}

// Negative acceleration beyond the limit should be rejected.
TEST_F(SafetyLayerTests, RejectsNegativeAccelerationBeyondLimit)
{
    EXPECT_FALSE(
        safety_layer.validate(
            joint,
            AngularAcceleration{-31.0}));
}

// Acceleration exactly at the negative limit should be accepted.
TEST_F(SafetyLayerTests, AcceptsNegativeAccelerationAtLimit)
{
    EXPECT_TRUE(
        safety_layer.validate(
            joint,
            AngularAcceleration{-30.0}));
}

// Zero acceleration is a valid request within the joint's limits.
TEST_F(SafetyLayerTests, AcceptsZeroAcceleration)
{
    EXPECT_TRUE(
        safety_layer.validate(
            joint,
            AngularAcceleration{0.0}));
}

// NaN is not a valid acceleration command and must be rejected.
TEST_F(SafetyLayerTests, RejectsNaNAcceleration)
{
    const double not_a_number =
        std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(
        safety_layer.validate(
            joint,
            AngularAcceleration{not_a_number}));
}

// Positive infinity must not be accepted as a valid command.
TEST_F(SafetyLayerTests, RejectsPositiveInfiniteAcceleration)
{
    const double positive_infinity =
        std::numeric_limits<double>::infinity();

    EXPECT_FALSE(
        safety_layer.validate(
            joint,
            AngularAcceleration{positive_infinity}));
}

// Negative infinity must not be accepted as a valid command.
TEST_F(SafetyLayerTests, RejectsNegativeInfiniteAcceleration)
{
    const double negative_infinity =
        -std::numeric_limits<double>::infinity();

    EXPECT_FALSE(
        safety_layer.validate(
            joint,
            AngularAcceleration{negative_infinity}));
}
