#include <gtest/gtest.h>

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