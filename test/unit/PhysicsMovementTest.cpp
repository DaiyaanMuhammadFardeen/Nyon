#include <gtest/gtest.h>
#include "nyon/utils/MovementPhysics.h"
#include "nyon/utils/Physics.h"
#include "TestHelpers.h"

using namespace Nyon::Utils;

/**
 * @brief Unit tests for MovementPhysics utility functions.
 * 
 * Tests force application, impulse effects, velocity control,
 * and kinematic calculations with comprehensive validation.
 */
class MovementPhysicsTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        // Initialize test physics body
        testBody = Physics::Body();
        testBody.position = Nyon::Math::Vector2(0.0f, 0.0f);
        testBody.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
        testBody.acceleration = Nyon::Math::Vector2(0.0f, 0.0f);
        testBody.mass = 2.0f; // Non-unit mass for better testing
        testBody.friction = 0.0f;
        testBody.drag = 0.0f;
        testBody.maxSpeed = 1000.0f;
        testBody.isStatic = false;
        LOG_FUNC_EXIT();
    }

    Physics::Body testBody;
};

// Force application tests
TEST_F(MovementPhysicsTest, ApplyForce_Basic)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 force(100.0f, 50.0f);
    
    MovementPhysics::ApplyForce(testBody, force);
    
    LOG_VAR_DEBUG(testBody.acceleration.x);
    LOG_VAR_DEBUG(testBody.acceleration.y);
    // F = ma, so a = F/m
    EXPECT_FLOAT_NEAR(testBody.acceleration.x, force.x / testBody.mass, 1e-6f);
    EXPECT_FLOAT_NEAR(testBody.acceleration.y, force.y / testBody.mass, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(MovementPhysicsTest, ApplyForce_MultipleTimes)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 force1(50.0f, 0.0f);
    Nyon::Math::Vector2 force2(0.0f, 30.0f);
    
    MovementPhysics::ApplyForce(testBody, force1);
    MovementPhysics::ApplyForce(testBody, force2);
    
    LOG_VAR_DEBUG(testBody.acceleration.x);
    LOG_VAR_DEBUG(testBody.acceleration.y);
    // Forces should accumulate
    EXPECT_FLOAT_NEAR(testBody.acceleration.x, force1.x / testBody.mass, 1e-6f);
    EXPECT_FLOAT_NEAR(testBody.acceleration.y, force2.y / testBody.mass, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(MovementPhysicsTest, ApplyForce_StaticBody)
{
    LOG_FUNC_ENTER();
    testBody.isStatic = true;
    Nyon::Math::Vector2 force(100.0f, 100.0f);
    Nyon::Math::Vector2 initialAccel = testBody.acceleration;
    
    MovementPhysics::ApplyForce(testBody, force);
    
    LOG_VAR_DEBUG(testBody.acceleration.x);
    LOG_VAR_DEBUG(testBody.acceleration.y);
    // Static bodies should ignore forces
    EXPECT_VECTOR2_NEAR(testBody.acceleration, initialAccel, 1e-6f);
    LOG_FUNC_EXIT();
}

// Impulse tests
TEST_F(MovementPhysicsTest, ApplyImpulse_Basic)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 impulse(50.0f, -25.0f);
    Nyon::Math::Vector2 initialVelocity = testBody.velocity;
    
    MovementPhysics::ApplyImpulse(testBody, impulse);
    
    LOG_VAR_DEBUG(testBody.velocity.x);
    LOG_VAR_DEBUG(testBody.velocity.y);
    // Impulse should directly add to velocity
    EXPECT_FLOAT_NEAR(testBody.velocity.x, initialVelocity.x + impulse.x, 1e-6f);
    EXPECT_FLOAT_NEAR(testBody.velocity.y, initialVelocity.y + impulse.y, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(MovementPhysicsTest, ApplyImpulse_StaticBody)
{
    LOG_FUNC_ENTER();
    testBody.isStatic = true;
    Nyon::Math::Vector2 impulse(100.0f, 100.0f);
    Nyon::Math::Vector2 initialVelocity = testBody.velocity;
    
    MovementPhysics::ApplyImpulse(testBody, impulse);
    
    LOG_VAR_DEBUG(testBody.velocity.x);
    LOG_VAR_DEBUG(testBody.velocity.y);
    // Static bodies should ignore impulses
    EXPECT_VECTOR2_NEAR(testBody.velocity, initialVelocity, 1e-6f);
    LOG_FUNC_EXIT();
}

// Velocity control tests
TEST_F(MovementPhysicsTest, SetVelocity)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 newVelocity(150.0f, -75.0f);
    
    MovementPhysics::SetVelocity(testBody, newVelocity);
    
    LOG_VAR_DEBUG(testBody.velocity.x);
    LOG_VAR_DEBUG(testBody.velocity.y);
    EXPECT_VECTOR2_NEAR(testBody.velocity, newVelocity, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(MovementPhysicsTest, SetVelocity_StaticBody)
{
    LOG_FUNC_ENTER();
    testBody.isStatic = true;
    Nyon::Math::Vector2 newVelocity(100.0f, 100.0f);
    Nyon::Math::Vector2 initialVelocity = testBody.velocity;
    
    MovementPhysics::SetVelocity(testBody, newVelocity);
    
    LOG_VAR_DEBUG(testBody.velocity.x);
    LOG_VAR_DEBUG(testBody.velocity.y);
    // Static bodies should maintain zero velocity
    EXPECT_VECTOR2_NEAR(testBody.velocity, initialVelocity, 1e-6f);
    LOG_FUNC_EXIT();
}

// Speed calculation tests
TEST_F(MovementPhysicsTest, GetSpeed_ZeroVelocity)
{
    LOG_FUNC_ENTER();
    float speed = MovementPhysics::GetSpeed(testBody);
    
    LOG_VAR_DEBUG(speed);
    EXPECT_FLOAT_EQ(speed, 0.0f);
    LOG_FUNC_EXIT();
}

TEST_F(MovementPhysicsTest, GetSpeed_NonZeroVelocity)
{
    LOG_FUNC_ENTER();
    testBody.velocity.Set(3.0f, 4.0f); // 3-4-5 triangle
    float speed = MovementPhysics::GetSpeed(testBody);
    
    LOG_VAR_DEBUG(speed);
    EXPECT_FLOAT_NEAR(speed, 5.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(MovementPhysicsTest, GetSpeed_NegativeComponents)
{
    LOG_FUNC_ENTER();
    testBody.velocity.Set(-3.0f, -4.0f);
    float speed = MovementPhysics::GetSpeed(testBody);
    
    LOG_VAR_DEBUG(speed);
    EXPECT_FLOAT_NEAR(speed, 5.0f, 1e-6f); // Speed should always be positive
    LOG_FUNC_EXIT();
}

// Velocity angle tests
TEST_F(MovementPhysicsTest, GetVelocityAngle_Horizontal)
{
    LOG_FUNC_ENTER();
    testBody.velocity.Set(100.0f, 0.0f);
    float angle = MovementPhysics::GetVelocityAngle(testBody);
    
    LOG_VAR_DEBUG(angle);
    EXPECT_FLOAT_NEAR(angle, 0.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(MovementPhysicsTest, GetVelocityAngle_Vertical)
{
    LOG_FUNC_ENTER();
    testBody.velocity.Set(0.0f, 100.0f);
    float angle = MovementPhysics::GetVelocityAngle(testBody);
    
    LOG_VAR_DEBUG(angle);
    EXPECT_FLOAT_NEAR(angle, M_PI/2.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(MovementPhysicsTest, GetVelocityAngle_Diagonal)
{
    LOG_FUNC_ENTER();
    testBody.velocity.Set(100.0f, 100.0f);
    float angle = MovementPhysics::GetVelocityAngle(testBody);
    
    LOG_VAR_DEBUG(angle);
    EXPECT_FLOAT_NEAR(angle, M_PI/4.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

// Speed limiting tests
TEST_F(MovementPhysicsTest, LimitSpeed_BelowThreshold)
{
    LOG_FUNC_ENTER();
    testBody.velocity.Set(50.0f, 0.0f);
    float maxSpeed = 100.0f;
    Nyon::Math::Vector2 initialVelocity = testBody.velocity;
    
    MovementPhysics::LimitSpeed(testBody, maxSpeed);
    
    LOG_VAR_DEBUG(testBody.velocity.x);
    LOG_VAR_DEBUG(testBody.velocity.y);
    // Should remain unchanged when below threshold
    EXPECT_VECTOR2_NEAR(testBody.velocity, initialVelocity, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(MovementPhysicsTest, LimitSpeed_AboveThreshold)
{
    LOG_FUNC_ENTER();
    testBody.velocity.Set(150.0f, 0.0f);
    float maxSpeed = 100.0f;
    
    MovementPhysics::LimitSpeed(testBody, maxSpeed);
    
    LOG_VAR_DEBUG(testBody.velocity.Length());
    LOG_VAR_DEBUG(maxSpeed);
    // Should be clamped to max speed
    EXPECT_LE(testBody.velocity.Length(), maxSpeed + 1e-3f);
    // Direction should be preserved
    EXPECT_FLOAT_NEAR(testBody.velocity.y, 0.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(MovementPhysicsTest, LimitSpeed_ExactlyAtThreshold)
{
    LOG_FUNC_ENTER();
    testBody.velocity.Set(100.0f, 0.0f);
    float maxSpeed = 100.0f;
    Nyon::Math::Vector2 initialVelocity = testBody.velocity;
    
    MovementPhysics::LimitSpeed(testBody, maxSpeed);
    
    LOG_VAR_DEBUG(testBody.velocity.x);
    LOG_VAR_DEBUG(testBody.velocity.y);
    // Should remain unchanged when exactly at threshold
    EXPECT_VECTOR2_NEAR(testBody.velocity, initialVelocity, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(MovementPhysicsTest, LimitSpeed_ZeroMaxSpeed)
{
    LOG_FUNC_ENTER();
    testBody.velocity.Set(50.0f, 50.0f);
    float maxSpeed = 0.0f;
    
    MovementPhysics::LimitSpeed(testBody, maxSpeed);
    
    LOG_VAR_DEBUG(testBody.velocity.Length());
    // Should reduce to zero velocity
    EXPECT_FLOAT_NEAR(testBody.velocity.Length(), 0.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

// Combined operations tests
TEST_F(MovementPhysicsTest, ForceThenIntegration)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 force(100.0f, 0.0f);
    float deltaTime = 1.0f / 60.0f;
    
    MovementPhysics::ApplyForce(testBody, force);
    // Simulate integration step (manual for testing)
    testBody.velocity += testBody.acceleration * deltaTime;
    testBody.position += testBody.velocity * deltaTime;
    
    LOG_VAR_DEBUG(testBody.acceleration.x);
    LOG_VAR_DEBUG(testBody.velocity.x);
    LOG_VAR_DEBUG(testBody.position.x);
    
    float expectedAccel = force.x / testBody.mass;
    float expectedVel = expectedAccel * deltaTime;
    float expectedPos = 0.5f * expectedAccel * deltaTime * deltaTime; // Assuming starts from rest
    
    EXPECT_FLOAT_NEAR(testBody.acceleration.x, expectedAccel, 1e-6f);
    EXPECT_FLOAT_NEAR(testBody.velocity.x, expectedVel, 1e-6f);
    EXPECT_FLOAT_NEAR(testBody.position.x, expectedPos, 1e-6f);
    LOG_FUNC_EXIT();
}

// Edge cases
TEST_F(MovementPhysicsTest, ZeroMass)
{
    LOG_FUNC_ENTER();
    testBody.mass = 0.0f;
    Nyon::Math::Vector2 force(100.0f, 100.0f);
    
    // Should handle division by zero gracefully
    EXPECT_NO_THROW(MovementPhysics::ApplyForce(testBody, force));
    LOG_FUNC_EXIT();
}

TEST_F(MovementPhysicsTest, InfiniteValues)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 infiniteForce(INFINITY, INFINITY);
    
    // Should handle infinite values gracefully
    EXPECT_NO_THROW(MovementPhysics::ApplyForce(testBody, infiniteForce));
    LOG_FUNC_EXIT();
}