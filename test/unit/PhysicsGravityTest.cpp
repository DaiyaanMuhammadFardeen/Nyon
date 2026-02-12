#include <gtest/gtest.h>
#include "nyon/utils/GravityPhysics.h"
#include "nyon/utils/Physics.h"
#include "TestHelpers.h"

using namespace Nyon::Utils;

/**
 * @brief Unit tests for GravityPhysics utility functions.
 * 
 * Tests gravity simulation, sub-stepping behavior, friction, drag,
 * and velocity limiting with various edge cases and boundary conditions.
 */
class GravityPhysicsTest : public ::testing::Test
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
        testBody.mass = 1.0f;
        testBody.friction = 0.1f;
        testBody.drag = 0.01f;
        testBody.maxSpeed = 1000.0f;
        testBody.isStatic = false;
        LOG_FUNC_EXIT();
    }

    Physics::Body testBody;
};

// Basic gravity tests
TEST_F(GravityPhysicsTest, GravityConstant)
{
    LOG_FUNC_ENTER();
    LOG_VAR_DEBUG(GravityPhysics::Gravity);
    EXPECT_FLOAT_EQ(GravityPhysics::Gravity, 980.0f);
    LOG_FUNC_EXIT();
}

TEST_F(GravityPhysicsTest, FreeFall_Basic)
{
    LOG_FUNC_ENTER();
    NyonTest::PERF_TIMER("FreeFall_Basic");
    
    float deltaTime = 1.0f / 60.0f; // 60 FPS
    GravityPhysics::UpdateBody(testBody, deltaTime, false);
    
    LOG_VAR_DEBUG(testBody.velocity.y);
    LOG_VAR_DEBUG(testBody.position.y);
    
    // After 1/60th second, velocity should be gravity * deltaTime
    EXPECT_FLOAT_NEAR(testBody.velocity.y, GravityPhysics::Gravity * deltaTime, 1e-3f);
    // Position should be 0.5 * gravity * deltaTime^2 (kinematic equation)
    EXPECT_FLOAT_NEAR(testBody.position.y, 0.5f * GravityPhysics::Gravity * deltaTime * deltaTime, 1e-3f);
    LOG_FUNC_EXIT();
}

TEST_F(GravityPhysicsTest, FreeFall_MultipleSteps)
{
    LOG_FUNC_ENTER();
    NyonTest::PERF_TIMER("FreeFall_MultipleSteps");
    
    float deltaTime = 1.0f / 60.0f;
    float totalTime = 1.0f; // 1 second
    int steps = static_cast<int>(totalTime / deltaTime);
    
    for (int i = 0; i < steps; ++i) {
        GravityPhysics::UpdateBody(testBody, deltaTime, false);
    }
    
    LOG_VAR_DEBUG(testBody.velocity.y);
    LOG_VAR_DEBUG(testBody.position.y);
    
    // After 1 second of free fall
    EXPECT_FLOAT_NEAR(testBody.velocity.y, GravityPhysics::Gravity * totalTime, 1.0f);
    // Position: s = 0.5 * g * t^2
    EXPECT_FLOAT_NEAR(testBody.position.y, 0.5f * GravityPhysics::Gravity * totalTime * totalTime, 1.0f);
    LOG_FUNC_EXIT();
}

// Grounded behavior tests
TEST_F(GravityPhysicsTest, Grounded_NoVerticalMovement)
{
    LOG_FUNC_ENTER();
    testBody.velocity.y = 100.0f; // Moving downward
    
    GravityPhysics::UpdateBody(testBody, 1.0f / 60.0f, true); // Grounded = true
    
    LOG_VAR_DEBUG(testBody.velocity.y);
    // Should clamp vertical velocity when grounded
    EXPECT_LE(testBody.velocity.y, 0.0f);
    LOG_FUNC_EXIT();
}

TEST_F(GravityPhysicsTest, Grounded_FrictionApplied)
{
    LOG_FUNC_ENTER();
    testBody.velocity.x = 100.0f; // Moving horizontally
    float initialVelocity = testBody.velocity.x;
    
    GravityPhysics::UpdateBody(testBody, 1.0f / 60.0f, true); // Grounded = true
    
    LOG_VAR_DEBUG(testBody.velocity.x);
    LOG_VAR_DEBUG(initialVelocity);
    // Friction should reduce horizontal velocity
    EXPECT_LT(testBody.velocity.x, initialVelocity);
    EXPECT_GT(testBody.velocity.x, 0.0f); // Still positive but reduced
    LOG_FUNC_EXIT();
}

TEST_F(GravityPhysicsTest, Airborne_NoFriction)
{
    LOG_FUNC_ENTER();
    testBody.velocity.x = 100.0f; // Moving horizontally
    
    GravityPhysics::UpdateBody(testBody, 1.0f / 60.0f, false); // Not grounded
    
    LOG_VAR_DEBUG(testBody.velocity.x);
    // No friction when airborne, but drag should still apply
    EXPECT_LT(testBody.velocity.x, 100.0f);
    LOG_FUNC_EXIT();
}

// Drag and air resistance tests
TEST_F(GravityPhysicsTest, Drag_Application)
{
    LOG_FUNC_ENTER();
    testBody.drag = 0.5f; // High drag
    testBody.velocity.x = 100.0f;
    float initialVelocity = testBody.velocity.x;
    
    GravityPhysics::UpdateBody(testBody, 1.0f / 60.0f, false);
    
    LOG_VAR_DEBUG(testBody.velocity.x);
    LOG_VAR_DEBUG(initialVelocity);
    // High drag should significantly reduce velocity
    EXPECT_LT(testBody.velocity.x, initialVelocity * 0.9f);
    LOG_FUNC_EXIT();
}

TEST_F(GravityPhysicsTest, Drag_Limiting)
{
    LOG_FUNC_ENTER();
    testBody.drag = 1.0f; // Maximum drag
    testBody.velocity.Set(1000.0f, 0.0f);
    
    GravityPhysics::UpdateBody(testBody, 1.0f / 60.0f, false);
    
    LOG_VAR_DEBUG(testBody.velocity.x);
    // With drag = 1.0, velocity should be nearly eliminated
    EXPECT_LT(testBody.velocity.x, 100.0f);
    LOG_FUNC_EXIT();
}

// Velocity limiting tests
TEST_F(GravityPhysicsTest, VelocityLimiting_MaxSpeed)
{
    LOG_FUNC_ENTER();
    testBody.maxSpeed = 100.0f;
    testBody.velocity.Set(200.0f, 200.0f); // Exceeds max speed
    
    GravityPhysics::UpdateBody(testBody, 1.0f / 60.0f, false);
    
    LOG_VAR_DEBUG(testBody.velocity.Length());
    LOG_VAR_DEBUG(testBody.maxSpeed);
    // Should be clamped to max speed
    EXPECT_LE(testBody.velocity.Length(), testBody.maxSpeed + 1e-3f);
    LOG_FUNC_EXIT();
}

TEST_F(GravityPhysicsTest, VelocityLimiting_NormalSpeed)
{
    LOG_FUNC_ENTER();
    testBody.maxSpeed = 1000.0f;
    testBody.velocity.Set(100.0f, 50.0f); // Within limits
    
    GravityPhysics::UpdateBody(testBody, 1.0f / 60.0f, false);
    
    LOG_VAR_DEBUG(testBody.velocity.Length());
    LOG_VAR_DEBUG(testBody.maxSpeed);
    // Should not be affected by limiting
    EXPECT_FLOAT_NEAR(testBody.velocity.x, 100.0f, 1.0f);
    EXPECT_FLOAT_NEAR(testBody.velocity.y, 50.0f + GravityPhysics::Gravity/60.0f, 1.0f);
    LOG_FUNC_EXIT();
}

// Static body tests
TEST_F(GravityPhysicsTest, StaticBody_NoMovement)
{
    LOG_FUNC_ENTER();
    testBody.isStatic = true;
    testBody.velocity.Set(100.0f, 100.0f);
    testBody.acceleration.Set(50.0f, 50.0f);
    
    GravityPhysics::UpdateBody(testBody, 1.0f, false);
    
    LOG_VAR_DEBUG(testBody.velocity.x);
    LOG_VAR_DEBUG(testBody.velocity.y);
    LOG_VAR_DEBUG(testBody.acceleration.x);
    LOG_VAR_DEBUG(testBody.acceleration.y);
    // Static bodies should have zero velocity and acceleration
    EXPECT_FLOAT_EQ(testBody.velocity.x, 0.0f);
    EXPECT_FLOAT_EQ(testBody.velocity.y, 0.0f);
    EXPECT_FLOAT_EQ(testBody.acceleration.x, 0.0f);
    EXPECT_FLOAT_EQ(testBody.acceleration.y, 0.0f);
    LOG_FUNC_EXIT();
}

// Sub-stepping tests
TEST_F(GravityPhysicsTest, SubStepping_LargeDeltaTime)
{
    LOG_FUNC_ENTER();
    NyonTest::PERF_TIMER("SubStepping_LargeDeltaTime");
    
    float largeDeltaTime = 0.1f; // 100ms - larger than typical frame
    testBody.velocity.Set(0.0f, 0.0f);
    
    GravityPhysics::UpdateBody(testBody, largeDeltaTime, false);
    
    LOG_VAR_DEBUG(testBody.velocity.y);
    LOG_VAR_DEBUG(testBody.position.y);
    // Should still produce reasonable results through sub-stepping
    EXPECT_GT(testBody.velocity.y, 0.0f);
    EXPECT_GT(testBody.position.y, 0.0f);
    LOG_FUNC_EXIT();
}

// Edge cases
TEST_F(GravityPhysicsTest, ZeroDeltaTime)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 initialVelocity = testBody.velocity;
    Nyon::Math::Vector2 initialPosition = testBody.position;
    
    GravityPhysics::UpdateBody(testBody, 0.0f, false);
    
    LOG_VAR_DEBUG(testBody.velocity.x);
    LOG_VAR_DEBUG(testBody.velocity.y);
    LOG_VAR_DEBUG(testBody.position.x);
    LOG_VAR_DEBUG(testBody.position.y);
    // Zero deltaTime should result in no change
    EXPECT_VECTOR2_NEAR(testBody.velocity, initialVelocity, 1e-6f);
    EXPECT_VECTOR2_NEAR(testBody.position, initialPosition, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(GravityPhysicsTest, NegativeDeltaTime)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 initialVelocity = testBody.velocity;
    Nyon::Math::Vector2 initialPosition = testBody.position;
    
    GravityPhysics::UpdateBody(testBody, -0.1f, false);
    
    LOG_VAR_DEBUG(testBody.velocity.x);
    LOG_VAR_DEBUG(testBody.velocity.y);
    LOG_VAR_DEBUG(testBody.position.x);
    LOG_VAR_DEBUG(testBody.position.y);
    // Negative deltaTime should be handled gracefully (no change)
    EXPECT_VECTOR2_NEAR(testBody.velocity, initialVelocity, 1e-6f);
    EXPECT_VECTOR2_NEAR(testBody.position, initialPosition, 1e-6f);
    LOG_FUNC_EXIT();
}