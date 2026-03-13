#include <gtest/gtest.h>
#include "TestHelpers.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"

using namespace Nyon::ECS;

/**
 * @brief Comprehensive unit tests for PhysicsWorldComponent.
 * 
 * Tests cover all core functionalities including:
 * - Construction and initialization
 * - Simulation settings configuration
 * - Position correction parameters
 * - Sleep and damping parameters
 * - Global control methods
 * - Contact manifold management
 * - Event callbacks
 * - Debug features
 * - Performance counters and profiling
 */

// ============================================================================
// CONSTRUCTION AND INITIALIZATION TESTS
// ============================================================================

TEST(PhysicsWorldComponentTest, DefaultConstructor)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Simulation settings
    EXPECT_VECTOR2_NEAR(world.gravity, Nyon::Math::Vector2(0.0f, 980.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(world.timeStep, 1.0f / 60.0f, 1e-5f);
    EXPECT_EQ(world.velocityIterations, 8);
    EXPECT_EQ(world.positionIterations, 3);
    EXPECT_EQ(world.subStepCount, 4);
    
    // Position correction parameters
    EXPECT_FLOAT_NEAR(world.baumgarteBeta, 0.2f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.linearSlop, 0.005f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.maxLinearCorrection, 0.2f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.maxAngularCorrection, 0.1f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.maxPenetrationCorrection, 50.0f, 1e-5f);
    
    // Sleep parameters
    EXPECT_FLOAT_NEAR(world.sleepLinearThreshold, 0.01f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.sleepAngularThreshold, 0.01f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.sleepTimeThreshold, 0.5f, 1e-5f);
    
    // Global parameters
    EXPECT_FLOAT_NEAR(world.restitutionThreshold, 100.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.maxLinearSpeed, 10000.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.maxAngularSpeed, 100.0f, 1e-5f);
    EXPECT_TRUE(world.enableSleep);
    EXPECT_TRUE(world.enableWarmStarting);
    EXPECT_TRUE(world.enableContinuous);
    EXPECT_TRUE(world.enableSpeculative);
    
    // Contact tuning
    EXPECT_FLOAT_NEAR(world.contactHertz, 30.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.contactDampingRatio, 1.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.contactPushSpeed, 10.0f, 1e-5f);
    
    // Debugging flags
    EXPECT_FALSE(world.drawShapes);
    EXPECT_FALSE(world.drawJoints);
    EXPECT_FALSE(world.drawAABBs);
    EXPECT_FALSE(world.drawContacts);
    EXPECT_FALSE(world.drawIslands);
    
    // Contact manifolds should be empty
    EXPECT_TRUE(world.contactManifolds.empty());
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// SIMULATION SETTINGS TESTS
// ============================================================================

TEST(PhysicsWorldComponentTest, SetGravity)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    Nyon::Math::Vector2 newGravity(0.0f, 500.0f);
    
    world.SetGravity(newGravity);
    
    EXPECT_VECTOR2_NEAR(world.gravity, newGravity, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SetGravityNegative)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    Nyon::Math::Vector2 newGravity(0.0f, -980.0f);  // Y-up coordinate system
    
    world.SetGravity(newGravity);
    
    EXPECT_VECTOR2_NEAR(world.gravity, newGravity, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SetTimeStep)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetTimeStep(1.0f / 120.0f);  // 120 Hz
    
    EXPECT_FLOAT_NEAR(world.timeStep, 1.0f / 120.0f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SetTimeStepVariable)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetTimeStep(0.016f);  // ~60 Hz
    
    EXPECT_FLOAT_NEAR(world.timeStep, 0.016f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SetIterations)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetIterations(16, 8);  // Higher iterations for accuracy
    
    EXPECT_EQ(world.velocityIterations, 16);
    EXPECT_EQ(world.positionIterations, 8);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SetIterationsLow)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetIterations(1, 1);  // Minimum iterations (fast but inaccurate)
    
    EXPECT_EQ(world.velocityIterations, 1);
    EXPECT_EQ(world.positionIterations, 1);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SetSubSteps)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetSubSteps(8);  // High sub-stepping for stability
    
    EXPECT_EQ(world.subStepCount, 8);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SetSubStepsSingle)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetSubSteps(1);  // No sub-stepping
    
    EXPECT_EQ(world.subStepCount, 1);
    LOG_FUNC_EXIT();
}

// ============================================================================
// POSITION CORRECTION TESTS
// ============================================================================

TEST(PhysicsWorldComponentTest, BaumgarteStabilization)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.baumgarteBeta = 0.1f;  // Softer correction
    
    EXPECT_FLOAT_NEAR(world.baumgarteBeta, 0.1f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, LinearSlop)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.linearSlop = 0.01f;  // More tolerance
    
    EXPECT_FLOAT_NEAR(world.linearSlop, 0.01f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, MaxCorrections)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.maxLinearCorrection = 0.5f;
    world.maxAngularCorrection = 0.3f;
    
    EXPECT_FLOAT_NEAR(world.maxLinearCorrection, 0.5f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.maxAngularCorrection, 0.3f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, MaxPenetrationCorrection)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.maxPenetrationCorrection = 100.0f;  // Aggressive penetration recovery
    
    EXPECT_FLOAT_NEAR(world.maxPenetrationCorrection, 100.0f, 1e-5f);
    LOG_FUNC_EXIT();
}

// ============================================================================
// SLEEP PARAMETERS TESTS
// ============================================================================

TEST(PhysicsWorldComponentTest, SleepThresholds)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.sleepLinearThreshold = 0.001f;  // Very sensitive
    world.sleepAngularThreshold = 0.001f;
    
    EXPECT_FLOAT_NEAR(world.sleepLinearThreshold, 0.001f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.sleepAngularThreshold, 0.001f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SleepTimeThreshold)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.sleepTimeThreshold = 1.0f;  // Wait longer before sleeping
    
    EXPECT_FLOAT_NEAR(world.sleepTimeThreshold, 1.0f, 1e-5f);
    LOG_FUNC_EXIT();
}

// ============================================================================
// GLOBAL CONTROL TESTS
// ============================================================================

TEST(PhysicsWorldComponentTest, EnableSleeping)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.EnableSleeping(false);
    EXPECT_FALSE(world.enableSleep);
    EXPECT_FALSE(world.IsSleepingEnabled());
    
    world.EnableSleeping(true);
    EXPECT_TRUE(world.enableSleep);
    EXPECT_TRUE(world.IsSleepingEnabled());
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, EnableWarmStarting)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.EnableWarmStarting(false);
    EXPECT_FALSE(world.enableWarmStarting);
    EXPECT_FALSE(world.IsWarmStartingEnabled());
    
    world.EnableWarmStarting(true);
    EXPECT_TRUE(world.enableWarmStarting);
    EXPECT_TRUE(world.IsWarmStartingEnabled());
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, EnableContinuous)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.EnableContinuous(false);
    EXPECT_FALSE(world.enableContinuous);
    EXPECT_FALSE(world.IsContinuousEnabled());
    
    world.EnableContinuous(true);
    EXPECT_TRUE(world.enableContinuous);
    EXPECT_TRUE(world.IsContinuousEnabled());
    LOG_FUNC_EXIT();
}

// ============================================================================
// CONTACT TUNING TESTS
// ============================================================================

TEST(PhysicsWorldComponentTest, SetContactTuning)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetContactTuning(60.0f, 0.7f, 20.0f);
    
    EXPECT_FLOAT_NEAR(world.contactHertz, 60.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.contactDampingRatio, 0.7f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.contactPushSpeed, 20.0f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SetContactTuningSoft)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Soft contacts (like rubber)
    world.SetContactTuning(10.0f, 0.5f, 5.0f);
    
    EXPECT_FLOAT_NEAR(world.contactHertz, 10.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.contactDampingRatio, 0.5f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.contactPushSpeed, 5.0f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SetContactTuningHard)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Hard contacts (like steel)
    world.SetContactTuning(100.0f, 1.0f, 50.0f);
    
    EXPECT_FLOAT_NEAR(world.contactHertz, 100.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.contactDampingRatio, 1.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.contactPushSpeed, 50.0f, 1e-5f);
    LOG_FUNC_EXIT();
}

// ============================================================================
// UTILITY METHODS TESTS
// ============================================================================

TEST(PhysicsWorldComponentTest, GetInvTimeStep)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetTimeStep(1.0f / 60.0f);
    float invDt = world.GetInvTimeStep();
    
    EXPECT_FLOAT_NEAR(invDt, 60.0f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, GetInvTimeStepZero)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetTimeStep(0.0f);  // Paused
    float invDt = world.GetInvTimeStep();
    
    EXPECT_FLOAT_NEAR(invDt, 0.0f, 1e-5f);  // Should handle division by zero
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, GetInvTimeStepSmall)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetTimeStep(0.001f);  // Very small timestep
    float invDt = world.GetInvTimeStep();
    
    EXPECT_FLOAT_NEAR(invDt, 1000.0f, 1e-5f);
    LOG_FUNC_EXIT();
}

// ============================================================================
// DEBUG FEATURES TESTS
// ============================================================================

TEST(PhysicsWorldComponentTest, SetDebugDrawAll)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetDebugDraw(true, true, true, true, true);
    
    EXPECT_TRUE(world.drawShapes);
    EXPECT_TRUE(world.drawJoints);
    EXPECT_TRUE(world.drawAABBs);
    EXPECT_TRUE(world.drawContacts);
    EXPECT_TRUE(world.drawIslands);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SetDebugDrawNone)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Start with all enabled
    world.drawShapes = true;
    world.drawJoints = true;
    world.drawAABBs = true;
    world.drawContacts = true;
    world.drawIslands = true;
    
    world.SetDebugDraw(false, false, false, false, false);
    
    EXPECT_FALSE(world.drawShapes);
    EXPECT_FALSE(world.drawJoints);
    EXPECT_FALSE(world.drawAABBs);
    EXPECT_FALSE(world.drawContacts);
    EXPECT_FALSE(world.drawIslands);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SetDebugDrawSelective)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Enable only collision visualization
    world.SetDebugDraw(false, false, true, true, false);
    
    EXPECT_FALSE(world.drawShapes);
    EXPECT_FALSE(world.drawJoints);
    EXPECT_TRUE(world.drawAABBs);
    EXPECT_TRUE(world.drawContacts);
    EXPECT_FALSE(world.drawIslands);
    LOG_FUNC_EXIT();
}

// ============================================================================
// PERFORMANCE COUNTERS TESTS
// ============================================================================

TEST(PhysicsWorldComponentTest, ProfileDefaultValues)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // All profile times should start at 0
    EXPECT_FLOAT_NEAR(world.profile.broadPhaseTime, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.profile.narrowPhaseTime, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.profile.solverTime, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.profile.islandTime, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.profile.totalTime, 0.0f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, CountersDefaultValues)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // All counters should start at 0
    EXPECT_EQ(world.counters.bodyCount, 0);
    EXPECT_EQ(world.counters.awakeBodyCount, 0);
    EXPECT_EQ(world.counters.contactCount, 0);
    EXPECT_EQ(world.counters.jointCount, 0);
    EXPECT_EQ(world.counters.islandCount, 0);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, GetProfile)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Simulate some profile data
    world.profile.broadPhaseTime = 0.001f;
    world.profile.narrowPhaseTime = 0.002f;
    world.profile.solverTime = 0.005f;
    world.profile.islandTime = 0.001f;
    world.profile.totalTime = 0.009f;
    
    const auto& profile = world.GetProfile();
    
    EXPECT_FLOAT_NEAR(profile.broadPhaseTime, 0.001f, 1e-5f);
    EXPECT_FLOAT_NEAR(profile.narrowPhaseTime, 0.002f, 1e-5f);
    EXPECT_FLOAT_NEAR(profile.solverTime, 0.005f, 1e-5f);
    EXPECT_FLOAT_NEAR(profile.islandTime, 0.001f, 1e-5f);
    EXPECT_FLOAT_NEAR(profile.totalTime, 0.009f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, GetCounters)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Simulate some counter data
    world.counters.bodyCount = 100;
    world.counters.awakeBodyCount = 50;
    world.counters.contactCount = 75;
    world.counters.jointCount = 25;
    world.counters.islandCount = 5;
    
    const auto& counters = world.GetCounters();
    
    EXPECT_EQ(counters.bodyCount, 100);
    EXPECT_EQ(counters.awakeBodyCount, 50);
    EXPECT_EQ(counters.contactCount, 75);
    EXPECT_EQ(counters.jointCount, 25);
    EXPECT_EQ(counters.islandCount, 5);
    LOG_FUNC_EXIT();
}

// ============================================================================
// CONTACT MANIFOLD TESTS
// ============================================================================

TEST(PhysicsWorldComponentTest, ContactManifoldEmpty)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    EXPECT_TRUE(world.contactManifolds.empty());
    EXPECT_EQ(world.contactManifolds.size(), 0);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, ContactManifoldAddRemove)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Add contact manifolds
    world.contactManifolds.emplace_back();
    world.contactManifolds.emplace_back();
    
    EXPECT_EQ(world.contactManifolds.size(), 2);
    
    // Clear contacts
    world.contactManifolds.clear();
    
    EXPECT_TRUE(world.contactManifolds.empty());
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, ContactManifoldInitialization)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.contactManifolds.emplace_back();
    ContactManifold& manifold = world.contactManifolds.back();
    
    // Check default initialization
    EXPECT_TRUE(manifold.points.empty());
    EXPECT_VECTOR2_NEAR(manifold.normal, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_VECTOR2_NEAR(manifold.localNormal, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_VECTOR2_NEAR(manifold.localPoint, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(manifold.friction, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(manifold.restitution, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(manifold.tangentSpeed, 0.0f, 1e-5f);
    EXPECT_EQ(manifold.entityIdA, 0);
    EXPECT_EQ(manifold.entityIdB, 0);
    EXPECT_EQ(manifold.shapeIdA, 0);
    EXPECT_EQ(manifold.shapeIdB, 0);
    EXPECT_FALSE(manifold.touching);
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// CONTACT POINT TESTS
// ============================================================================

TEST(PhysicsWorldComponentTest, ContactPointDefaultConstruction)
{
    LOG_FUNC_ENTER();
    ContactPoint point;
    
    EXPECT_VECTOR2_NEAR(point.position, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_VECTOR2_NEAR(point.normal, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(point.separation, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(point.normalImpulse, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(point.tangentImpulse, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(point.normalMass, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(point.tangentMass, 0.0f, 1e-5f);
    EXPECT_EQ(point.featureId, 0);
    EXPECT_FALSE(point.persisted);
    
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, ContactPointInitialization)
{
    LOG_FUNC_ENTER();
    ContactPoint point;
    point.position = Nyon::Math::Vector2(10.0f, 20.0f);
    point.normal = Nyon::Math::Vector2(0.0f, -1.0f);
    point.separation = -0.01f;  // Penetration
    point.normalImpulse = 5.0f;
    point.tangentImpulse = 2.0f;
    point.featureId = 12345;
    point.persisted = true;
    
    EXPECT_VECTOR2_NEAR(point.position, Nyon::Math::Vector2(10.0f, 20.0f), 1e-5f);
    EXPECT_VECTOR2_NEAR(point.normal, Nyon::Math::Vector2(0.0f, -1.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(point.separation, -0.01f, 1e-5f);
    EXPECT_FLOAT_NEAR(point.normalImpulse, 5.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(point.tangentImpulse, 2.0f, 1e-5f);
    EXPECT_EQ(point.featureId, 12345);
    EXPECT_TRUE(point.persisted);
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// EVENT CALLBACK TESTS
// ============================================================================

TEST(PhysicsWorldComponentTest, BeginContactCallback)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    bool callbackCalled = false;
    uint32_t capturedA = 0, capturedB = 0;
    
    world.SetBeginContactCallback([&](uint32_t a, uint32_t b) {
        callbackCalled = true;
        capturedA = a;
        capturedB = b;
    });
    
    // Trigger callback
    if (world.callbacks.beginContact) {
        world.callbacks.beginContact(100, 200);
    }
    
    EXPECT_TRUE(callbackCalled);
    EXPECT_EQ(capturedA, 100);
    EXPECT_EQ(capturedB, 200);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, EndContactCallback)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    bool callbackCalled = false;
    
    world.SetEndContactCallback([&](uint32_t, uint32_t) {
        callbackCalled = true;
    });
    
    if (world.callbacks.endContact) {
        world.callbacks.endContact(1, 2);
    }
    
    EXPECT_TRUE(callbackCalled);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, PreSolveCallback)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    float capturedImpulse = 0.0f;
    
    world.SetPreSolveCallback([&](uint32_t, uint32_t, float impulse) {
        capturedImpulse = impulse;
    });
    
    if (world.callbacks.preSolve) {
        world.callbacks.preSolve(1, 2, 15.5f);
    }
    
    EXPECT_FLOAT_NEAR(capturedImpulse, 15.5f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, PostSolveCallback)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    float capturedImpulse = 0.0f;
    
    world.SetPostSolveCallback([&](uint32_t, uint32_t, float impulse) {
        capturedImpulse = impulse;
    });
    
    if (world.callbacks.postSolve) {
        world.callbacks.postSolve(1, 2, 25.5f);
    }
    
    EXPECT_FLOAT_NEAR(capturedImpulse, 25.5f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, JointBreakCallback)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    float capturedForce = 0.0f;
    float capturedTorque = 0.0f;
    
    world.SetJointBreakCallback([&](uint32_t, float force, float torque) {
        capturedForce = force;
        capturedTorque = torque;
    });
    
    if (world.callbacks.jointBreak) {
        world.callbacks.jointBreak(42, 100.0f, 50.0f);
    }
    
    EXPECT_FLOAT_NEAR(capturedForce, 100.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(capturedTorque, 50.0f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SensorCallbacks)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    int sensorBeginCount = 0;
    int sensorEndCount = 0;
    
    world.SetSensorBeginCallback([&](uint32_t, uint32_t) {
        sensorBeginCount++;
    });
    
    world.SetSensorEndCallback([&](uint32_t, uint32_t) {
        sensorEndCount++;
    });
    
    if (world.callbacks.sensorBegin) {
        world.callbacks.sensorBegin(1, 2);
    }
    
    if (world.callbacks.sensorEnd) {
        world.callbacks.sensorEnd(1, 2);
    }
    
    EXPECT_EQ(sensorBeginCount, 1);
    EXPECT_EQ(sensorEndCount, 1);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, MultipleCallbacks)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    int callback1Count = 0;
    int callback2Count = 0;
    
    // Set first callback
    world.SetBeginContactCallback([&](uint32_t, uint32_t) {
        callback1Count++;
    });
    
    if (world.callbacks.beginContact) {
        world.callbacks.beginContact(1, 2);
    }
    
    EXPECT_EQ(callback1Count, 1);
    
    // Replace with second callback
    world.SetBeginContactCallback([&](uint32_t, uint32_t) {
        callback2Count++;
    });
    
    if (world.callbacks.beginContact) {
        world.callbacks.beginContact(3, 4);
    }
    
    EXPECT_EQ(callback1Count, 1);  // Old callback not called
    EXPECT_EQ(callback2Count, 1);  // New callback called
    LOG_FUNC_EXIT();
}

// ============================================================================
// COMPREHENSIVE SCENARIO TESTS
// ============================================================================

TEST(PhysicsWorldComponentTest, PlatformerGameSetup)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Typical 2D platformer settings
    world.SetGravity(Nyon::Math::Vector2(0.0f, 980.0f));  // Earth gravity
    world.SetTimeStep(1.0f / 60.0f);
    world.SetIterations(8, 3);
    world.SetSubSteps(4);
    
    // Enable sleep for performance
    world.EnableSleeping(true);
    
    // Enable continuous collision for player character
    world.EnableContinuous(true);
    
    EXPECT_VECTOR2_NEAR(world.gravity, Nyon::Math::Vector2(0.0f, 980.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(world.timeStep, 1.0f / 60.0f, 1e-5f);
    EXPECT_EQ(world.velocityIterations, 8);
    EXPECT_EQ(world.positionIterations, 3);
    EXPECT_TRUE(world.enableSleep);
    EXPECT_TRUE(world.enableContinuous);
    
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, TopDownGameSetup)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Top-down game (no gravity)
    world.SetGravity(Nyon::Math::Vector2(0.0f, 0.0f));
    world.SetTimeStep(1.0f / 60.0f);
    world.SetIterations(4, 2);  // Lower iterations OK for top-down
    
    EXPECT_VECTOR2_NEAR(world.gravity, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_EQ(world.velocityIterations, 4);
    EXPECT_EQ(world.positionIterations, 2);
    
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, HighPrecisionSimulation)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // High precision settings for accurate simulation
    world.SetTimeStep(1.0f / 120.0f);  // 120 Hz
    world.SetIterations(16, 8);  // Many iterations
    world.SetSubSteps(8);  // High sub-stepping
    
    // Strict position correction
    world.baumgarteBeta = 0.2f;
    world.linearSlop = 0.001f;
    
    EXPECT_FLOAT_NEAR(world.timeStep, 1.0f / 120.0f, 1e-5f);
    EXPECT_EQ(world.velocityIterations, 16);
    EXPECT_EQ(world.positionIterations, 8);
    EXPECT_EQ(world.subStepCount, 8);
    EXPECT_FLOAT_NEAR(world.linearSlop, 0.001f, 1e-5f);
    
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, FastPacedArcadeGame)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Arcade game - prioritize speed over accuracy
    world.SetTimeStep(1.0f / 60.0f);
    world.SetIterations(4, 2);  // Few iterations
    world.SetSubSteps(1);  // No sub-stepping
    
    // Disable sleep for instant response
    world.EnableSleeping(false);
    
    // Disable CCD for performance
    world.EnableContinuous(false);
    
    EXPECT_EQ(world.velocityIterations, 4);
    EXPECT_EQ(world.positionIterations, 2);
    EXPECT_EQ(world.subStepCount, 1);
    EXPECT_FALSE(world.enableSleep);
    EXPECT_FALSE(world.enableContinuous);
    
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, DebugVisualizationSetup)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Enable all debug visualization
    world.SetDebugDraw(true, true, true, true, true);
    
    // Verify all flags are set
    EXPECT_TRUE(world.drawShapes);
    EXPECT_TRUE(world.drawJoints);
    EXPECT_TRUE(world.drawAABBs);
    EXPECT_TRUE(world.drawContacts);
    EXPECT_TRUE(world.drawIslands);
    
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, ContactTuningScenario)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Bouncy ball scenario
    world.contactHertz = 30.0f;
    world.contactDampingRatio = 0.3f;  // Underdamped for bounce
    world.contactPushSpeed = 10.0f;
    world.restitutionThreshold = 1.0f;  // Low threshold for bounce
    
    EXPECT_FLOAT_NEAR(world.contactHertz, 30.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.contactDampingRatio, 0.3f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.contactPushSpeed, 10.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.restitutionThreshold, 1.0f, 1e-5f);
    
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, SimulationProfileTracking)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Simulate a physics step profile
    world.profile.broadPhaseTime = 0.002f;
    world.profile.narrowPhaseTime = 0.003f;
    world.profile.solverTime = 0.008f;
    world.profile.islandTime = 0.001f;
    world.profile.totalTime = 0.014f;
    
    // Verify total is sum of parts (approximately)
    float sum = world.profile.broadPhaseTime + 
                world.profile.narrowPhaseTime + 
                world.profile.solverTime + 
                world.profile.islandTime;
    
    EXPECT_FLOAT_NEAR(sum, world.profile.totalTime, 0.001f);
    
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, CounterManagement)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Simulate adding bodies and contacts
    world.counters.bodyCount = 50;
    world.counters.awakeBodyCount = 30;
    world.counters.contactCount = 40;
    world.counters.jointCount = 10;
    world.counters.islandCount = 3;
    
    // Verify counters via accessor
    const auto& counters = world.GetCounters();
    EXPECT_EQ(counters.bodyCount, 50);
    EXPECT_EQ(counters.awakeBodyCount, 30);
    EXPECT_EQ(counters.contactCount, 40);
    EXPECT_EQ(counters.jointCount, 10);
    EXPECT_EQ(counters.islandCount, 3);
    
    // Simulate bodies going to sleep
    world.counters.awakeBodyCount = 10;
    EXPECT_EQ(world.counters.awakeBodyCount, 10);
    
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, FullConfigurationCycle)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Start with defaults
    EXPECT_VECTOR2_NEAR(world.gravity, Nyon::Math::Vector2(0.0f, 980.0f), 1e-5f);
    EXPECT_TRUE(world.enableSleep);
    
    // Change configuration
    world.SetGravity(Nyon::Math::Vector2(0.0f, 0.0f));
    world.EnableSleeping(false);
    world.EnableContinuous(false);
    world.SetTimeStep(1.0f / 120.0f);
    
    // Verify changes
    EXPECT_VECTOR2_NEAR(world.gravity, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_FALSE(world.enableSleep);
    EXPECT_FALSE(world.enableContinuous);
    EXPECT_FLOAT_NEAR(world.timeStep, 1.0f / 120.0f, 1e-5f);
    
    // Restore defaults
    world.SetGravity(Nyon::Math::Vector2(0.0f, 980.0f));
    world.EnableSleeping(true);
    world.EnableContinuous(true);
    world.SetTimeStep(1.0f / 60.0f);
    
    // Verify restoration
    EXPECT_VECTOR2_NEAR(world.gravity, Nyon::Math::Vector2(0.0f, 980.0f), 1e-5f);
    EXPECT_TRUE(world.enableSleep);
    EXPECT_TRUE(world.enableContinuous);
    EXPECT_FLOAT_NEAR(world.timeStep, 1.0f / 60.0f, 1e-5f);
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// EDGE CASES AND BOUNDARY CONDITIONS
// ============================================================================

TEST(PhysicsWorldComponentTest, ZeroTimeStep)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetTimeStep(0.0f);
    
    EXPECT_FLOAT_NEAR(world.timeStep, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.GetInvTimeStep(), 0.0f, 1e-5f);  // Should handle gracefully
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, VeryLargeTimeStep)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetTimeStep(1.0f);  // 1 second timestep (very large)
    
    EXPECT_FLOAT_NEAR(world.timeStep, 1.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(world.GetInvTimeStep(), 1.0f, 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, NegativeGravity)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetGravity(Nyon::Math::Vector2(0.0f, -980.0f));  // Y-up system
    
    EXPECT_VECTOR2_NEAR(world.gravity, Nyon::Math::Vector2(0.0f, -980.0f), 1e-5f);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, ZeroIterations)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    world.SetIterations(0, 0);  // No solver iterations (unstable but valid)
    
    EXPECT_EQ(world.velocityIterations, 0);
    EXPECT_EQ(world.positionIterations, 0);
    LOG_FUNC_EXIT();
}

TEST(PhysicsWorldComponentTest, ContactManifoldReserve)
{
    LOG_FUNC_ENTER();
    PhysicsWorldComponent world;
    
    // Reserve space for many contacts
    world.contactManifolds.reserve(100);
    
    EXPECT_GE(world.contactManifolds.capacity(), 100);
    EXPECT_EQ(world.contactManifolds.size(), 0);  // Size unchanged
    
    LOG_FUNC_EXIT();
}
