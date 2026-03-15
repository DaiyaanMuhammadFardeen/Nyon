#include <gtest/gtest.h>
#include "TestHelpers.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
using namespace Nyon::ECS;
/**
 * @brief Comprehensive unit tests for PhysicsBodyComponent.
 * 
 * Tests cover all core functionalities including:
 * - Construction and initialization
 * - Mass properties management
 * - Force and torque application
 * - Impulse application
 * - Sleep mechanism
 * - Body type behaviors
 * - Motion locks
 * - Backward compatibility features
 */
TEST(PhysicsBodyComponentTest, DefaultConstructor)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    EXPECT_VECTOR2_NEAR(body.velocity, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_VECTOR2_NEAR(body.acceleration, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_VECTOR2_NEAR(body.force, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(body.angularVelocity, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.angularAcceleration, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.torque, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.mass, 1.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inverseMass, 1.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inertia, 1.0f, 1e-5f);   
    EXPECT_FLOAT_NEAR(body.inverseInertia, 1.0f, 1e-5f);
    EXPECT_VECTOR2_NEAR(body.centerOfMass, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(body.friction, 0.1f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.restitution, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.angularDamping, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.drag, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.maxLinearSpeed, 1000.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.maxAngularSpeed, 100.0f, 1e-5f);
    EXPECT_FALSE(body.isStatic);
    EXPECT_FALSE(body.isKinematic);
    EXPECT_FALSE(body.isBullet);
    EXPECT_TRUE(body.isAwake);
    EXPECT_TRUE(body.allowSleep);
    EXPECT_FLOAT_NEAR(body.sleepTimer, 0.0f, 1e-5f);
    EXPECT_FALSE(body.isGrounded);
    EXPECT_EQ(body.groundedFrames, 0);
    EXPECT_FALSE(body.motionLocks.lockTranslationX);
    EXPECT_FALSE(body.motionLocks.lockTranslationY);
    EXPECT_FALSE(body.motionLocks.lockRotation);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ConstructorWithMass)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body(5.0f);
    EXPECT_FLOAT_NEAR(body.mass, 5.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inverseMass, 0.2f, 1e-5f);
    EXPECT_FALSE(body.isStatic);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ConstructorWithMassAndStaticFlag)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body(10.0f, true);
    EXPECT_FLOAT_NEAR(body.mass, 10.0f, 1e-5f);
    EXPECT_TRUE(body.isStatic);
    EXPECT_FLOAT_NEAR(body.inverseMass, 0.0f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, SetMassDynamicBody)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.SetMass(10.0f);
    EXPECT_FLOAT_NEAR(body.mass, 10.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inverseMass, 0.1f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, UpdateMassPropertiesStaticBody)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isStatic = true;
    body.mass = 5.0f;
    body.UpdateMassProperties();
    EXPECT_FLOAT_NEAR(body.mass, 5.0f, 1e-5f);   
    EXPECT_FLOAT_NEAR(body.inverseMass, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inverseInertia, 0.0f, 1e-5f);   
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, UpdateMassPropertiesKinematicBody)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isKinematic = true;
    body.mass = 5.0f;
    body.UpdateMassProperties();
    EXPECT_FLOAT_NEAR(body.mass, 5.0f, 1e-5f);   
    EXPECT_FLOAT_NEAR(body.inverseMass, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inverseInertia, 0.0f, 1e-5f);   
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, UpdateMassPropertiesZeroMass)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.mass = 0.0f;
    body.UpdateMassProperties();
    EXPECT_FLOAT_NEAR(body.mass, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inverseMass, 0.0f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, UpdateMassPropertiesWithInertia)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.mass = 2.0f;
    body.inertia = 10.0f;
    body.UpdateMassProperties();
    EXPECT_FLOAT_NEAR(body.mass, 2.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inverseMass, 0.5f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inertia, 10.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inverseInertia, 0.1f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, UpdateMassPropertiesNoInertia)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.mass = 2.0f;
    body.inertia = 0.0f;   
    body.UpdateMassProperties();
    EXPECT_FLOAT_NEAR(body.mass, 2.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inverseMass, 0.5f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inertia, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inverseInertia, 0.0f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, SetAwake)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isAwake = false;
    body.sleepTimer = 10.0f;
    body.SetAwake(true);
    EXPECT_TRUE(body.isAwake);
    EXPECT_FLOAT_NEAR(body.sleepTimer, 0.0f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, SetAwakeStaticBody)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isStatic = true;
    body.isAwake = false;
    body.SetAwake(true);
    EXPECT_FALSE(body.isAwake);   
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, AllowSleepDisable)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isAwake = false;
    body.AllowSleep(false);
    EXPECT_FALSE(body.allowSleep);
    EXPECT_TRUE(body.isAwake);   
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, AllowSleepEnable)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.AllowSleep(true);
    EXPECT_TRUE(body.allowSleep);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyForce)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    Nyon::Math::Vector2 forceVec(10.0f, 20.0f);
    body.ApplyForce(forceVec);
    EXPECT_VECTOR2_NEAR(body.force, forceVec, 1e-5f);
    EXPECT_TRUE(body.isAwake);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyForceAccumulation)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    Nyon::Math::Vector2 force1(10.0f, 0.0f);
    Nyon::Math::Vector2 force2(0.0f, 20.0f);
    body.ApplyForce(force1);
    body.ApplyForce(force2);
    EXPECT_VECTOR2_NEAR(body.force, Nyon::Math::Vector2(10.0f, 20.0f), 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyForceStaticBody)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isStatic = true;
    Nyon::Math::Vector2 forceVec(10.0f, 20.0f);
    body.ApplyForce(forceVec);
    EXPECT_VECTOR2_NEAR(body.force, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);   
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyForceAtPoint)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.centerOfMass = Nyon::Math::Vector2(0.0f, 0.0f);
    Nyon::Math::Vector2 forceVec(10.0f, 0.0f);
    Nyon::Math::Vector2 point(0.0f, 5.0f);   
    body.ApplyForceAtPoint(forceVec, point);
    EXPECT_VECTOR2_NEAR(body.force, forceVec, 1e-5f);
    EXPECT_FLOAT_NEAR(body.torque, -50.0f, 1e-5f);
    EXPECT_TRUE(body.isAwake);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyForceAtPointWithOffset)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.centerOfMass = Nyon::Math::Vector2(1.0f, 1.0f);
    Nyon::Math::Vector2 forceVec(5.0f, 0.0f);
    Nyon::Math::Vector2 point(3.0f, 1.0f);   
    body.ApplyForceAtPoint(forceVec, point);
    EXPECT_VECTOR2_NEAR(body.force, forceVec, 1e-5f);
    EXPECT_FLOAT_NEAR(body.torque, 0.0f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyForceAtPointStaticBody)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isStatic = true;
    Nyon::Math::Vector2 forceVec(10.0f, 20.0f);
    Nyon::Math::Vector2 point(5.0f, 5.0f);
    body.ApplyForceAtPoint(forceVec, point);
    EXPECT_VECTOR2_NEAR(body.force, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(body.torque, 0.0f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyTorque)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.ApplyTorque(15.0f);
    EXPECT_FLOAT_NEAR(body.torque, 15.0f, 1e-5f);
    EXPECT_TRUE(body.isAwake);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyTorqueAccumulation)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.ApplyTorque(10.0f);
    body.ApplyTorque(5.0f);
    EXPECT_FLOAT_NEAR(body.torque, 15.0f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyTorqueStaticBody)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isStatic = true;
    body.ApplyTorque(15.0f);
    EXPECT_FLOAT_NEAR(body.torque, 0.0f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyLinearImpulse)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.mass = 2.0f;
    body.UpdateMassProperties();
    Nyon::Math::Vector2 impulse(4.0f, 6.0f);
    body.ApplyLinearImpulse(impulse);
    EXPECT_VECTOR2_NEAR(body.velocity, Nyon::Math::Vector2(2.0f, 3.0f), 1e-5f);
    EXPECT_TRUE(body.isAwake);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyLinearImpulseAccumulation)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.mass = 1.0f;
    body.UpdateMassProperties();
    Nyon::Math::Vector2 impulse1(2.0f, 0.0f);
    Nyon::Math::Vector2 impulse2(0.0f, 3.0f);
    body.ApplyLinearImpulse(impulse1);
    body.ApplyLinearImpulse(impulse2);
    EXPECT_VECTOR2_NEAR(body.velocity, Nyon::Math::Vector2(2.0f, 3.0f), 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyLinearImpulseStaticBody)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isStatic = true;
    Nyon::Math::Vector2 impulse(10.0f, 20.0f);
    body.ApplyLinearImpulse(impulse);
    EXPECT_VECTOR2_NEAR(body.velocity, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyAngularImpulse)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.inertia = 4.0f;
    body.UpdateMassProperties();
    body.ApplyAngularImpulse(8.0f);
    EXPECT_FLOAT_NEAR(body.angularVelocity, 2.0f, 1e-5f);
    EXPECT_TRUE(body.isAwake);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ApplyAngularImpulseStaticBody)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isStatic = true;
    body.ApplyAngularImpulse(10.0f);
    EXPECT_FLOAT_NEAR(body.angularVelocity, 0.0f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ClearForces)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.force = Nyon::Math::Vector2(10.0f, 20.0f);
    body.torque = 15.0f;
    body.ClearForces();
    EXPECT_VECTOR2_NEAR(body.force, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(body.torque, 0.0f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, IsStatic)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isStatic = true;
    EXPECT_TRUE(body.IsStatic());
    body.isStatic = false;
    EXPECT_FALSE(body.IsStatic());
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, IsKinematic)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isKinematic = true;
    EXPECT_TRUE(body.IsKinematic());
    body.isKinematic = false;
    EXPECT_FALSE(body.IsKinematic());
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, IsDynamic)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    EXPECT_TRUE(body.IsDynamic());
    body.isStatic = true;
    EXPECT_FALSE(body.IsDynamic());
    body.isStatic = false;
    body.isKinematic = true;
    EXPECT_FALSE(body.IsDynamic());
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ShouldCollide)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isAwake = true;
    EXPECT_TRUE(body.ShouldCollide());
    body.isAwake = false;
    EXPECT_TRUE(body.ShouldCollide());
    body.isKinematic = true;
    EXPECT_TRUE(body.ShouldCollide());
    body.isKinematic = false;
    body.isStatic = true;
    EXPECT_TRUE(body.ShouldCollide());
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, GetMass)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.mass = 7.5f;
    EXPECT_FLOAT_NEAR(body.GetMass(), 7.5f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, GetInertia)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.inertia = 12.5f;
    EXPECT_FLOAT_NEAR(body.GetInertia(), 12.5f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, MotionLocksDefault)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    EXPECT_FALSE(body.motionLocks.lockTranslationX);
    EXPECT_FALSE(body.motionLocks.lockTranslationY);
    EXPECT_FALSE(body.motionLocks.lockRotation);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, MotionLocksSetting)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.motionLocks.lockTranslationX = true;
    body.motionLocks.lockTranslationY = true;
    body.motionLocks.lockRotation = true;
    EXPECT_TRUE(body.motionLocks.lockTranslationX);
    EXPECT_TRUE(body.motionLocks.lockTranslationY);
    EXPECT_TRUE(body.motionLocks.lockRotation);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, GroundedStateDefault)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    EXPECT_FALSE(body.isGrounded);
    EXPECT_EQ(body.groundedFrames, 0);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, UpdateGroundedState)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.UpdateGroundedState(true);
    EXPECT_EQ(body.groundedFrames, 1);
    EXPECT_FALSE(body.isGrounded);   
    body.UpdateGroundedState(true);
    EXPECT_EQ(body.groundedFrames, 2);
    EXPECT_TRUE(body.isGrounded);   
    body.UpdateGroundedState(true);
    EXPECT_EQ(body.groundedFrames, 3);
    EXPECT_TRUE(body.isGrounded);
    body.UpdateGroundedState(false);
    EXPECT_EQ(body.groundedFrames, 0);
    EXPECT_FALSE(body.isGrounded);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, IsStablyGrounded)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    EXPECT_FALSE(body.IsStablyGrounded());
    body.groundedFrames = 1;
    EXPECT_FALSE(body.IsStablyGrounded());
    body.groundedFrames = 2;
    EXPECT_TRUE(body.IsStablyGrounded());
    body.groundedFrames = 5;
    EXPECT_TRUE(body.IsStablyGrounded());
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, VeryLargeMass)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.SetMass(1000000.0f);
    EXPECT_FLOAT_NEAR(body.mass, 1000000.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inverseMass, 0.000001f, 1e-10f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, VerySmallMass)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.SetMass(0.0001f);
    EXPECT_FLOAT_NEAR(body.mass, 0.0001f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.inverseMass, 10000.0f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, MultipleForceApplications)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    for (int i = 0; i < 10; i++) {
        body.ApplyForce(Nyon::Math::Vector2(1.0f, 1.0f));
    }
    EXPECT_VECTOR2_NEAR(body.force, Nyon::Math::Vector2(10.0f, 10.0f), 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ForceApplicationThenClear)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.ApplyForce(Nyon::Math::Vector2(5.0f, 10.0f));
    EXPECT_VECTOR2_NEAR(body.force, Nyon::Math::Vector2(5.0f, 10.0f), 1e-5f);
    body.ClearForces();
    EXPECT_VECTOR2_NEAR(body.force, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    body.ApplyForce(Nyon::Math::Vector2(3.0f, 7.0f));
    EXPECT_VECTOR2_NEAR(body.force, Nyon::Math::Vector2(3.0f, 7.0f), 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, StaticToDynamicTransition)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.isStatic = true;
    body.mass = 5.0f;
    body.UpdateMassProperties();
    EXPECT_FLOAT_NEAR(body.inverseMass, 0.0f, 1e-5f);
    body.isStatic = false;
    body.UpdateMassProperties();
    EXPECT_FLOAT_NEAR(body.inverseMass, 0.2f, 1e-5f);   
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, SleepThresholds)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    EXPECT_FLOAT_NEAR(body.TIME_TO_SLEEP, 0.5f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.LINEAR_SLEEP_TOLERANCE, 0.01f, 1e-5f);
    EXPECT_FLOAT_NEAR(body.ANGULAR_SLEEP_TOLERANCE, 0.01f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, BackwardCompatibilityConstants)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    EXPECT_EQ(body.GROUNDED_THRESHOLD, 2);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, FullPhysicsUpdateCycle)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.mass = 2.0f;
    body.inertia = 8.0f;
    body.UpdateMassProperties();
    body.ApplyForce(Nyon::Math::Vector2(10.0f, 0.0f));
    body.ApplyTorque(4.0f);
    EXPECT_VECTOR2_NEAR(body.force, Nyon::Math::Vector2(10.0f, 0.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(body.torque, 4.0f, 1e-5f);
    body.ClearForces();
    EXPECT_VECTOR2_NEAR(body.force, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(body.torque, 0.0f, 1e-5f);
    EXPECT_VECTOR2_NEAR(body.velocity, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(body.angularVelocity, 0.0f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ComplexForceApplicationAtPoint)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.mass = 1.0f;
    body.inertia = 1.0f;
    body.UpdateMassProperties();
    body.centerOfMass = Nyon::Math::Vector2(0.0f, 0.0f);
    body.ApplyForceAtPoint(Nyon::Math::Vector2(10.0f, 0.0f), Nyon::Math::Vector2(0.0f, 1.0f));
    EXPECT_FLOAT_NEAR(body.torque, -10.0f, 1e-5f);   
    body.ClearForces();
    body.ApplyForceAtPoint(Nyon::Math::Vector2(0.0f, 10.0f), Nyon::Math::Vector2(1.0f, 0.0f));
    EXPECT_FLOAT_NEAR(body.torque, 10.0f, 1e-5f);   
    body.ClearForces();
    body.ApplyForceAtPoint(Nyon::Math::Vector2(10.0f, 10.0f), Nyon::Math::Vector2(1.0f, 1.0f));
    EXPECT_FLOAT_NEAR(body.torque, 0.0f, 1e-5f);   
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, ImpulseBasedMovement)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    body.mass = 2.0f;
    body.inertia = 4.0f;
    body.UpdateMassProperties();
    body.ApplyLinearImpulse(Nyon::Math::Vector2(4.0f, 0.0f));
    body.ApplyLinearImpulse(Nyon::Math::Vector2(0.0f, 6.0f));
    EXPECT_VECTOR2_NEAR(body.velocity, Nyon::Math::Vector2(2.0f, 3.0f), 1e-5f);
    body.ApplyAngularImpulse(8.0f);
    EXPECT_FLOAT_NEAR(body.angularVelocity, 2.0f, 1e-5f);
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, BodyTypeBehaviorComprehensive)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent staticBody(10.0f, true);
    staticBody.ApplyForce(Nyon::Math::Vector2(100.0f, 100.0f));
    staticBody.ApplyTorque(50.0f);
    staticBody.ApplyLinearImpulse(Nyon::Math::Vector2(10.0f, 10.0f));
    staticBody.ApplyAngularImpulse(5.0f);
    EXPECT_VECTOR2_NEAR(staticBody.force, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(staticBody.torque, 0.0f, 1e-5f);
    EXPECT_VECTOR2_NEAR(staticBody.velocity, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(staticBody.angularVelocity, 0.0f, 1e-5f);
    EXPECT_TRUE(staticBody.IsStatic());
    EXPECT_FALSE(staticBody.IsDynamic());
    PhysicsBodyComponent kinematicBody;
    kinematicBody.isKinematic = true;
    kinematicBody.UpdateMassProperties();
    EXPECT_FLOAT_NEAR(kinematicBody.inverseMass, 0.0f, 1e-5f);
    EXPECT_TRUE(kinematicBody.IsKinematic());
    EXPECT_FALSE(kinematicBody.IsDynamic());
    EXPECT_TRUE(kinematicBody.ShouldCollide());   
    PhysicsBodyComponent dynamicBody;
    dynamicBody.mass = 5.0f;
    dynamicBody.UpdateMassProperties();
    EXPECT_FLOAT_NEAR(dynamicBody.inverseMass, 0.2f, 1e-5f);
    EXPECT_TRUE(dynamicBody.IsDynamic());
    EXPECT_FALSE(dynamicBody.IsStatic());
    EXPECT_FALSE(dynamicBody.IsKinematic());
    LOG_FUNC_EXIT();
}
TEST(PhysicsBodyComponentTest, SleepWakeCycleScenario)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent body;
    EXPECT_TRUE(body.isAwake);
    body.SetAwake(false);
    EXPECT_FALSE(body.isAwake);
    body.ApplyForce(Nyon::Math::Vector2(1.0f, 1.0f));
    EXPECT_TRUE(body.isAwake);
    body.AllowSleep(false);
    EXPECT_FALSE(body.allowSleep);
    EXPECT_TRUE(body.isAwake);
    body.AllowSleep(true);
    EXPECT_TRUE(body.allowSleep);
    EXPECT_TRUE(body.isAwake);
    LOG_FUNC_EXIT();
}
