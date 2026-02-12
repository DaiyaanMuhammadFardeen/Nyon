#include <gtest/gtest.h>
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/BehaviorComponent.h"
#include "nyon/ecs/EntityManager.h"
#include "TestHelpers.h"

using namespace Nyon::ECS;

/**
 * @brief Unit tests for TransformComponent functionality.
 * 
 * Tests position manipulation and basic transform operations.
 */
class TransformComponentTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        transform = TransformComponent();
        LOG_FUNC_EXIT();
    }

    TransformComponent transform;
};

// Constructor tests
TEST_F(TransformComponentTest, Constructor_Default)
{
    LOG_FUNC_ENTER();
    TransformComponent defaultTransform;
    
    LOG_VAR_DEBUG(defaultTransform.position.x);
    LOG_VAR_DEBUG(defaultTransform.position.y);
    LOG_VAR_DEBUG(defaultTransform.rotation);
    LOG_VAR_DEBUG(defaultTransform.scale.x);
    LOG_VAR_DEBUG(defaultTransform.scale.y);
    
    EXPECT_VECTOR2_NEAR(defaultTransform.position, Nyon::Math::Vector2(0.0f, 0.0f), 1e-6f);
    EXPECT_FLOAT_EQ(defaultTransform.rotation, 0.0f);
    EXPECT_VECTOR2_NEAR(defaultTransform.scale, Nyon::Math::Vector2(1.0f, 1.0f), 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(TransformComponentTest, Constructor_WithPosition)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 pos(100.0f, 200.0f);
    TransformComponent positionedTransform(pos);
    
    LOG_VAR_DEBUG(positionedTransform.position.x);
    LOG_VAR_DEBUG(positionedTransform.position.y);
    
    EXPECT_VECTOR2_NEAR(positionedTransform.position, pos, 1e-6f);
    LOG_FUNC_EXIT();
}

// Position manipulation tests
TEST_F(TransformComponentTest, SetPosition)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 newPosition(500.0f, 300.0f);
    
    transform.position = newPosition;
    
    LOG_VAR_DEBUG(transform.position.x);
    LOG_VAR_DEBUG(transform.position.y);
    EXPECT_VECTOR2_NEAR(transform.position, newPosition, 1e-6f);
    LOG_FUNC_EXIT();
}

/**
 * @brief Unit tests for PhysicsBodyComponent functionality.
 * 
 * Tests physics properties and basic kinematic calculations.
 */
class PhysicsBodyComponentTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        physicsBody = PhysicsBodyComponent();
        LOG_FUNC_EXIT();
    }

    PhysicsBodyComponent physicsBody;
};

// Constructor tests
TEST_F(PhysicsBodyComponentTest, Constructor_Default)
{
    LOG_FUNC_ENTER();
    PhysicsBodyComponent defaultBody;
    
    LOG_VAR_DEBUG(defaultBody.mass);
    LOG_VAR_DEBUG(defaultBody.friction);
    LOG_VAR_DEBUG(defaultBody.drag);
    LOG_VAR_DEBUG(defaultBody.maxSpeed);
    LOG_VAR_DEBUG(defaultBody.isStatic);
    LOG_VAR_DEBUG(defaultBody.isGrounded);
    LOG_VAR_DEBUG(defaultBody.groundedFrames);
    
    EXPECT_FLOAT_EQ(defaultBody.mass, 1.0f);
    EXPECT_FLOAT_EQ(defaultBody.friction, 0.1f);
    EXPECT_FLOAT_EQ(defaultBody.drag, 0.0f);
    EXPECT_FLOAT_EQ(defaultBody.maxSpeed, 1000.0f);
    EXPECT_FALSE(defaultBody.isStatic);
    EXPECT_FALSE(defaultBody.isGrounded);
    EXPECT_EQ(defaultBody.groundedFrames, 0);
    LOG_FUNC_EXIT();
}

TEST_F(PhysicsBodyComponentTest, Constructor_WithMass)
{
    LOG_FUNC_ENTER();
    float mass = 5.0f;
    PhysicsBodyComponent bodyWithMass(mass);
    
    LOG_VAR_DEBUG(bodyWithMass.mass);
    EXPECT_FLOAT_EQ(bodyWithMass.mass, mass);
    LOG_FUNC_EXIT();
}

TEST_F(PhysicsBodyComponentTest, Constructor_WithMassAndStatic)
{
    LOG_FUNC_ENTER();
    float mass = 3.0f;
    bool isStatic = true;
    PhysicsBodyComponent staticBody(mass, isStatic);
    
    LOG_VAR_DEBUG(staticBody.mass);
    LOG_VAR_DEBUG(staticBody.isStatic);
    EXPECT_FLOAT_EQ(staticBody.mass, mass);
    EXPECT_TRUE(staticBody.isStatic);
    LOG_FUNC_EXIT();
}

// Velocity and acceleration tests
TEST_F(PhysicsBodyComponentTest, Velocity_Manipulation)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 newVelocity(100.0f, -50.0f);
    
    physicsBody.velocity = newVelocity;
    
    LOG_VAR_DEBUG(physicsBody.velocity.x);
    LOG_VAR_DEBUG(physicsBody.velocity.y);
    EXPECT_VECTOR2_NEAR(physicsBody.velocity, newVelocity, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(PhysicsBodyComponentTest, Acceleration_Manipulation)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 newAcceleration(10.0f, 20.0f);
    
    physicsBody.acceleration = newAcceleration;
    
    LOG_VAR_DEBUG(physicsBody.acceleration.x);
    LOG_VAR_DEBUG(physicsBody.acceleration.y);
    EXPECT_VECTOR2_NEAR(physicsBody.acceleration, newAcceleration, 1e-6f);
    LOG_FUNC_EXIT();
}

/**
 * @brief Unit tests for RenderComponent functionality.
 * 
 * Tests rendering properties and visibility control.
 */
class RenderComponentTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        render = RenderComponent();
        LOG_FUNC_EXIT();
    }

    RenderComponent render;
};

// Constructor tests
TEST_F(RenderComponentTest, Constructor_Default)
{
    LOG_FUNC_ENTER();
    RenderComponent defaultRender;
    
    LOG_VAR_DEBUG(defaultRender.size.x);
    LOG_VAR_DEBUG(defaultRender.size.y);
    LOG_VAR_DEBUG(defaultRender.color.x);
    LOG_VAR_DEBUG(defaultRender.color.y);
    LOG_VAR_DEBUG(defaultRender.color.z);
    LOG_VAR_DEBUG(defaultRender.visible);
    LOG_VAR_DEBUG(defaultRender.layer);
    
    EXPECT_VECTOR2_NEAR(defaultRender.size, Nyon::Math::Vector2(32.0f, 32.0f), 1e-6f);
    EXPECT_VECTOR3_NEAR(defaultRender.color, Nyon::Math::Vector3(1.0f, 1.0f, 1.0f), 1e-6f);
    EXPECT_TRUE(defaultRender.visible);
    EXPECT_EQ(defaultRender.layer, 0);
    LOG_FUNC_EXIT();
}

TEST_F(RenderComponentTest, Constructor_WithSize)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 size(64.0f, 48.0f);
    RenderComponent sizedRender(size);
    
    LOG_VAR_DEBUG(sizedRender.size.x);
    LOG_VAR_DEBUG(sizedRender.size.y);
    EXPECT_VECTOR2_NEAR(sizedRender.size, size, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(RenderComponentTest, Constructor_WithSizeAndColor)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 size(64.0f, 48.0f);
    Nyon::Math::Vector3 color(0.5f, 0.7f, 0.9f);
    RenderComponent coloredRender(size, color);
    
    LOG_VAR_DEBUG(coloredRender.size.x);
    LOG_VAR_DEBUG(coloredRender.size.y);
    LOG_VAR_DEBUG(coloredRender.color.x);
    LOG_VAR_DEBUG(coloredRender.color.y);
    LOG_VAR_DEBUG(coloredRender.color.z);
    
    EXPECT_VECTOR2_NEAR(coloredRender.size, size, 1e-6f);
    EXPECT_VECTOR3_NEAR(coloredRender.color, color, 1e-6f);
    LOG_FUNC_EXIT();
}

// Visibility tests
TEST_F(RenderComponentTest, SetVisible_True)
{
    LOG_FUNC_ENTER();
    render.visible = false;
    
    render.visible = true;
    
    LOG_VAR_DEBUG(render.visible);
    EXPECT_TRUE(render.visible);
    LOG_FUNC_EXIT();
}

TEST_F(RenderComponentTest, SetVisible_False)
{
    LOG_FUNC_ENTER();
    render.visible = true;
    
    render.visible = false;
    
    LOG_VAR_DEBUG(render.visible);
    EXPECT_FALSE(render.visible);
    LOG_FUNC_EXIT();
}

/**
 * @brief Unit tests for BehaviorComponent functionality.
 * 
 * Tests behavior attachment and update callbacks.
 */
class BehaviorComponentTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        behavior = BehaviorComponent();
        testEntity = 123u; // Mock entity ID
        LOG_FUNC_EXIT();
    }

    BehaviorComponent behavior;
    EntityID testEntity;
};

// Update function tests
TEST_F(BehaviorComponentTest, SetUpdateFunction)
{
    LOG_FUNC_ENTER();
    bool updateCalled = false;
    float capturedDelta = 0.0f;
    EntityID capturedEntity = INVALID_ENTITY;
    
    auto updateFunc = [&](EntityID entity, float deltaTime) {
        updateCalled = true;
        capturedEntity = entity;
        capturedDelta = deltaTime;
    };
    
    behavior.SetUpdateFunction(updateFunc);
    
    float testDelta = 1.0f / 60.0f;
    behavior.Update(testEntity, testDelta);
    
    LOG_VAR_DEBUG(updateCalled);
    LOG_VAR_DEBUG(capturedEntity);
    LOG_VAR_DEBUG(capturedDelta);
    
    EXPECT_TRUE(updateCalled);
    EXPECT_EQ(capturedEntity, testEntity);
    EXPECT_FLOAT_NEAR(capturedDelta, testDelta, 1e-6f);
    LOG_FUNC_EXIT();
}


