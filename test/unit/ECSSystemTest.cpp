#include <gtest/gtest.h>
#include "nyon/ecs/systems/PhysicsSystem.h"
#include "nyon/ecs/systems/InputSystem.h"
#include "nyon/ecs/systems/RenderSystem.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "TestHelpers.h"

using namespace Nyon::ECS;

/**
 * @brief Mock renderer for testing RenderSystem without OpenGL context.
 */
class MockRenderer
{
public:
    static void Init() { initialized = true; }
    static void Shutdown() { initialized = false; }
    static void BeginScene() { sceneStarted = true; clearCalls++; }
    static void EndScene() { sceneStarted = false; flushCalls++; }
    static void DrawQuad(const Nyon::Math::Vector2& pos, const Nyon::Math::Vector2& size, 
                        const Nyon::Math::Vector2& origin, const Nyon::Math::Vector3& color) {
        drawCalls++;
        lastPosition = pos;
        lastSize = size;
        lastColor = color;
    }
    
    static bool initialized;
    static bool sceneStarted;
    static int clearCalls;
    static int flushCalls;
    static int drawCalls;
    static Nyon::Math::Vector2 lastPosition;
    static Nyon::Math::Vector2 lastSize;
    static Nyon::Math::Vector3 lastColor;
};

bool MockRenderer::initialized = false;
bool MockRenderer::sceneStarted = false;
int MockRenderer::clearCalls = 0;
int MockRenderer::flushCalls = 0;
int MockRenderer::drawCalls = 0;
Nyon::Math::Vector2 MockRenderer::lastPosition;
Nyon::Math::Vector2 MockRenderer::lastSize;
Nyon::Math::Vector3 MockRenderer::lastColor;

/**
 * @brief Unit tests for PhysicsSystem functionality.
 * 
 * Tests physics integration, body updates, grounded state handling,
 * and integration with ECS components.
 */
class PhysicsSystemTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        entityManager = std::make_unique<EntityManager>();
        componentStore = std::make_unique<ComponentStore>(*entityManager);
        physicsSystem = std::make_unique<PhysicsSystem>();
        physicsSystem->Initialize(*entityManager, *componentStore);
        LOG_FUNC_EXIT();
    }

    void TearDown() override
    {
        LOG_FUNC_ENTER();
        physicsSystem.reset();
        componentStore.reset();
        entityManager.reset();
        LOG_FUNC_EXIT();
    }

    std::unique_ptr<EntityManager> entityManager;
    std::unique_ptr<ComponentStore> componentStore;
    std::unique_ptr<PhysicsSystem> physicsSystem;
};

// Basic update tests
TEST_F(PhysicsSystemTest, Update_EmptySystem)
{
    LOG_FUNC_ENTER();
    // Removed PERF_TIMER due to compilation issues
    // PERF_TIMER("PhysicsSystem_Update_Empty");
    
    EXPECT_NO_THROW(physicsSystem->Update(1.0f / 60.0f));
    LOG_FUNC_EXIT();
}

TEST_F(PhysicsSystemTest, Update_SingleDynamicBody)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    TransformComponent transform({100.0f, 200.0f});
    PhysicsBodyComponent physics(1.0f, false);
    physics.velocity.y = 100.0f; // Moving downward
    
    componentStore->AddComponent(entity, std::move(transform));
    componentStore->AddComponent(entity, std::move(physics));
    
    float deltaTime = 1.0f / 60.0f;
    physicsSystem->Update(deltaTime);
    
    const TransformComponent& updatedTransform = componentStore->GetComponent<TransformComponent>(entity);
    const PhysicsBodyComponent& updatedPhysics = componentStore->GetComponent<PhysicsBodyComponent>(entity);
    
    LOG_VAR_DEBUG(updatedTransform.position.y);
    LOG_VAR_DEBUG(updatedPhysics.velocity.y);
    
    // Should have moved due to velocity and gravity
    EXPECT_GT(updatedTransform.position.y, 200.0f);
    EXPECT_GT(updatedPhysics.velocity.y, 100.0f);
    LOG_FUNC_EXIT();
}

TEST_F(PhysicsSystemTest, Update_StaticBody_NoMovement)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    TransformComponent transform({100.0f, 200.0f});
    PhysicsBodyComponent physics(1.0f, true); // Static body
    physics.velocity.y = 100.0f; // Even with velocity
    
    componentStore->AddComponent(entity, std::move(transform));
    componentStore->AddComponent(entity, std::move(physics));
    
    physicsSystem->Update(1.0f / 60.0f);
    
    const TransformComponent& updatedTransform = componentStore->GetComponent<TransformComponent>(entity);
    const PhysicsBodyComponent& updatedPhysics = componentStore->GetComponent<PhysicsBodyComponent>(entity);
    
    LOG_VAR_DEBUG(updatedTransform.position.x);
    LOG_VAR_DEBUG(updatedTransform.position.y);
    LOG_VAR_DEBUG(updatedPhysics.velocity.x);
    LOG_VAR_DEBUG(updatedPhysics.velocity.y);
    
    // Static bodies should not move regardless of velocity
    EXPECT_FLOAT_NEAR(updatedTransform.position.x, 100.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(updatedTransform.position.y, 200.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(updatedPhysics.velocity.x, 0.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(updatedPhysics.velocity.y, 0.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

// Grounded state integration tests
TEST_F(PhysicsSystemTest, Update_GroundedStatePropagation)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    TransformComponent transform({100.0f, 200.0f});
    PhysicsBodyComponent physics(1.0f, false);
    
    componentStore->AddComponent(entity, std::move(transform));
    componentStore->AddComponent(entity, std::move(physics));
    
    // Simulate being grounded for several frames
    for (int i = 0; i < PhysicsBodyComponent::GROUNDED_THRESHOLD + 1; ++i) {
        // Manually update grounded state (simulating collision system)
        auto& physicsRef = componentStore->GetComponent<PhysicsBodyComponent>(entity);
        physicsRef.UpdateGroundedState(true);
        physicsSystem->Update(1.0f / 60.0f);
    }
    
    const PhysicsBodyComponent& finalPhysics = componentStore->GetComponent<PhysicsBodyComponent>(entity);
    
    LOG_VAR_DEBUG(finalPhysics.IsStablyGrounded());
    LOG_VAR_DEBUG(finalPhysics.groundedFrames);
    
    EXPECT_TRUE(finalPhysics.IsStablyGrounded());
    EXPECT_GE(finalPhysics.groundedFrames, PhysicsBodyComponent::GROUNDED_THRESHOLD);
    LOG_FUNC_EXIT();
}

// Multiple entity tests
TEST_F(PhysicsSystemTest, Update_MultipleBodies)
{
    LOG_FUNC_ENTER();
    // Removed PERF_TIMER due to compilation issues
    // PERF_TIMER("PhysicsSystem_Update_MultipleBodies");
    
    const int entityCount = 10;
    std::vector<EntityID> entities;
    
    // Create multiple entities with different properties
    for (int i = 0; i < entityCount; ++i) {
        EntityID entity = entityManager->CreateEntity();
        entities.push_back(entity);
        
        TransformComponent transform({static_cast<float>(i * 50), static_cast<float>(i * 30)});
        PhysicsBodyComponent physics(1.0f, false);
        physics.velocity.y = static_cast<float>(i * 10); // Different velocities
        
        componentStore->AddComponent(entity, std::move(transform));
        componentStore->AddComponent(entity, std::move(physics));
    }
    
    physicsSystem->Update(1.0f / 60.0f);
    
    // Verify all entities were processed
    for (const auto& entity : entities) {
        EXPECT_TRUE(componentStore->HasComponent<TransformComponent>(entity));
        EXPECT_TRUE(componentStore->HasComponent<PhysicsBodyComponent>(entity));
    }
    LOG_FUNC_EXIT();
}

/**
 * @brief Unit tests for InputSystem functionality.
 * 
 * Tests input processing, behavior component integration,
 * and event handling without requiring actual input devices.
 */
class InputSystemTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        entityManager = std::make_unique<EntityManager>();
        componentStore = std::make_unique<ComponentStore>(*entityManager);
        inputSystem = std::make_unique<InputSystem>();
        inputSystem->Initialize(*entityManager, *componentStore);
        LOG_FUNC_EXIT();
    }

    void TearDown() override
    {
        LOG_FUNC_ENTER();
        inputSystem.reset();
        componentStore.reset();
        entityManager.reset();
        LOG_FUNC_EXIT();
    }

    std::unique_ptr<EntityManager> entityManager;
    std::unique_ptr<ComponentStore> componentStore;
    std::unique_ptr<InputSystem> inputSystem;
};

// Basic update tests
TEST_F(InputSystemTest, Update_EmptySystem)
{
    LOG_FUNC_ENTER();
    // Removed PERF_TIMER due to compilation issues
    // PERF_TIMER("InputSystem_Update_Empty");
    
    EXPECT_NO_THROW(inputSystem->Update(1.0f / 60.0f));
    LOG_FUNC_EXIT();
}

TEST_F(InputSystemTest, Update_WithBehaviorComponent)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    bool behaviorCalled = false;
    float capturedDelta = 0.0f;
    
    BehaviorComponent behavior;
    auto updateFunc = [&](EntityID entityId, float deltaTime) {
        behaviorCalled = true;
        capturedDelta = deltaTime;
        LOG_VAR_DEBUG(entityId);
        LOG_VAR_DEBUG(deltaTime);
    };
    
    behavior.SetUpdateFunction(updateFunc);
    
    componentStore->AddComponent(entity, std::move(behavior));
    
    float deltaTime = 1.0f / 60.0f;
    inputSystem->Update(deltaTime);
    
    EXPECT_TRUE(behaviorCalled);
    EXPECT_FLOAT_NEAR(capturedDelta, deltaTime, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(InputSystemTest, Update_MultipleBehaviorComponents)
{
    LOG_FUNC_ENTER();
    const int entityCount = 5;
    std::vector<bool> behaviorCalled(entityCount, false);
    
    for (int i = 0; i < entityCount; ++i) {
        EntityID entity = entityManager->CreateEntity();
        
        BehaviorComponent behavior;
        int index = i; // Capture by value
        auto updateFunc = [&, index](EntityID entityId, float deltaTime) {
            behaviorCalled[index] = true;
        };
        
        behavior.SetUpdateFunction(updateFunc);
        
        componentStore->AddComponent(entity, std::move(behavior));
    }
    
    inputSystem->Update(1.0f / 60.0f);
    
    // All behaviors should have been called
    for (int i = 0; i < entityCount; ++i) {
        LOG_VAR_DEBUG(i);
        LOG_VAR_DEBUG(behaviorCalled[i]);
        EXPECT_TRUE(behaviorCalled[i]);
    }
    LOG_FUNC_EXIT();
}

TEST_F(InputSystemTest, Update_EntitiesWithoutBehavior)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    // Add a transform component but no behavior component
    TransformComponent transform({100.0f, 200.0f});
    componentStore->AddComponent(entity, std::move(transform));
    
    // Should not crash when updating entities without behavior components
    EXPECT_NO_THROW(inputSystem->Update(1.0f / 60.0f));
    LOG_FUNC_EXIT();
}

/**
 * @brief Unit tests for RenderSystem functionality.
 * 
 * Tests rendering integration, component processing,
 * and rendering calls without requiring OpenGL context.
 */
class RenderSystemTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        entityManager = std::make_unique<EntityManager>();
        componentStore = std::make_unique<ComponentStore>(*entityManager);
        renderSystem = std::make_unique<RenderSystem>();
        
        // Mock the renderer functions
        MockRenderer::clearCalls = 0;
        MockRenderer::flushCalls = 0;
        MockRenderer::drawCalls = 0;
        
        renderSystem->Initialize(*entityManager, *componentStore);
        LOG_FUNC_EXIT();
    }

    void TearDown() override
    {
        LOG_FUNC_ENTER();
        renderSystem.reset();
        componentStore.reset();
        entityManager.reset();
        LOG_FUNC_EXIT();
    }

    std::unique_ptr<EntityManager> entityManager;
    std::unique_ptr<ComponentStore> componentStore;
    std::unique_ptr<RenderSystem> renderSystem;
};

// Basic rendering tests
TEST_F(RenderSystemTest, Update_EmptySystem)
{
    LOG_FUNC_ENTER();
    // Removed PERF_TIMER due to compilation issues
    // PERF_TIMER("RenderSystem_Update_Empty");
    
    renderSystem->Update(1.0f / 60.0f);
    
    LOG_VAR_DEBUG(MockRenderer::clearCalls);
    LOG_VAR_DEBUG(MockRenderer::flushCalls);
    EXPECT_GE(MockRenderer::clearCalls, 1);
    EXPECT_GE(MockRenderer::flushCalls, 1);
    LOG_FUNC_EXIT();
}

TEST_F(RenderSystemTest, Update_SingleRenderableEntity)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    TransformComponent transform({100.0f, 200.0f});
    RenderComponent render({32.0f, 32.0f}, {1.0f, 0.0f, 0.0f}); // Red square
    
    componentStore->AddComponent(entity, std::move(transform));
    componentStore->AddComponent(entity, std::move(render));
    
    int initialDrawCalls = MockRenderer::drawCalls;
    renderSystem->Update(1.0f / 60.0f);
    
    LOG_VAR_DEBUG(MockRenderer::drawCalls);
    LOG_VAR_DEBUG(MockRenderer::lastPosition.x);
    LOG_VAR_DEBUG(MockRenderer::lastPosition.y);
    LOG_VAR_DEBUG(MockRenderer::lastSize.x);
    LOG_VAR_DEBUG(MockRenderer::lastSize.y);
    LOG_VAR_DEBUG(MockRenderer::lastColor.x);
    LOG_VAR_DEBUG(MockRenderer::lastColor.y);
    LOG_VAR_DEBUG(MockRenderer::lastColor.z);
    
    // Should have drawn the entity
    EXPECT_GT(MockRenderer::drawCalls, initialDrawCalls);
    EXPECT_VECTOR2_NEAR(MockRenderer::lastPosition, Nyon::Math::Vector2(100.0f, 200.0f), 1e-6f);
    EXPECT_VECTOR2_NEAR(MockRenderer::lastSize, Nyon::Math::Vector2(32.0f, 32.0f), 1e-6f);
    EXPECT_VECTOR3_NEAR(MockRenderer::lastColor, Nyon::Math::Vector3(1.0f, 0.0f, 0.0f), 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(RenderSystemTest, Update_InvisibleEntity)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    TransformComponent transform({100.0f, 200.0f});
    RenderComponent render({32.0f, 32.0f}, {1.0f, 0.0f, 0.0f});
    render.visible = false; // Make invisible
    
    componentStore->AddComponent(entity, std::move(transform));
    componentStore->AddComponent(entity, std::move(render));
    
    int initialDrawCalls = MockRenderer::drawCalls;
    renderSystem->Update(1.0f / 60.0f);
    
    LOG_VAR_DEBUG(MockRenderer::drawCalls);
    // Invisible entities should not be rendered
    EXPECT_EQ(MockRenderer::drawCalls, initialDrawCalls);
    LOG_FUNC_EXIT();
}

TEST_F(RenderSystemTest, Update_EntityWithoutTransform)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    // Only render component, no transform
    RenderComponent render({32.0f, 32.0f}, {1.0f, 0.0f, 0.0f});
    componentStore->AddComponent(entity, std::move(render));
    
    int initialDrawCalls = MockRenderer::drawCalls;
    renderSystem->Update(1.0f / 60.0f);
    
    LOG_VAR_DEBUG(MockRenderer::drawCalls);
    // Entities without transform should not be rendered
    EXPECT_EQ(MockRenderer::drawCalls, initialDrawCalls);
    LOG_FUNC_EXIT();
}

TEST_F(RenderSystemTest, Update_MultipleRenderableEntities)
{
    LOG_FUNC_ENTER();
    // Removed PERF_TIMER due to compilation issues
    // PERF_TIMER("RenderSystem_Update_Multiple");
    
    const int entityCount = 5;
    std::vector<EntityID> entities;
    
    for (int i = 0; i < entityCount; ++i) {
        EntityID entity = entityManager->CreateEntity();
        entities.push_back(entity);
        
        TransformComponent transform({static_cast<float>(i * 50), static_cast<float>(i * 50)});
        RenderComponent render({32.0f, 32.0f}, {static_cast<float>(i)/entityCount, 0.5f, 0.8f});
        
        componentStore->AddComponent(entity, std::move(transform));
        componentStore->AddComponent(entity, std::move(render));
    }
    
    int initialDrawCalls = MockRenderer::drawCalls;
    renderSystem->Update(1.0f / 60.0f);
    
    LOG_VAR_DEBUG(MockRenderer::drawCalls);
    LOG_VAR_DEBUG(initialDrawCalls);
    // Should draw all visible entities
    EXPECT_GE(MockRenderer::drawCalls, initialDrawCalls + entityCount);
    LOG_FUNC_EXIT();
}

// Performance and stress tests
TEST_F(RenderSystemTest, Performance_ManyEntities)
{
    LOG_FUNC_ENTER();
    // Removed PERF_TIMER due to compilation issues
    // PERF_TIMER("RenderSystem_Performance_ManyEntities");
    
    const int entityCount = 1000;
    
    for (int i = 0; i < entityCount; ++i) {
        EntityID entity = entityManager->CreateEntity();
        
        TransformComponent transform({static_cast<float>(i), static_cast<float>(i)});
        RenderComponent render({16.0f, 16.0f}, {0.5f, 0.5f, 0.5f});
        
        componentStore->AddComponent(entity, std::move(transform));
        componentStore->AddComponent(entity, std::move(render));
    }
    
    renderSystem->Update(1.0f / 60.0f);
    
    LOG_VAR_DEBUG(MockRenderer::drawCalls);
    EXPECT_GE(MockRenderer::drawCalls, entityCount);
    LOG_FUNC_EXIT();
}

// Edge cases
TEST_F(RenderSystemTest, Update_ZeroSizedEntity)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    TransformComponent transform({100.0f, 200.0f});
    RenderComponent render({0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}); // Zero size
    
    componentStore->AddComponent(entity, std::move(transform));
    componentStore->AddComponent(entity, std::move(render));
    
    int initialDrawCalls = MockRenderer::drawCalls;
    renderSystem->Update(1.0f / 60.0f);
    
    LOG_VAR_DEBUG(MockRenderer::drawCalls);
    // Zero-sized entities should still be rendered (point rendering)
    EXPECT_GT(MockRenderer::drawCalls, initialDrawCalls);
    LOG_FUNC_EXIT();
}

TEST_F(RenderSystemTest, Update_NegativeSizedEntity)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    TransformComponent transform({100.0f, 200.0f});
    RenderComponent render({-32.0f, -32.0f}, {1.0f, 0.0f, 0.0f}); // Negative size
    
    componentStore->AddComponent(entity, std::move(transform));
    componentStore->AddComponent(entity, std::move(render));
    
    int initialDrawCalls = MockRenderer::drawCalls;
    renderSystem->Update(1.0f / 60.0f);
    
    LOG_VAR_DEBUG(MockRenderer::drawCalls);
    // Negative sizes should be handled gracefully
    EXPECT_NO_THROW(renderSystem->Update(1.0f / 60.0f));
    LOG_FUNC_EXIT();
}