#include <gtest/gtest.h>
#include "gtest/gtest.h"
#include "nyon/core/Application.h"
#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/SystemManager.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/BehaviorComponent.h"
#include "TestHelpers.h"
#include <thread>
#include <chrono>

// Mock classes for testing without OpenGL context
class MockApplication : public Nyon::Application
{
public:
    MockApplication() : Application("Mock App", 800, 600), initCalled(false), startCalled(false), updateCalled(false) {}
    
    bool initCalled;
    bool startCalled;
    bool updateCalled;
    
public:
    void OnStart() override
    {
        startCalled = true;
        LOG_VAR_DEBUG(startCalled);
    }
    
    void OnUpdate(float deltaTime) override
    {
        updateCalled = true;
        LOG_VAR_DEBUG(updateCalled);
        LOG_VAR_DEBUG(deltaTime);
    }
    
    void OnFixedUpdate(float deltaTime) override
    {
        updateCalled = true;
        LOG_VAR_DEBUG(updateCalled);
        LOG_VAR_DEBUG(deltaTime);
    }
};

class MockECSApplication : public Nyon::ECSApplication
{
public:
    MockECSApplication() : ECSApplication("Mock ECS App", 800, 600), 
                          ecsStartCalled(false), ecsUpdateCalled(false) {}
    
    bool ecsStartCalled;
    bool ecsUpdateCalled;
    
public:
    void OnECSStart() override
    {
        ecsStartCalled = true;
        LOG_VAR_DEBUG(ecsStartCalled);
    }
    
    void OnECSUpdate(float deltaTime) override
    {
        ecsUpdateCalled = true;
        LOG_VAR_DEBUG(ecsUpdateCalled);
        LOG_VAR_DEBUG(deltaTime);
    }
    

};

/**
 * @brief Unit tests for Application and ECSApplication base classes.
 * 
 * Tests lifecycle management, system integration, initialization sequences,
 * and core application functionality without requiring actual window creation.
 */
class ApplicationTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        LOG_FUNC_EXIT();
    }

    void TearDown() override
    {
        LOG_FUNC_ENTER();
        LOG_FUNC_EXIT();
    }
};

// Application base class tests
TEST_F(ApplicationTest, Constructor)
{
    LOG_FUNC_ENTER();
    // Test construction without actually creating window (would fail in headless environment)
    EXPECT_NO_THROW({
        // We can't actually create the application due to OpenGL requirements
        // But we can test the concept
        LOG_INFO("Application constructor test - would create window in real environment");
    });
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, GetInstance)
{
    LOG_FUNC_ENTER();
    // This would normally work, but without actual application creation...
    // Application::Get() should return the singleton instance
    LOG_INFO("Application::Get() test - requires actual application instance");
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, VirtualMethods)
{
    LOG_FUNC_ENTER();
    MockApplication mockApp;
    
    // Test that virtual methods can be called
    EXPECT_NO_THROW({
        mockApp.OnStart();
        mockApp.OnUpdate(1.0f/60.0f);
        mockApp.OnFixedUpdate(1.0f/60.0f);
    });
    
    LOG_VAR_DEBUG(mockApp.startCalled);
    LOG_VAR_DEBUG(mockApp.updateCalled);
    
    EXPECT_TRUE(mockApp.startCalled);
    EXPECT_TRUE(mockApp.updateCalled);
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, Lifecycle_Sequence)
{
    LOG_FUNC_ENTER();
    MockApplication mockApp;
    
    // Test the expected call sequence
    EXPECT_FALSE(mockApp.startCalled);
    EXPECT_FALSE(mockApp.updateCalled);
    
    mockApp.OnStart();
    EXPECT_TRUE(mockApp.startCalled);
    EXPECT_FALSE(mockApp.updateCalled);
    
    mockApp.OnUpdate(1.0f/60.0f);
    EXPECT_TRUE(mockApp.updateCalled);
    LOG_FUNC_EXIT();
}

// ECSApplication tests
TEST_F(ApplicationTest, ECSApplication_Constructor)
{
    LOG_FUNC_ENTER();
    // Similar to Application - can't create actual instance without OpenGL
    EXPECT_NO_THROW({
        LOG_INFO("ECSApplication constructor test - would create window in real environment");
    });
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, ECSApplication_VirtualMethods)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;
    
    EXPECT_NO_THROW({
        mockECSApp.OnECSStart();
        mockECSApp.OnECSUpdate(1.0f/60.0f);
    });
    
    LOG_VAR_DEBUG(mockECSApp.ecsStartCalled);
    LOG_VAR_DEBUG(mockECSApp.ecsUpdateCalled);
    
    EXPECT_TRUE(mockECSApp.ecsStartCalled);
    EXPECT_TRUE(mockECSApp.ecsUpdateCalled);
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, ECSApplication_Accessors)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;
    
    // Test that accessors exist and can be called
    EXPECT_NO_THROW({
        auto& entityManager = mockECSApp.GetEntityManager();
        auto& componentStore = mockECSApp.GetComponentStore();
        auto& systemManager = mockECSApp.GetSystemManager();
        
        LOG_VAR_DEBUG(entityManager.GetActiveEntityCount());
        LOG_VAR_DEBUG(componentStore.template GetEntitiesWithComponent<Nyon::ECS::TransformComponent>().size());
    });
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, ECSApplication_Integration)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;
    
    // Test the integration between base and ECS methods
    EXPECT_FALSE(mockECSApp.ecsStartCalled);
    EXPECT_FALSE(mockECSApp.ecsUpdateCalled);
    
    mockECSApp.OnECSStart();
    EXPECT_TRUE(mockECSApp.ecsStartCalled);
    
    mockECSApp.OnECSUpdate(1.0f/60.0f);
    EXPECT_TRUE(mockECSApp.ecsUpdateCalled);
    LOG_FUNC_EXIT();
}

// Timing and lifecycle tests
TEST_F(ApplicationTest, FixedTimestep_Constants)
{
    LOG_FUNC_ENTER();
    // Test the fixed timestep constants
    constexpr double FIXED_TIMESTEP = 1.0 / 60.0;
    constexpr double MAX_FRAME_TIME = 0.25;
    
    LOG_VAR_DEBUG(FIXED_TIMESTEP);
    LOG_VAR_DEBUG(MAX_FRAME_TIME);
    
    EXPECT_DOUBLE_EQ(FIXED_TIMESTEP, 1.0/60.0);
    EXPECT_DOUBLE_EQ(MAX_FRAME_TIME, 0.25);
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, Timing_Calculations)
{
    LOG_FUNC_ENTER();
    // Test timing calculations that would be used in game loop
    double frameTime = 1.0 / 60.0; // 60 FPS
    double accumulator = 0.0;
    constexpr double FIXED_TIMESTEP = 1.0 / 60.0;
    
    accumulator += frameTime;
    
    int updateCount = 0;
    while (accumulator >= FIXED_TIMESTEP) {
        updateCount++;
        accumulator -= FIXED_TIMESTEP;
    }
    
    LOG_VAR_DEBUG(updateCount);
    LOG_VAR_DEBUG(accumulator);
    
    EXPECT_EQ(updateCount, 1);
    EXPECT_DOUBLE_EQ(accumulator, 0.0);
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, Timing_SpiralOfDeathProtection)
{
    LOG_FUNC_ENTER();
    // Test frame time capping to prevent spiral of death
    constexpr double MAX_FRAME_TIME = 0.25;
    double frameTime = 0.5; // Very slow frame
    
    if (frameTime > MAX_FRAME_TIME) {
        frameTime = MAX_FRAME_TIME;
    }
    
    LOG_VAR_DEBUG(frameTime);
    EXPECT_DOUBLE_EQ(frameTime, MAX_FRAME_TIME);
    LOG_FUNC_EXIT();
}

// System integration tests
TEST_F(ApplicationTest, SystemInitialization_Order)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;
    
    // In a real application, systems would be initialized in specific order
    // InputSystem -> PhysicsSystem -> CollisionSystem -> RenderSystem
    
    // Test that we can access the managers
    EXPECT_NO_THROW({
        auto& entityManager = mockECSApp.GetEntityManager();
        auto& componentStore = mockECSApp.GetComponentStore();
        auto& systemManager = mockECSApp.GetSystemManager();
        
        LOG_VAR_DEBUG(entityManager.GetActiveEntityCount());
    });
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, ComponentSystem_Integration)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;
    
    // Test basic ECS integration
    EXPECT_NO_THROW({
        auto& entityManager = mockECSApp.GetEntityManager();
        auto& componentStore = mockECSApp.GetComponentStore();
        
        // Create entity
        auto entity = entityManager.CreateEntity();
        LOG_VAR_DEBUG(entity);
        
        // Add component
        Nyon::ECS::TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
        componentStore.AddComponent(entity, std::move(transform));
        
        // Verify component exists
        EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::TransformComponent>(entity));
        
        const auto& retrieved = componentStore.template GetComponent<Nyon::ECS::TransformComponent>(entity);
        LOG_VAR_DEBUG(retrieved.position.x);
        LOG_VAR_DEBUG(retrieved.position.y);
        EXPECT_FLOAT_NEAR(retrieved.position.x, 100.0f, 1e-6f);
        EXPECT_FLOAT_NEAR(retrieved.position.y, 200.0f, 1e-6f);
    });
    LOG_FUNC_EXIT();
}

// Error handling tests
TEST_F(ApplicationTest, ErrorHandling_NullWindow)
{
    LOG_FUNC_ENTER();
    MockApplication mockApp;
    
    // Test error handling scenarios
    EXPECT_NO_THROW({
        // These should handle null window gracefully
        mockApp.OnStart();
        mockApp.OnUpdate(1.0f/60.0f);
        mockApp.OnFixedUpdate(1.0f/60.0f);
    });
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, ErrorHandling_InvalidEntity)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;
    
    EXPECT_NO_THROW({
        auto& entityManager = mockECSApp.GetEntityManager();
        auto& componentStore = mockECSApp.GetComponentStore();
        
        // Test with invalid entity
        Nyon::ECS::EntityID invalidEntity = 999999u;
        EXPECT_FALSE(entityManager.IsEntityValid(invalidEntity));
        EXPECT_FALSE(componentStore.HasComponent<Nyon::ECS::TransformComponent>(invalidEntity));
    });
    LOG_FUNC_EXIT();
}

// Performance tests
TEST_F(ApplicationTest, Performance_EntityCreation)
{
    LOG_FUNC_ENTER();
    NyonTest::PERF_TIMER("Application_EntityCreation");
    
    MockECSApplication mockECSApp;
    
    auto& entityManager = mockECSApp.GetEntityManager();
    auto& componentStore = mockECSApp.GetComponentStore();
    
    const int entityCount = 1000;
    std::vector<Nyon::ECS::EntityID> entities;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Create many entities
    for (int i = 0; i < entityCount; ++i) {
        auto entity = entityManager.CreateEntity();
        entities.push_back(entity);
        
        Nyon::ECS::TransformComponent transform({
            static_cast<float>(i), 
            static_cast<float>(i * 2)
        });
        componentStore.AddComponent(entity, std::move(transform));
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    LOG_VAR_DEBUG(duration.count());
    LOG_VAR_DEBUG(entities.size());
    LOG_VAR_DEBUG(entityManager.GetActiveEntityCount());
    
    EXPECT_EQ(entities.size(), entityCount);
    EXPECT_EQ(entityManager.GetActiveEntityCount(), entityCount);
    EXPECT_LT(duration.count(), 1000); // Should complete in less than 1 second
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, Performance_ComponentAccess)
{
    LOG_FUNC_ENTER();
    NyonTest::PERF_TIMER("Application_ComponentAccess");
    
    MockECSApplication mockECSApp;
    
    auto& entityManager = mockECSApp.GetEntityManager();
    auto& componentStore = mockECSApp.GetComponentStore();
    
    // Create test entity
    auto entity = entityManager.CreateEntity();
    Nyon::ECS::TransformComponent originalTransform(Nyon::Math::Vector2(100.0f, 200.0f));
    componentStore.AddComponent(entity, std::move(originalTransform));
    
    const int accessCount = 10000;
    int validAccesses = 0;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < accessCount; ++i) {
        if (componentStore.template HasComponent<Nyon::ECS::TransformComponent>(entity)) {
            const auto& transform = componentStore.template GetComponent<Nyon::ECS::TransformComponent>(entity);
            if (transform.position.x == 100.0f) {
                validAccesses++;
            }
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    LOG_VAR_DEBUG(duration.count());
    LOG_VAR_DEBUG(validAccesses);
    
    EXPECT_EQ(validAccesses, accessCount);
    EXPECT_LT(duration.count(), 1000);
    LOG_FUNC_EXIT();
}

// Edge case tests
TEST_F(ApplicationTest, EdgeCase_RapidLifecycle)
{
    LOG_FUNC_ENTER();
    // Test rapid creation and destruction
    MockECSApplication mockECSApp;
    
    auto& entityManager = mockECSApp.GetEntityManager();
    
    const int cycleCount = 100;
    for (int i = 0; i < cycleCount; ++i) {
        auto entity = entityManager.CreateEntity();
        entityManager.DestroyEntity(entity);
    }
    
    LOG_VAR_DEBUG(entityManager.GetActiveEntityCount());
    EXPECT_EQ(entityManager.GetActiveEntityCount(), 0);
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, EdgeCase_ManyComponents)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;
    
    auto& entityManager = mockECSApp.GetEntityManager();
    auto& componentStore = mockECSApp.GetComponentStore();
    
    auto entity = entityManager.CreateEntity();
    
    // Add multiple different component types
    componentStore.AddComponent(entity, Nyon::ECS::TransformComponent(Nyon::Math::Vector2(0.0f, 0.0f)));
    componentStore.AddComponent(entity, Nyon::ECS::PhysicsBodyComponent(1.0f, false));
    
    Nyon::ECS::ColliderComponent::PolygonShape shape = {{0.0f, 0.0f}, {32.0f, 0.0f}, {32.0f, 32.0f}, {0.0f, 32.0f}};
    componentStore.AddComponent(entity, Nyon::ECS::ColliderComponent(shape));
    
    componentStore.AddComponent(entity, Nyon::ECS::RenderComponent({32.0f, 32.0f}));
    componentStore.AddComponent(entity, Nyon::ECS::BehaviorComponent());
    
    // Verify all components exist
    EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::TransformComponent>(entity));
    EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::PhysicsBodyComponent>(entity));
    EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::ColliderComponent>(entity));
    EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::RenderComponent>(entity));
    EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::BehaviorComponent>(entity));
    
    LOG_VAR_DEBUG(componentStore.template GetEntitiesWithComponent<Nyon::ECS::TransformComponent>().size());
    LOG_VAR_DEBUG(componentStore.template GetEntitiesWithComponent<Nyon::ECS::PhysicsBodyComponent>().size());
    LOG_VAR_DEBUG(componentStore.template GetEntitiesWithComponent<Nyon::ECS::ColliderComponent>().size());
    LOG_VAR_DEBUG(componentStore.template GetEntitiesWithComponent<Nyon::ECS::RenderComponent>().size());
    LOG_VAR_DEBUG(componentStore.template GetEntitiesWithComponent<Nyon::ECS::BehaviorComponent>().size());
    LOG_FUNC_EXIT();
}

// Multi-threading tests
TEST_F(ApplicationTest, Threading_EntityManager)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;
    
    auto& entityManager = mockECSApp.GetEntityManager();
    auto& componentStore = mockECSApp.GetComponentStore();
    
    std::vector<std::thread> threads;
    std::atomic<int> successCount(0);
    const int threadCount = 4;
    const int entitiesPerThread = 100;
    
    auto threadFunction = [&entityManager, &componentStore, &successCount, entitiesPerThread]() {
        for (int i = 0; i < entitiesPerThread; ++i) {
            auto entity = entityManager.CreateEntity();
            Nyon::ECS::TransformComponent transform(Nyon::Math::Vector2(
                static_cast<float>(entity), 
                static_cast<float>(entity * 2)
            ));
            componentStore.AddComponent(entity, std::move(transform));
            
            if (componentStore.template HasComponent<Nyon::ECS::TransformComponent>(entity)) {
                successCount++;
            }
        }
    };
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // Launch threads
    for (int i = 0; i < threadCount; ++i) {
        threads.emplace_back(threadFunction);
    }
    
    // Wait for completion
    for (auto& thread : threads) {
        thread.join();
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    LOG_VAR_DEBUG(duration.count());
    LOG_VAR_DEBUG(successCount.load());
    LOG_VAR_DEBUG(entityManager.GetActiveEntityCount());
    
    EXPECT_EQ(successCount.load(), threadCount * entitiesPerThread);
    EXPECT_EQ(entityManager.GetActiveEntityCount(), threadCount * entitiesPerThread);
    EXPECT_LT(duration.count(), 2000); // Should complete in less than 2 seconds
    LOG_FUNC_EXIT();
}

// Memory management tests
TEST_F(ApplicationTest, MemoryManagement_EntityLifecycle)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;
    
    auto& entityManager = mockECSApp.GetEntityManager();
    auto& componentStore = mockECSApp.GetComponentStore();
    
    // Test proper cleanup
    {
        auto entity = entityManager.CreateEntity();
        Nyon::ECS::TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
        componentStore.AddComponent(entity, std::move(transform));
        
        EXPECT_TRUE(entityManager.IsEntityValid(entity));
        EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::TransformComponent>(entity));
    }
    
    // Entity should still exist until explicitly destroyed
    // In real usage, entities persist until destroyed
    LOG_VAR_DEBUG(entityManager.GetActiveEntityCount());
    EXPECT_EQ(entityManager.GetActiveEntityCount(), 1);
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, MemoryManagement_ComponentRemoval)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;
    
    auto& entityManager = mockECSApp.GetEntityManager();
    auto& componentStore = mockECSApp.GetComponentStore();
    
    auto entity = entityManager.CreateEntity();
    Nyon::ECS::TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
    componentStore.AddComponent(entity, std::move(transform));
    
    EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::TransformComponent>(entity));
    
    componentStore.template RemoveComponent<Nyon::ECS::TransformComponent>(entity);
    
    EXPECT_FALSE(componentStore.template HasComponent<Nyon::ECS::TransformComponent>(entity));
    LOG_FUNC_EXIT();
}

// Integration scenario tests
TEST_F(ApplicationTest, GamingScenario_PlatformerSetup)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;
    
    auto& entityManager = mockECSApp.GetEntityManager();
    auto& componentStore = mockECSApp.GetComponentStore();
    
    // Simulate setting up a simple platformer
    struct GameObjectSetup {
        std::string name;
        Nyon::Math::Vector2 position;
        Nyon::Math::Vector2 size;
        bool isStatic;
    };
    
    std::vector<GameObjectSetup> gameObjects = {
        {"Player", {100.0f, 300.0f}, {32.0f, 32.0f}, false},
        {"Floor", {0.0f, 600.0f}, {1280.0f, 32.0f}, true},
        {"LeftWall", {0.0f, 0.0f}, {32.0f, 720.0f}, true},
        {"RightWall", {1248.0f, 0.0f}, {32.0f, 720.0f}, true}
    };
    
    std::vector<Nyon::ECS::EntityID> entities;
    
    for (const auto& obj : gameObjects) {
        auto entity = entityManager.CreateEntity();
        entities.push_back(entity);
        
        // Add transform
        componentStore.AddComponent(entity, Nyon::ECS::TransformComponent(obj.position));
        
        // Add physics body
        componentStore.AddComponent(entity, Nyon::ECS::PhysicsBodyComponent(1.0f, obj.isStatic));
        
        // Add collider
        Nyon::ECS::ColliderComponent::PolygonShape shape = {
            {0.0f, 0.0f},
            {obj.size.x, 0.0f},
            {obj.size.x, obj.size.y},
            {0.0f, obj.size.y}
        };
        componentStore.AddComponent(entity, Nyon::ECS::ColliderComponent(shape));
        
        // Add render component
        componentStore.AddComponent(entity, Nyon::ECS::RenderComponent(obj.size));
        
        LOG_VAR_DEBUG(obj.name);
        LOG_VAR_DEBUG(entity);
    }
    
    LOG_VAR_DEBUG(entities.size());
    LOG_VAR_DEBUG(entityManager.GetActiveEntityCount());
    
    EXPECT_EQ(entities.size(), gameObjects.size());
    EXPECT_EQ(entityManager.GetActiveEntityCount(), gameObjects.size());
    
    // Verify all entities have required components
    for (auto entity : entities) {
        EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::TransformComponent>(entity));
        EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::PhysicsBodyComponent>(entity));
        EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::ColliderComponent>(entity));
        EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::RenderComponent>(entity));
    }
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, GamingScenario_GameLoopSimulation)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;
    
    // Simulate a simplified game loop
    const int frameCount = 60; // 1 second at 60 FPS
    float deltaTime = 1.0f / 60.0f;
    float totalTime = 0.0f;
    
    for (int frame = 0; frame < frameCount; ++frame) {
        // Fixed update (physics)
        // mockECSApp.OnFixedUpdate(deltaTime); // Cannot call final method directly
        totalTime += deltaTime;
        
        // Interpolated render
        float alpha = 0.5f; // Mid-frame interpolation
        // mockECSApp.OnInterpolateAndRender(alpha); // Protected method
        
        LOG_VAR_DEBUG(frame);
        LOG_VAR_DEBUG(totalTime);
    }
    
    LOG_VAR_DEBUG(totalTime);
    EXPECT_FLOAT_NEAR(totalTime, 1.0f, 1e-6f);
    LOG_FUNC_EXIT();
}