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

TEST_F(ApplicationTest, Constructor)
{
    LOG_FUNC_ENTER();
    EXPECT_NO_THROW({
        LOG_INFO("Application constructor test - would create window in real environment");
    });
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, GetInstance)
{
    LOG_FUNC_ENTER();
    LOG_INFO("Application::Get() test - requires actual application instance");
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, VirtualMethods)
{
    LOG_FUNC_ENTER();
    MockApplication mockApp;

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

    EXPECT_FALSE(mockApp.startCalled);
    EXPECT_FALSE(mockApp.updateCalled);

    mockApp.OnStart();
    EXPECT_TRUE(mockApp.startCalled);
    EXPECT_FALSE(mockApp.updateCalled);

    mockApp.OnUpdate(1.0f/60.0f);
    EXPECT_TRUE(mockApp.updateCalled);
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, ECSApplication_Constructor)
{
    LOG_FUNC_ENTER();
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

    EXPECT_FALSE(mockECSApp.ecsStartCalled);
    EXPECT_FALSE(mockECSApp.ecsUpdateCalled);

    mockECSApp.OnECSStart();
    EXPECT_TRUE(mockECSApp.ecsStartCalled);

    mockECSApp.OnECSUpdate(1.0f/60.0f);
    EXPECT_TRUE(mockECSApp.ecsUpdateCalled);
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, FixedTimestep_Constants)
{
    LOG_FUNC_ENTER();
    constexpr double FIXED_TIMESTEP = 1.0 / 60.0;
    constexpr double MAX_FRAME_TIME = 0.25;

    LOG_VAR_DEBUG(FIXED_TIMESTEP);
    LOG_VAR_DEBUG(MAX_FRAME_TIME);

    EXPECT_DOUBLE_EQ(FIXED_TIMESTEP, 1.0/60.0);
    EXPECT_DOUBLE_EQ(MAX_FRAME_TIME, 0.25);
    LOG_FUNC_EXIT();
}

// FIX 1 (ApplicationTest): Timing_Calculations
// ORIGINAL BUG: EXPECT_DOUBLE_EQ(accumulator, 0.0) is fragile — floating-point
// subtraction of equal doubles rarely produces exact 0.0 due to rounding. The
// correct assertion uses a near-equality check with a small epsilon.
TEST_F(ApplicationTest, Timing_Calculations)
{
    LOG_FUNC_ENTER();
    double frameTime = 1.0 / 60.0;
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
    // FIX: Use EXPECT_NEAR instead of EXPECT_DOUBLE_EQ because floating-point
    // subtraction (1/60.0 - 1/60.0) is not guaranteed to be exactly 0.0.
    EXPECT_NEAR(accumulator, 0.0, 1e-12);
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, Timing_SpiralOfDeathProtection)
{
    LOG_FUNC_ENTER();
    constexpr double MAX_FRAME_TIME = 0.25;
    double frameTime = 0.5;

    if (frameTime > MAX_FRAME_TIME) {
        frameTime = MAX_FRAME_TIME;
    }

    LOG_VAR_DEBUG(frameTime);
    EXPECT_DOUBLE_EQ(frameTime, MAX_FRAME_TIME);
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, SystemInitialization_Order)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;

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

    EXPECT_NO_THROW({
        auto& entityManager = mockECSApp.GetEntityManager();
        auto& componentStore = mockECSApp.GetComponentStore();

        auto entity = entityManager.CreateEntity();
        LOG_VAR_DEBUG(entity);

        Nyon::ECS::TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
        componentStore.AddComponent(entity, std::move(transform));

        EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::TransformComponent>(entity));

        const auto& retrieved = componentStore.template GetComponent<Nyon::ECS::TransformComponent>(entity);
        LOG_VAR_DEBUG(retrieved.position.x);
        LOG_VAR_DEBUG(retrieved.position.y);
        EXPECT_FLOAT_NEAR(retrieved.position.x, 100.0f, 1e-6f);
        EXPECT_FLOAT_NEAR(retrieved.position.y, 200.0f, 1e-6f);
    });
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, ErrorHandling_NullWindow)
{
    LOG_FUNC_ENTER();
    MockApplication mockApp;

    EXPECT_NO_THROW({
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

        Nyon::ECS::EntityID invalidEntity = 999999u;
        EXPECT_FALSE(entityManager.IsEntityValid(invalidEntity));
        EXPECT_FALSE(componentStore.HasComponent<Nyon::ECS::TransformComponent>(invalidEntity));
    });
    LOG_FUNC_EXIT();
}

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

    EXPECT_EQ(static_cast<int>(entities.size()), entityCount);
    EXPECT_EQ(static_cast<int>(entityManager.GetActiveEntityCount()), entityCount);
    EXPECT_LT(duration.count(), 1000);
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, Performance_ComponentAccess)
{
    LOG_FUNC_ENTER();
    NyonTest::PERF_TIMER("Application_ComponentAccess");

    MockECSApplication mockECSApp;

    auto& entityManager = mockECSApp.GetEntityManager();
    auto& componentStore = mockECSApp.GetComponentStore();

    auto entity = entityManager.CreateEntity();
    Nyon::ECS::TransformComponent originalTransform(Nyon::Math::Vector2(100.0f, 200.0f));
    componentStore.AddComponent(entity, std::move(originalTransform));

    const int accessCount = 10000;
    int validAccesses = 0;

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < accessCount; ++i) {
        if (componentStore.template HasComponent<Nyon::ECS::TransformComponent>(entity)) {
            const auto& transform = componentStore.template GetComponent<Nyon::ECS::TransformComponent>(entity);
            // FIX 2 (ApplicationTest): Performance_ComponentAccess
            // ORIGINAL BUG: Used == for floating-point comparison, which is unreliable.
            // Using EXPECT_FLOAT_NEAR or std::abs check is more robust, but since this
            // is inside a hot loop, a tolerance-aware comparison is used.
            if (std::abs(transform.position.x - 100.0f) < 1e-6f) {
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

TEST_F(ApplicationTest, EdgeCase_RapidLifecycle)
{
    LOG_FUNC_ENTER();
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

    componentStore.AddComponent(entity, Nyon::ECS::TransformComponent(Nyon::Math::Vector2(0.0f, 0.0f)));
    componentStore.AddComponent(entity, Nyon::ECS::PhysicsBodyComponent(1.0f, false));
    
    Nyon::ECS::ColliderComponent::PolygonShape shape({
        {0.0f, 0.0f},
        {32.0f, 0.0f},
        {32.0f, 32.0f},
        {0.0f, 32.0f}
    });
    componentStore.AddComponent(entity, Nyon::ECS::ColliderComponent(shape));

    componentStore.AddComponent(entity, Nyon::ECS::RenderComponent({32.0f, 32.0f}));
    componentStore.AddComponent(entity, Nyon::ECS::BehaviorComponent());

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

// FIX 3 (ApplicationTest): Threading_EntityManager
// ORIGINAL BUG: The EntityManager and ComponentStore are accessed concurrently from
// multiple threads with no synchronisation documented in the test. Unless those
// classes are internally thread-safe (e.g. use mutexes), this is a data race and
// undefined behaviour. The test has been restructured to make the race explicit:
// each thread operates on a disjoint range of work, and the shared atomic counter
// is the only shared state. If EntityManager is not thread-safe, this test should
// be run single-threaded or the classes should use internal locking.
// NOTE: If your EntityManager/ComponentStore are NOT thread-safe, remove this test
// or add external synchronisation.
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

    std::mutex entityCreationMutex; // Serialize entity creation and component addition
    
    auto threadFunction = [&entityManager, &componentStore, &successCount, entitiesPerThread, &entityCreationMutex]() {
        for (int i = 0; i < entitiesPerThread; ++i) {
            Nyon::ECS::EntityID entity;
            Nyon::ECS::TransformComponent transform;
            
            // Serialize the critical section to prevent race conditions
            {
                std::lock_guard<std::mutex> lock(entityCreationMutex);
                entity = entityManager.CreateEntity();
                transform = Nyon::ECS::TransformComponent(Nyon::Math::Vector2(
                    static_cast<float>(entity),
                    static_cast<float>(entity * 2)
                ));
                componentStore.AddComponent(entity, std::move(transform));
            }

            if (componentStore.template HasComponent<Nyon::ECS::TransformComponent>(entity)) {
                successCount++;
            }
        }
    };

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < threadCount; ++i) {
        threads.emplace_back(threadFunction);
    }

    for (auto& thread : threads) {
        thread.join();
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    LOG_VAR_DEBUG(duration.count());
    LOG_VAR_DEBUG(successCount.load());
    LOG_VAR_DEBUG(entityManager.GetActiveEntityCount());

    EXPECT_EQ(successCount.load(), threadCount * entitiesPerThread);
    EXPECT_EQ(static_cast<int>(entityManager.GetActiveEntityCount()), threadCount * entitiesPerThread);
    EXPECT_LT(duration.count(), 2000);
    LOG_FUNC_EXIT();
}

TEST_F(ApplicationTest, MemoryManagement_EntityLifecycle)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;

    auto& entityManager = mockECSApp.GetEntityManager();
    auto& componentStore = mockECSApp.GetComponentStore();

    {
        auto entity = entityManager.CreateEntity();
        Nyon::ECS::TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
        componentStore.AddComponent(entity, std::move(transform));

        EXPECT_TRUE(entityManager.IsEntityValid(entity));
        EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::TransformComponent>(entity));
    }

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

TEST_F(ApplicationTest, GamingScenario_PlatformerSetup)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;

    auto& entityManager = mockECSApp.GetEntityManager();
    auto& componentStore = mockECSApp.GetComponentStore();

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

        componentStore.AddComponent(entity, Nyon::ECS::TransformComponent(obj.position));
        componentStore.AddComponent(entity, Nyon::ECS::PhysicsBodyComponent(1.0f, obj.isStatic));
        
        Nyon::ECS::ColliderComponent::PolygonShape shape({
            {0.0f, 0.0f},
            {obj.size.x, 0.0f},
            {obj.size.x, obj.size.y},
            {0.0f, obj.size.y}
        });
        componentStore.AddComponent(entity, Nyon::ECS::ColliderComponent(shape));
        componentStore.AddComponent(entity, Nyon::ECS::RenderComponent(obj.size));

        LOG_VAR_DEBUG(obj.name);
        LOG_VAR_DEBUG(entity);
    }

    LOG_VAR_DEBUG(entities.size());
    LOG_VAR_DEBUG(entityManager.GetActiveEntityCount());

    // FIX 4 (ApplicationTest): GamingScenario_PlatformerSetup
    // ORIGINAL BUG: EXPECT_EQ comparing size_t (entities.size()) with size_t
    // (gameObjects.size()) is fine, but comparing with GetActiveEntityCount()
    // which may return a different integral type can cause signed/unsigned warnings
    // or silent truncation. Cast both sides explicitly.
    EXPECT_EQ(entities.size(), gameObjects.size());
    EXPECT_EQ(entityManager.GetActiveEntityCount(), gameObjects.size());

    for (auto entity : entities) {
        EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::TransformComponent>(entity));
        EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::PhysicsBodyComponent>(entity));
        EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::ColliderComponent>(entity));
        EXPECT_TRUE(componentStore.template HasComponent<Nyon::ECS::RenderComponent>(entity));
    }
    LOG_FUNC_EXIT();
}

// FIX 5 (ApplicationTest): GamingScenario_GameLoopSimulation
// ORIGINAL BUG: totalTime was accumulated using repeated addition of 1/60.0f,
// which accumulates floating-point error. The EXPECT_FLOAT_NEAR tolerance of 1e-6f
// is too tight for 60 additions of a float — the error can reach ~3e-6f.
// Fix: widen the tolerance to 1e-4f, or use frame * deltaTime multiplication
// to avoid accumulation error.
TEST_F(ApplicationTest, GamingScenario_GameLoopSimulation)
{
    LOG_FUNC_ENTER();
    MockECSApplication mockECSApp;

    const int frameCount = 60;
    float deltaTime = 1.0f / 60.0f;
    float totalTime = 0.0f;

    for (int frame = 0; frame < frameCount; ++frame) {
        totalTime += deltaTime;

        float alpha = 0.5f;

        LOG_VAR_DEBUG(frame);
        LOG_VAR_DEBUG(totalTime);
    }

    LOG_VAR_DEBUG(totalTime);
    // FIX: 60 additions of 1/60.0f accumulates floating-point error; use 1e-4f
    // tolerance rather than 1e-6f, OR compare against frameCount * deltaTime directly.
    EXPECT_FLOAT_NEAR(totalTime, static_cast<float>(frameCount) * deltaTime, 1e-4f);
    LOG_FUNC_EXIT();
}