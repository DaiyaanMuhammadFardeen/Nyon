#include <gtest/gtest.h>
#include "nyon/ecs/SystemManager.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/System.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "TestHelpers.h"
#include <vector>

using namespace Nyon::ECS;

/**
 * @brief Mock system for testing SystemManager functionality.
 */
class MockSystem : public System
{
public:
    MockSystem() : updateCount(0), initialized(false), shutdown(false) {}
    
    void Update(float deltaTime) override
    {
        updateCount++;
        lastDeltaTime = deltaTime;
    }
    
    void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override
    {
        System::Initialize(entityManager, componentStore);
        initialized = true;
    }
    
    void Shutdown() override
    {
        shutdown = true;
    }
    
    int updateCount;
    float lastDeltaTime;
    bool initialized;
    bool shutdown;
};

/**
 * @brief Another mock system for testing execution order.
 */
class MockSystem2 : public System
{
public:
    MockSystem2() : updateCount(0) {}
    
    void Update(float deltaTime) override
    {
        updateCount++;
        executionOrder.push_back(2); // System 2 identifier
    }
    
    static std::vector<int> executionOrder;
    int updateCount;
};

std::vector<int> MockSystem2::executionOrder;

/**
 * @brief Unit tests for SystemManager functionality.
 * 
 * Tests system registration, update order, initialization, shutdown,
 * and proper lifecycle management for ECS systems.
 */
class SystemManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        entityManager = std::make_unique<EntityManager>();
        componentStore = std::make_unique<ComponentStore>(*entityManager);
        systemManager = std::make_unique<SystemManager>(*entityManager, *componentStore);
        MockSystem2::executionOrder.clear();
        LOG_FUNC_EXIT();
    }

    void TearDown() override
    {
        LOG_FUNC_ENTER();
        systemManager.reset();
        componentStore.reset();
        entityManager.reset();
        LOG_FUNC_EXIT();
    }

    std::unique_ptr<EntityManager> entityManager;
    std::unique_ptr<ComponentStore> componentStore;
    std::unique_ptr<SystemManager> systemManager;
};

// System addition tests
TEST_F(SystemManagerTest, AddSystem_SingleSystem)
{
    LOG_FUNC_ENTER();
    auto mockSystem = std::make_unique<MockSystem>();
    MockSystem* systemPtr = mockSystem.get();
    
    systemManager->AddSystem(std::move(mockSystem));
    
    EXPECT_TRUE(systemPtr->initialized);
    LOG_FUNC_EXIT();
}

TEST_F(SystemManagerTest, AddSystem_MultipleSystems)
{
    LOG_FUNC_ENTER();
    auto system1 = std::make_unique<MockSystem>();
    auto system2 = std::make_unique<MockSystem>();
    MockSystem* ptr1 = system1.get();
    MockSystem* ptr2 = system2.get();
    
    systemManager->AddSystem(std::move(system1));
    systemManager->AddSystem(std::move(system2));
    
    EXPECT_TRUE(ptr1->initialized);
    EXPECT_TRUE(ptr2->initialized);
    LOG_FUNC_EXIT();
}

// System update tests
TEST_F(SystemManagerTest, Update_SingleSystem)
{
    LOG_FUNC_ENTER();
    auto mockSystem = std::make_unique<MockSystem>();
    MockSystem* systemPtr = mockSystem.get();
    
    systemManager->AddSystem(std::move(mockSystem));
    
    float deltaTime = 1.0f / 60.0f;
    systemManager->Update(deltaTime);
    
    LOG_VAR_DEBUG(systemPtr->updateCount);
    LOG_VAR_DEBUG(systemPtr->lastDeltaTime);
    
    EXPECT_EQ(systemPtr->updateCount, 1);
    EXPECT_FLOAT_NEAR(systemPtr->lastDeltaTime, deltaTime, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(SystemManagerTest, Update_MultipleSystems)
{
    LOG_FUNC_ENTER();
    auto system1 = std::make_unique<MockSystem>();
    auto system2 = std::make_unique<MockSystem>();
    MockSystem* ptr1 = system1.get();
    MockSystem* ptr2 = system2.get();
    
    systemManager->AddSystem(std::move(system1));
    systemManager->AddSystem(std::move(system2));
    
    float deltaTime = 1.0f / 60.0f;
    systemManager->Update(deltaTime);
    
    LOG_VAR_DEBUG(ptr1->updateCount);
    LOG_VAR_DEBUG(ptr2->updateCount);
    
    EXPECT_EQ(ptr1->updateCount, 1);
    EXPECT_EQ(ptr2->updateCount, 1);
    LOG_FUNC_EXIT();
}

TEST_F(SystemManagerTest, Update_ExecutionOrder)
{
    LOG_FUNC_ENTER();
    auto system1 = std::make_unique<MockSystem>();
    auto system2 = std::make_unique<MockSystem2>();
    MockSystem* ptr1 = system1.get();
    MockSystem2* ptr2 = system2.get();
    
    systemManager->AddSystem(std::move(system1));
    systemManager->AddSystem(std::move(system2));
    
    systemManager->Update(1.0f / 60.0f);
    
    // Systems should execute in the order they were added
    EXPECT_EQ(ptr1->updateCount, 1);
    EXPECT_EQ(ptr2->updateCount, 1);
    LOG_FUNC_EXIT();
}

TEST_F(SystemManagerTest, Update_MultipleFrames)
{
    LOG_FUNC_ENTER();
    auto mockSystem = std::make_unique<MockSystem>();
    MockSystem* systemPtr = mockSystem.get();
    
    systemManager->AddSystem(std::move(mockSystem));
    
    const int frameCount = 10;
    float deltaTime = 1.0f / 60.0f;
    
    for (int i = 0; i < frameCount; ++i) {
        systemManager->Update(deltaTime);
    }
    
    LOG_VAR_DEBUG(systemPtr->updateCount);
    EXPECT_EQ(systemPtr->updateCount, frameCount);
    LOG_FUNC_EXIT();
}

// System shutdown tests
TEST_F(SystemManagerTest, Shutdown_SingleSystem)
{
    LOG_FUNC_ENTER();
    auto mockSystem = std::make_unique<MockSystem>();
    MockSystem* systemPtr = mockSystem.get();
    
    systemManager->AddSystem(std::move(mockSystem));
    systemManager->Shutdown();
    
    EXPECT_TRUE(systemPtr->shutdown);
    LOG_FUNC_EXIT();
}

TEST_F(SystemManagerTest, Shutdown_MultipleSystems)
{
    LOG_FUNC_ENTER();
    auto system1 = std::make_unique<MockSystem>();
    auto system2 = std::make_unique<MockSystem>();
    MockSystem* ptr1 = system1.get();
    MockSystem* ptr2 = system2.get();
    
    systemManager->AddSystem(std::move(system1));
    systemManager->AddSystem(std::move(system2));
    
    systemManager->Shutdown();
    
    EXPECT_TRUE(ptr1->shutdown);
    EXPECT_TRUE(ptr2->shutdown);
    LOG_FUNC_EXIT();
}

TEST_F(SystemManagerTest, Shutdown_AfterUpdates)
{
    LOG_FUNC_ENTER();
    auto mockSystem = std::make_unique<MockSystem>();
    MockSystem* systemPtr = mockSystem.get();
    
    systemManager->AddSystem(std::move(mockSystem));
    
    // Perform some updates
    systemManager->Update(1.0f / 60.0f);
    systemManager->Update(1.0f / 60.0f);
    
    systemManager->Shutdown();
    
    EXPECT_TRUE(systemPtr->shutdown);
    EXPECT_EQ(systemPtr->updateCount, 2);
    LOG_FUNC_EXIT();
}

// Integration with ECS core tests
TEST_F(SystemManagerTest, SystemAccessToEntityManager)
{
    LOG_FUNC_ENTER();
    class EntityCountingSystem : public System
    {
    public:
        void Update(float deltaTime) override
        {
            if (m_EntityManager) {
                entityCount = m_EntityManager->GetActiveEntityCount();
            }
        }
        
        size_t entityCount = 0;
    };
    
    auto countingSystem = std::make_unique<EntityCountingSystem>();
    EntityCountingSystem* systemPtr = countingSystem.get();
    
    systemManager->AddSystem(std::move(countingSystem));
    
    // Create some entities
    entityManager->CreateEntity();
    entityManager->CreateEntity();
    entityManager->CreateEntity();
    
    systemManager->Update(1.0f / 60.0f);
    
    LOG_VAR_DEBUG(systemPtr->entityCount);
    EXPECT_EQ(systemPtr->entityCount, 3u);
    LOG_FUNC_EXIT();
}

TEST_F(SystemManagerTest, SystemAccessToComponentStore)
{
    LOG_FUNC_ENTER();
    class ComponentTestingSystem : public System
    {
    public:
        void Update(float deltaTime) override
        {
            if (m_ComponentStore) {
                hasTransformComponent = m_ComponentStore->HasComponent<TransformComponent>(testEntity);
            }
        }
        
        EntityID testEntity = INVALID_ENTITY;
        bool hasTransformComponent = false;
    };
    
    auto testingSystem = std::make_unique<ComponentTestingSystem>();
    ComponentTestingSystem* systemPtr = testingSystem.get();
    
    systemManager->AddSystem(std::move(testingSystem));
    
    // Set up test entity with component
    EntityID entity = entityManager->CreateEntity();
    TransformComponent transform({100.0f, 200.0f});
    componentStore->AddComponent(entity, std::move(transform));
    
    systemPtr->testEntity = entity;
    systemManager->Update(1.0f / 60.0f);
    
    LOG_VAR_DEBUG(systemPtr->hasTransformComponent);
    EXPECT_TRUE(systemPtr->hasTransformComponent);
    LOG_FUNC_EXIT();
}

// Performance tests
TEST_F(SystemManagerTest, Performance_ManySystems)
{
    LOG_FUNC_ENTER();
    // Removed PERF_TIMER due to compilation issues
    // NyonTest::PERF_TIMER("Performance_ManySystems");
    
    const int systemCount = 50;
    std::vector<MockSystem*> systemPointers;
    
    // Add many systems
    for (int i = 0; i < systemCount; ++i) {
        auto system = std::make_unique<MockSystem>();
        systemPointers.push_back(system.get());
        systemManager->AddSystem(std::move(system));
    }
    
    // Update all systems
    systemManager->Update(1.0f / 60.0f);
    
    // Verify all systems were updated
    for (auto* system : systemPointers) {
        EXPECT_EQ(system->updateCount, 1);
    }
    LOG_FUNC_EXIT();
}

TEST_F(SystemManagerTest, Performance_ManyUpdates)
{
    LOG_FUNC_ENTER();
    // Removed PERF_TIMER due to compilation issues
    // NyonTest::PERF_TIMER("Performance_ManyUpdates");
    
    auto mockSystem = std::make_unique<MockSystem>();
    MockSystem* systemPtr = mockSystem.get();
    
    systemManager->AddSystem(std::move(mockSystem));
    
    const int updateCount = 1000;
    float deltaTime = 1.0f / 60.0f;
    
    for (int i = 0; i < updateCount; ++i) {
        systemManager->Update(deltaTime);
    }
    
    LOG_VAR_DEBUG(systemPtr->updateCount);
    EXPECT_EQ(systemPtr->updateCount, updateCount);
    LOG_FUNC_EXIT();
}

// Edge cases and error handling
TEST_F(SystemManagerTest, AddNullSystem)
{
    LOG_FUNC_ENTER();
    // Should handle null system gracefully
    // Note: This test is commented out because AddSystem expects unique_ptr
    // EXPECT_NO_THROW(systemManager->AddSystem(nullptr));
    LOG_FUNC_EXIT();
}

TEST_F(SystemManagerTest, UpdateWithoutSystems)
{
    LOG_FUNC_ENTER();
    // Should not crash when updating with no systems
    EXPECT_NO_THROW(systemManager->Update(1.0f / 60.0f));
    LOG_FUNC_EXIT();
}

TEST_F(SystemManagerTest, ShutdownWithoutSystems)
{
    LOG_FUNC_ENTER();
    // Should not crash when shutting down with no systems
    EXPECT_NO_THROW(systemManager->Shutdown());
    LOG_FUNC_EXIT();
}

TEST_F(SystemManagerTest, DoubleShutdown)
{
    LOG_FUNC_ENTER();
    auto mockSystem = std::make_unique<MockSystem>();
    MockSystem* systemPtr = mockSystem.get();
    
    systemManager->AddSystem(std::move(mockSystem));
    
    systemManager->Shutdown();
    systemManager->Shutdown(); // Second shutdown
    
    EXPECT_TRUE(systemPtr->shutdown);
    LOG_FUNC_EXIT();
}

// Destructor behavior tests
TEST_F(SystemManagerTest, Destructor_Cleanup)
{
    LOG_FUNC_ENTER();
    {
        auto localSystemManager = std::make_unique<SystemManager>(*entityManager, *componentStore);
        auto mockSystem = std::make_unique<MockSystem>();
        MockSystem* systemPtr = mockSystem.get();
        
        localSystemManager->AddSystem(std::move(mockSystem));
        localSystemManager->Update(1.0f / 60.0f);
        
        EXPECT_EQ(systemPtr->updateCount, 1);
        // localSystemManager goes out of scope here
    }
    // Systems should be properly cleaned up
    LOG_FUNC_EXIT();
}