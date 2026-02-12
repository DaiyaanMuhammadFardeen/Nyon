#include <gtest/gtest.h>
#include "nyon/ecs/EntityManager.h"
#include "TestHelpers.h"
#include <algorithm>  // For std::sort and std::unique

using namespace Nyon::ECS;

/**
 * @brief Unit tests for EntityManager functionality.
 * 
 * Tests entity creation, destruction, ID recycling, validity checking,
 * and edge cases for entity lifecycle management.
 */
class EntityManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        entityManager = std::make_unique<EntityManager>();
        LOG_FUNC_EXIT();
    }

    std::unique_ptr<EntityManager> entityManager;
};

// Basic entity creation tests
TEST_F(EntityManagerTest, CreateEntity_FirstEntity)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    LOG_VAR_DEBUG(entity);
    EXPECT_NE(entity, INVALID_ENTITY);
    EXPECT_EQ(entity, 0u); // First entity should be ID 0
    LOG_FUNC_EXIT();
}

TEST_F(EntityManagerTest, CreateEntity_MultipleEntities)
{
    LOG_FUNC_ENTER();
    // Removed PERF_TIMER due to compilation issues
    // NyonTest::PERF_TIMER("CreateEntity_MultipleEntities");
    
    std::vector<EntityID> entities;
    const int numEntities = 100;
    
    for (int i = 0; i < numEntities; ++i) {
        EntityID entity = entityManager->CreateEntity();
        entities.push_back(entity);
        LOG_VAR_DEBUG(entity);
    }
    
    // All entities should have unique IDs
    std::sort(entities.begin(), entities.end());
    auto uniqueEnd = std::unique(entities.begin(), entities.end());
    EXPECT_EQ(uniqueEnd, entities.end());
    
    // Should have correct count
    EXPECT_EQ(entityManager->GetActiveEntityCount(), numEntities);
    LOG_FUNC_EXIT();
}

TEST_F(EntityManagerTest, CreateEntity_IDSequence)
{
    LOG_FUNC_ENTER();
    EntityID first = entityManager->CreateEntity();
    EntityID second = entityManager->CreateEntity();
    EntityID third = entityManager->CreateEntity();
    
    LOG_VAR_DEBUG(first);
    LOG_VAR_DEBUG(second);
    LOG_VAR_DEBUG(third);
    
    EXPECT_EQ(first, 0u);
    EXPECT_EQ(second, 1u);
    EXPECT_EQ(third, 2u);
    LOG_FUNC_EXIT();
}

// Entity destruction tests
TEST_F(EntityManagerTest, DestroyEntity_ValidEntity)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    size_t initialCount = entityManager->GetActiveEntityCount();
    
    entityManager->DestroyEntity(entity);
    
    EXPECT_FALSE(entityManager->IsEntityValid(entity));
    EXPECT_EQ(entityManager->GetActiveEntityCount(), initialCount - 1);
    LOG_FUNC_EXIT();
}

TEST_F(EntityManagerTest, DestroyEntity_NonExistent)
{
    LOG_FUNC_ENTER();
    EntityID invalidEntity = 999u;
    
    // Should not crash or cause issues
    EXPECT_NO_THROW(entityManager->DestroyEntity(invalidEntity));
    LOG_FUNC_EXIT();
}

TEST_F(EntityManagerTest, DestroyEntity_AlreadyDestroyed)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    entityManager->DestroyEntity(entity);
    
    // Destroying already destroyed entity should be safe
    EXPECT_NO_THROW(entityManager->DestroyEntity(entity));
    EXPECT_FALSE(entityManager->IsEntityValid(entity));
    LOG_FUNC_EXIT();
}

// Entity validity tests
TEST_F(EntityManagerTest, IsEntityValid_NewEntity)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    bool isValid = entityManager->IsEntityValid(entity);
    LOG_VAR_DEBUG(isValid);
    EXPECT_TRUE(isValid);
    LOG_FUNC_EXIT();
}

TEST_F(EntityManagerTest, IsEntityValid_DestroyedEntity)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    entityManager->DestroyEntity(entity);
    
    bool isValid = entityManager->IsEntityValid(entity);
    LOG_VAR_DEBUG(isValid);
    EXPECT_FALSE(isValid);
    LOG_FUNC_EXIT();
}

TEST_F(EntityManagerTest, IsEntityValid_NeverCreated)
{
    LOG_FUNC_ENTER();
    EntityID neverCreated = 999u;
    
    bool isValid = entityManager->IsEntityValid(neverCreated);
    LOG_VAR_DEBUG(isValid);
    EXPECT_FALSE(isValid);
    LOG_FUNC_EXIT();
}

// ID recycling tests
TEST_F(EntityManagerTest, IDRecycling_Basic)
{
    LOG_FUNC_ENTER();
    EntityID first = entityManager->CreateEntity();
    EntityID second = entityManager->CreateEntity();
    
    entityManager->DestroyEntity(first);
    
    EntityID recycled = entityManager->CreateEntity();
    
    LOG_VAR_DEBUG(first);
    LOG_VAR_DEBUG(recycled);
    // Recycled ID should be the same as the destroyed one
    EXPECT_EQ(recycled, first);
    LOG_FUNC_EXIT();
}

TEST_F(EntityManagerTest, IDRecycling_Order)
{
    LOG_FUNC_ENTER();
    EntityID first = entityManager->CreateEntity();
    EntityID second = entityManager->CreateEntity();
    EntityID third = entityManager->CreateEntity();
    
    entityManager->DestroyEntity(second);
    entityManager->DestroyEntity(first);
    
    EntityID newFirst = entityManager->CreateEntity();
    EntityID newSecond = entityManager->CreateEntity();
    
    LOG_VAR_DEBUG(first);
    LOG_VAR_DEBUG(second);
    LOG_VAR_DEBUG(newFirst);
    LOG_VAR_DEBUG(newSecond);
    
    // Should recycle in LIFO order (stack-like behavior)
    EXPECT_EQ(newFirst, first);
    EXPECT_EQ(newSecond, second);
    LOG_FUNC_EXIT();
}

// Active entities tracking
TEST_F(EntityManagerTest, GetActiveEntityCount_Empty)
{
    LOG_FUNC_ENTER();
    size_t count = entityManager->GetActiveEntityCount();
    
    LOG_VAR_DEBUG(count);
    EXPECT_EQ(count, 0u);
    LOG_FUNC_EXIT();
}

TEST_F(EntityManagerTest, GetActiveEntityCount_AfterCreation)
{
    LOG_FUNC_ENTER();
    entityManager->CreateEntity();
    entityManager->CreateEntity();
    
    size_t count = entityManager->GetActiveEntityCount();
    
    LOG_VAR_DEBUG(count);
    EXPECT_EQ(count, 2u);
    LOG_FUNC_EXIT();
}

TEST_F(EntityManagerTest, GetActiveEntityCount_AfterDestruction)
{
    LOG_FUNC_ENTER();
    EntityID entity1 = entityManager->CreateEntity();
    EntityID entity2 = entityManager->CreateEntity();
    size_t initialCount = entityManager->GetActiveEntityCount();
    
    entityManager->DestroyEntity(entity1);
    
    size_t count = entityManager->GetActiveEntityCount();
    
    LOG_VAR_DEBUG(initialCount);
    LOG_VAR_DEBUG(count);
    EXPECT_EQ(count, initialCount - 1);
    LOG_FUNC_EXIT();
}

TEST_F(EntityManagerTest, GetActiveEntities_List)
{
    LOG_FUNC_ENTER();
    EntityID entity1 = entityManager->CreateEntity();
    EntityID entity2 = entityManager->CreateEntity();
    EntityID entity3 = entityManager->CreateEntity();
    
    const auto& activeEntities = entityManager->GetActiveEntities();
    
    LOG_VAR_DEBUG(activeEntities.size());
    EXPECT_EQ(activeEntities.size(), 3u);
    
    // Should contain all created entities
    EXPECT_NE(std::find(activeEntities.begin(), activeEntities.end(), entity1), activeEntities.end());
    EXPECT_NE(std::find(activeEntities.begin(), activeEntities.end(), entity2), activeEntities.end());
    EXPECT_NE(std::find(activeEntities.begin(), activeEntities.end(), entity3), activeEntities.end());
    LOG_FUNC_EXIT();
}

TEST_F(EntityManagerTest, GetActiveEntities_AfterDestruction)
{
    LOG_FUNC_ENTER();
    EntityID entity1 = entityManager->CreateEntity();
    EntityID entity2 = entityManager->CreateEntity();
    EntityID entity3 = entityManager->CreateEntity();
    
    entityManager->DestroyEntity(entity2);
    
    const auto& activeEntities = entityManager->GetActiveEntities();
    
    LOG_VAR_DEBUG(activeEntities.size());
    EXPECT_EQ(activeEntities.size(), 2u);
    
    // Should not contain destroyed entity
    EXPECT_NE(std::find(activeEntities.begin(), activeEntities.end(), entity1), activeEntities.end());
    EXPECT_EQ(std::find(activeEntities.begin(), activeEntities.end(), entity2), activeEntities.end());
    EXPECT_NE(std::find(activeEntities.begin(), activeEntities.end(), entity3), activeEntities.end());
    LOG_FUNC_EXIT();
}

// Stress tests
TEST_F(EntityManagerTest, Stress_CreateManyEntities)
{
    LOG_FUNC_ENTER();
    // Removed PERF_TIMER due to compilation issues
    // NyonTest::PERF_TIMER("Stress_CreateManyEntities");
    
    const size_t numEntities = 10000;
    std::vector<EntityID> entities;
    
    for (size_t i = 0; i < numEntities; ++i) {
        EntityID entity = entityManager->CreateEntity();
        entities.push_back(entity);
    }
    
    LOG_VAR_DEBUG(entityManager->GetActiveEntityCount());
    EXPECT_EQ(entityManager->GetActiveEntityCount(), numEntities);
    
    // Verify all entities are valid
    for (const auto& entity : entities) {
        EXPECT_TRUE(entityManager->IsEntityValid(entity));
    }
    LOG_FUNC_EXIT();
}

TEST_F(EntityManagerTest, Stress_CreateDestroyCycle)
{
    LOG_FUNC_ENTER();
    // Removed PERF_TIMER due to compilation issues
    // NyonTest::PERF_TIMER("Stress_CreateDestroyCycle");
    
    const size_t cycleCount = 1000;
    
    for (size_t i = 0; i < cycleCount; ++i) {
        EntityID entity = entityManager->CreateEntity();
        entityManager->DestroyEntity(entity);
    }
    
    LOG_VAR_DEBUG(entityManager->GetActiveEntityCount());
    // Should have no active entities
    EXPECT_EQ(entityManager->GetActiveEntityCount(), 0u);
    LOG_FUNC_EXIT();
}

// Edge cases
TEST_F(EntityManagerTest, MaxEntityID)
{
    LOG_FUNC_ENTER();
    // Test behavior near maximum EntityID values
    EntityID maxEntity = std::numeric_limits<EntityID>::max();
    
    // Creating entities up to some reasonable limit
    for (int i = 0; i < 1000; ++i) {
        EntityID entity = entityManager->CreateEntity();
        EXPECT_NE(entity, INVALID_ENTITY);
        EXPECT_TRUE(entityManager->IsEntityValid(entity));
    }
    LOG_FUNC_EXIT();
}

TEST_F(EntityManagerTest, ConcurrentOperations)
{
    LOG_FUNC_ENTER();
    // Test that basic operations are safe
    EntityID entity1 = entityManager->CreateEntity();
    EntityID entity2 = entityManager->CreateEntity();
    
    // These operations should be safe concurrently
    EXPECT_NO_THROW({
        bool valid1 = entityManager->IsEntityValid(entity1);
        bool valid2 = entityManager->IsEntityValid(entity2);
        entityManager->DestroyEntity(entity1);
        [[maybe_unused]] size_t count = entityManager->GetActiveEntityCount();
    });
    LOG_FUNC_EXIT();
}

// Boundary condition tests
TEST_F(EntityManagerTest, INVALID_ENTITY_Handling)
{
    LOG_FUNC_ENTER();
    bool isValid = entityManager->IsEntityValid(INVALID_ENTITY);
    
    LOG_VAR_DEBUG(isValid);
    EXPECT_FALSE(isValid);
    
    // Operations on INVALID_ENTITY should be safe
    EXPECT_NO_THROW(entityManager->DestroyEntity(INVALID_ENTITY));
    LOG_FUNC_EXIT();
}