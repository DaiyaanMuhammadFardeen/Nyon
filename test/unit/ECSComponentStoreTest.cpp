#include <gtest/gtest.h>
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "TestHelpers.h"

using namespace Nyon::ECS;

/**
 * @brief Unit tests for ComponentStore functionality.
 * 
 * Tests component addition, removal, retrieval, entity-component relationships,
 * and type safety for the ECS component storage system.
 */
class ComponentStoreTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        entityManager = std::make_unique<EntityManager>();
        componentStore = std::make_unique<ComponentStore>(*entityManager);
        LOG_FUNC_EXIT();
    }

    std::unique_ptr<EntityManager> entityManager;
    std::unique_ptr<ComponentStore> componentStore;
};

// Component addition tests
TEST_F(ComponentStoreTest, AddComponent_SingleComponent)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
    
    componentStore->AddComponent(entity, std::move(transform));
    
    EXPECT_TRUE(componentStore->HasComponent<TransformComponent>(entity));
    LOG_FUNC_EXIT();
}

TEST_F(ComponentStoreTest, AddComponent_MultipleComponentsSameEntity)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
    PhysicsBodyComponent physics(1.0f, false);
    
    componentStore->AddComponent(entity, std::move(transform));
    componentStore->AddComponent(entity, std::move(physics));
    
    EXPECT_TRUE(componentStore->HasComponent<TransformComponent>(entity));
    EXPECT_TRUE(componentStore->HasComponent<PhysicsBodyComponent>(entity));
    LOG_FUNC_EXIT();
}

TEST_F(ComponentStoreTest, AddComponent_SameTypeMultipleEntities)
{
    LOG_FUNC_ENTER();
    EntityID entity1 = entityManager->CreateEntity();
    EntityID entity2 = entityManager->CreateEntity();
    
    TransformComponent transform1({100.0f, 200.0f});
    TransformComponent transform2({300.0f, 400.0f});
    
    componentStore->AddComponent(entity1, std::move(transform1));
    componentStore->AddComponent(entity2, std::move(transform2));
    
    EXPECT_TRUE(componentStore->HasComponent<TransformComponent>(entity1));
    EXPECT_TRUE(componentStore->HasComponent<TransformComponent>(entity2));
    LOG_FUNC_EXIT();
}

// Component retrieval tests
TEST_F(ComponentStoreTest, GetComponent_ValidComponent)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    TransformComponent originalTransform({150.0f, 250.0f});
    Nyon::Math::Vector2 expectedPosition = originalTransform.position;
    
    componentStore->AddComponent(entity, std::move(originalTransform));
    
    const TransformComponent& retrieved = componentStore->GetComponent<TransformComponent>(entity);
    
    LOG_VAR_DEBUG(retrieved.position.x);
    LOG_VAR_DEBUG(retrieved.position.y);
    LOG_VAR_DEBUG(expectedPosition.x);
    LOG_VAR_DEBUG(expectedPosition.y);
    
    EXPECT_VECTOR2_NEAR(retrieved.position, expectedPosition, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ComponentStoreTest, GetComponent_ModifyComponent)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    TransformComponent transform(Nyon::Math::Vector2(0.0f, 0.0f));
    
    componentStore->AddComponent(entity, std::move(transform));
    
    // Modify through reference
    TransformComponent& retrieved = componentStore->GetComponent<TransformComponent>(entity);
    retrieved.position.x = 100.0f;
    retrieved.position.y = 200.0f;
    
    // Verify modification persists
    const TransformComponent& verified = componentStore->GetComponent<TransformComponent>(entity);
    
    LOG_VAR_DEBUG(verified.position.x);
    LOG_VAR_DEBUG(verified.position.y);
    EXPECT_FLOAT_NEAR(verified.position.x, 100.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(verified.position.y, 200.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

// Component removal tests
TEST_F(ComponentStoreTest, RemoveComponent_SingleComponent)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
    
    componentStore->AddComponent(entity, std::move(transform));
    EXPECT_TRUE(componentStore->HasComponent<TransformComponent>(entity));
    
    componentStore->RemoveComponent<TransformComponent>(entity);
    
    EXPECT_FALSE(componentStore->HasComponent<TransformComponent>(entity));
    LOG_FUNC_EXIT();
}

TEST_F(ComponentStoreTest, RemoveComponent_NonExistent)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    // Should not crash when removing non-existent component
    EXPECT_NO_THROW(componentStore->RemoveComponent<TransformComponent>(entity));
    EXPECT_FALSE(componentStore->HasComponent<TransformComponent>(entity));
    LOG_FUNC_EXIT();
}

TEST_F(ComponentStoreTest, RemoveComponent_PartialRemoval)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
    PhysicsBodyComponent physics(1.0f, false);
    
    componentStore->AddComponent(entity, std::move(transform));
    componentStore->AddComponent(entity, std::move(physics));
    
    componentStore->RemoveComponent<TransformComponent>(entity);
    
    EXPECT_FALSE(componentStore->HasComponent<TransformComponent>(entity));
    EXPECT_TRUE(componentStore->HasComponent<PhysicsBodyComponent>(entity));
    LOG_FUNC_EXIT();
}

// Component existence tests
TEST_F(ComponentStoreTest, HasComponent_ExistingComponent)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
    
    componentStore->AddComponent(entity, std::move(transform));
    
    bool hasComponent = componentStore->HasComponent<TransformComponent>(entity);
    LOG_VAR_DEBUG(hasComponent);
    EXPECT_TRUE(hasComponent);
    LOG_FUNC_EXIT();
}

TEST_F(ComponentStoreTest, HasComponent_NonExistentComponent)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    bool hasComponent = componentStore->HasComponent<TransformComponent>(entity);
    LOG_VAR_DEBUG(hasComponent);
    EXPECT_FALSE(hasComponent);
    LOG_FUNC_EXIT();
}

TEST_F(ComponentStoreTest, HasComponent_WrongEntityType)
{
    LOG_FUNC_ENTER();
    EntityID entity1 = entityManager->CreateEntity();
    EntityID entity2 = entityManager->CreateEntity();
    TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
    
    componentStore->AddComponent(entity1, std::move(transform));
    
    bool hasComponent = componentStore->HasComponent<TransformComponent>(entity2);
    LOG_VAR_DEBUG(hasComponent);
    EXPECT_FALSE(hasComponent);
    LOG_FUNC_EXIT();
}

// Entity-component queries
TEST_F(ComponentStoreTest, GetEntitiesWithComponent_Empty)
{
    LOG_FUNC_ENTER();
    const auto& entities = componentStore->GetEntitiesWithComponent<TransformComponent>();
    
    LOG_VAR_DEBUG(entities.size());
    EXPECT_TRUE(entities.empty());
    LOG_FUNC_EXIT();
}

TEST_F(ComponentStoreTest, GetEntitiesWithComponent_SingleEntity)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
    
    componentStore->AddComponent(entity, std::move(transform));
    
    const auto& entities = componentStore->GetEntitiesWithComponent<TransformComponent>();
    
    LOG_VAR_DEBUG(entities.size());
    EXPECT_EQ(entities.size(), 1u);
    EXPECT_EQ(entities[0], entity);
    LOG_FUNC_EXIT();
}

TEST_F(ComponentStoreTest, GetEntitiesWithComponent_MultipleEntities)
{
    LOG_FUNC_ENTER();
    EntityID entity1 = entityManager->CreateEntity();
    EntityID entity2 = entityManager->CreateEntity();
    EntityID entity3 = entityManager->CreateEntity();
    
    TransformComponent transform1({100.0f, 200.0f});
    TransformComponent transform2({300.0f, 400.0f});
    TransformComponent transform3({500.0f, 600.0f});
    
    componentStore->AddComponent(entity1, std::move(transform1));
    componentStore->AddComponent(entity2, std::move(transform2));
    componentStore->AddComponent(entity3, std::move(transform3));
    
    const auto& entities = componentStore->GetEntitiesWithComponent<TransformComponent>();
    
    LOG_VAR_DEBUG(entities.size());
    EXPECT_EQ(entities.size(), 3u);
    
    // Should contain all entities with the component
    EXPECT_NE(std::find(entities.begin(), entities.end(), entity1), entities.end());
    EXPECT_NE(std::find(entities.begin(), entities.end(), entity2), entities.end());
    EXPECT_NE(std::find(entities.begin(), entities.end(), entity3), entities.end());
    LOG_FUNC_EXIT();
}

TEST_F(ComponentStoreTest, GetEntitiesWithComponent_AfterRemoval)
{
    LOG_FUNC_ENTER();
    EntityID entity1 = entityManager->CreateEntity();
    EntityID entity2 = entityManager->CreateEntity();
    
    TransformComponent transform1({100.0f, 200.0f});
    TransformComponent transform2({300.0f, 400.0f});
    
    componentStore->AddComponent(entity1, std::move(transform1));
    componentStore->AddComponent(entity2, std::move(transform2));
    
    componentStore->RemoveComponent<TransformComponent>(entity1);
    
    const auto& entities = componentStore->GetEntitiesWithComponent<TransformComponent>();
    
    LOG_VAR_DEBUG(entities.size());
    EXPECT_EQ(entities.size(), 1u);
    EXPECT_EQ(entities[0], entity2);
    LOG_FUNC_EXIT();
}

// Type safety tests
TEST_F(ComponentStoreTest, DifferentComponentTypes_Isolation)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
    PhysicsBodyComponent physics(1.0f, false);
    
    componentStore->AddComponent(entity, std::move(transform));
    componentStore->AddComponent(entity, std::move(physics));
    
    // Each component type should be isolated
    EXPECT_TRUE(componentStore->HasComponent<TransformComponent>(entity));
    EXPECT_TRUE(componentStore->HasComponent<PhysicsBodyComponent>(entity));
    
    componentStore->RemoveComponent<TransformComponent>(entity);
    
    EXPECT_FALSE(componentStore->HasComponent<TransformComponent>(entity));
    EXPECT_TRUE(componentStore->HasComponent<PhysicsBodyComponent>(entity));
    LOG_FUNC_EXIT();
}

// Invalid entity tests
TEST_F(ComponentStoreTest, AddComponent_InvalidEntity)
{
    LOG_FUNC_ENTER();
    EntityID invalidEntity = 999u;
    TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
    
    // Should handle invalid entities gracefully
    EXPECT_DEATH(componentStore->AddComponent(invalidEntity, std::move(transform)), "");
    LOG_FUNC_EXIT();
}

TEST_F(ComponentStoreTest, GetComponent_InvalidEntity)
{
    LOG_FUNC_ENTER();
    EntityID invalidEntity = 999u;
    
    // Should handle invalid entities gracefully
    EXPECT_DEATH(componentStore->GetComponent<TransformComponent>(invalidEntity), "");
    LOG_FUNC_EXIT();
}

TEST_F(ComponentStoreTest, HasComponent_InvalidEntity)
{
    LOG_FUNC_ENTER();
    EntityID invalidEntity = 999u;
    
    bool hasComponent = componentStore->HasComponent<TransformComponent>(invalidEntity);
    LOG_VAR_DEBUG(hasComponent);
    EXPECT_FALSE(hasComponent);
    LOG_FUNC_EXIT();
}

// Performance and stress tests
TEST_F(ComponentStoreTest, Stress_AddManyComponents)
{
    LOG_FUNC_ENTER();
    // Removed PERF_TIMER due to compilation issues
    // NyonTest::PERF_TIMER("Stress_AddManyComponents");
    
    const size_t numEntities = 1000;
    std::vector<EntityID> entities;
    
    // Create entities and add components
    for (size_t i = 0; i < numEntities; ++i) {
        EntityID entity = entityManager->CreateEntity();
        entities.push_back(entity);
        TransformComponent transform(Nyon::Math::Vector2(static_cast<float>(i), static_cast<float>(i * 2)));
        componentStore->AddComponent(entity, std::move(transform));
    }
    
    // Verify all components exist
    for (const auto& entity : entities) {
        EXPECT_TRUE(componentStore->HasComponent<TransformComponent>(entity));
    }
    
    const auto& componentEntities = componentStore->GetEntitiesWithComponent<TransformComponent>();
    LOG_VAR_DEBUG(componentEntities.size());
    EXPECT_EQ(componentEntities.size(), numEntities);
    LOG_FUNC_EXIT();
}

TEST_F(ComponentStoreTest, MemoryManagement_MoveSemantics)
{
    LOG_FUNC_ENTER();
    EntityID entity = entityManager->CreateEntity();
    
    // Test that components are properly moved
    TransformComponent transform(Nyon::Math::Vector2(100.0f, 200.0f));
    Nyon::Math::Vector2 originalPosition = transform.position;
    
    componentStore->AddComponent(entity, std::move(transform));
    
    // Original should be in valid but unspecified state after move
    const TransformComponent& retrieved = componentStore->GetComponent<TransformComponent>(entity);
    
    LOG_VAR_DEBUG(retrieved.position.x);
    LOG_VAR_DEBUG(retrieved.position.y);
    EXPECT_VECTOR2_NEAR(retrieved.position, originalPosition, 1e-6f);
    LOG_FUNC_EXIT();
}