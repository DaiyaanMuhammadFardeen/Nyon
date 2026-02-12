#include <gtest/gtest.h>
#include "nyon/ecs/components/BehaviorComponent.h"
#include "TestHelpers.h"
#include <functional>
#include <memory>

using namespace Nyon::ECS;

/**
 * @brief Unit tests for BehaviorComponent functionality.
 * 
 * Tests callback execution, lambda capture, state management, edge cases,
 * and real-world behavior scenarios for entity behavior systems.
 */
class BehaviorComponentTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        behavior = std::make_unique<BehaviorComponent>();
        testEntity = 12345u;
        LOG_FUNC_EXIT();
    }

    void TearDown() override
    {
        LOG_FUNC_ENTER();
        behavior.reset();
        LOG_FUNC_EXIT();
    }

    std::unique_ptr<BehaviorComponent> behavior;
    EntityID testEntity;
};

// Basic functionality tests
TEST_F(BehaviorComponentTest, Constructor_Default)
{
    LOG_FUNC_ENTER();
    BehaviorComponent defaultBehavior;
    
    // Should be constructible and usable
    EXPECT_NO_THROW(defaultBehavior.Update(testEntity, 1.0f / 60.0f));
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, Update_WithoutFunction)
{
    LOG_FUNC_ENTER();
    // Should not crash when Update is called without setting a function
    EXPECT_NO_THROW(behavior->Update(testEntity, 1.0f / 60.0f));
    LOG_FUNC_EXIT();
}

// Lambda capture and execution tests
TEST_F(BehaviorComponentTest, SetUpdateFunction_Lambda)
{
    LOG_FUNC_ENTER();
    bool updateCalled = false;
    float capturedDelta = 0.0f;
    EntityID capturedEntity = INVALID_ENTITY;
    
    auto updateFunc = [&](EntityID entity, float deltaTime) {
        updateCalled = true;
        capturedEntity = entity;
        capturedDelta = deltaTime;
        LOG_VAR_DEBUG(entity);
        LOG_VAR_DEBUG(deltaTime);
    };
    
    behavior->SetUpdateFunction(updateFunc);
    
    float testDelta = 1.0f / 60.0f;
    behavior->Update(testEntity, testDelta);
    
    LOG_VAR_DEBUG(updateCalled);
    LOG_VAR_DEBUG(capturedEntity);
    LOG_VAR_DEBUG(capturedDelta);
    
    EXPECT_TRUE(updateCalled);
    EXPECT_EQ(capturedEntity, testEntity);
    EXPECT_FLOAT_NEAR(capturedDelta, testDelta, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, SetUpdateFunction_MultipleCalls)
{
    LOG_FUNC_ENTER();
    int callCount = 0;
    
    auto updateFunc = [&](EntityID entity, float deltaTime) {
        callCount++;
        LOG_VAR_DEBUG(callCount);
    };
    
    behavior->SetUpdateFunction(updateFunc);
    
    const int callTimes = 5;
    float deltaTime = 1.0f / 60.0f;
    
    for (int i = 0; i < callTimes; ++i) {
        behavior->Update(testEntity, deltaTime);
    }
    
    LOG_VAR_DEBUG(callCount);
    EXPECT_EQ(callCount, callTimes);
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, SetUpdateFunction_ChangeFunction)
{
    LOG_FUNC_ENTER();
    int firstCallCount = 0;
    int secondCallCount = 0;
    
    // First function
    auto firstFunc = [&](EntityID entity, float deltaTime) {
        firstCallCount++;
    };
    
    // Second function
    auto secondFunc = [&](EntityID entity, float deltaTime) {
        secondCallCount++;
    };
    
    // Set and call first function
    behavior->SetUpdateFunction(firstFunc);
    behavior->Update(testEntity, 1.0f / 60.0f);
    behavior->Update(testEntity, 1.0f / 60.0f);
    
    // Change to second function
    behavior->SetUpdateFunction(secondFunc);
    behavior->Update(testEntity, 1.0f / 60.0f);
    behavior->Update(testEntity, 1.0f / 60.0f);
    behavior->Update(testEntity, 1.0f / 60.0f);
    
    LOG_VAR_DEBUG(firstCallCount);
    LOG_VAR_DEBUG(secondCallCount);
    
    EXPECT_EQ(firstCallCount, 2);
    EXPECT_EQ(secondCallCount, 3);
    LOG_FUNC_EXIT();
}

// State capture tests
TEST_F(BehaviorComponentTest, StateCapture_LocalVariables)
{
    LOG_FUNC_ENTER();
    int externalCounter = 0;
    float externalValue = 3.14f;
    bool externalFlag = true;
    
    auto updateFunc = [&, externalValue](EntityID entity, float deltaTime) mutable {
        externalCounter++;
        LOG_VAR_DEBUG(externalCounter);
        LOG_VAR_DEBUG(externalValue);
        LOG_VAR_DEBUG(externalFlag);
        LOG_VAR_DEBUG(entity);
        LOG_VAR_DEBUG(deltaTime);
        
        // Modify captured values
        externalValue += deltaTime;
        externalFlag = !externalFlag;
    };
    
    behavior->SetUpdateFunction(updateFunc);
    
    behavior->Update(testEntity, 0.1f);
    behavior->Update(testEntity, 0.2f);
    
    LOG_VAR_DEBUG(externalCounter);
    LOG_VAR_DEBUG(externalValue);
    LOG_VAR_DEBUG(externalFlag);
    
    EXPECT_EQ(externalCounter, 2);
    EXPECT_GT(externalValue, 3.14f);
    EXPECT_FALSE(externalFlag);
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, StateCapture_ClassMembers)
{
    LOG_FUNC_ENTER();
    class TestState
    {
    public:
        int updateCount = 0;
        float accumulatedTime = 0.0f;
        EntityID lastEntity = INVALID_ENTITY;
        
        void Update(EntityID entity, float deltaTime)
        {
            updateCount++;
            accumulatedTime += deltaTime;
            lastEntity = entity;
        }
    };
    
    auto state = std::make_shared<TestState>();
    
    auto updateFunc = [state](EntityID entity, float deltaTime) {
        state->Update(entity, deltaTime);
        LOG_VAR_DEBUG(state->updateCount);
        LOG_VAR_DEBUG(state->accumulatedTime);
        LOG_VAR_DEBUG(state->lastEntity);
    };
    
    behavior->SetUpdateFunction(updateFunc);
    
    behavior->Update(testEntity, 0.1f);
    behavior->Update(testEntity + 1, 0.15f);
    behavior->Update(testEntity + 2, 0.2f);
    
    LOG_VAR_DEBUG(state->updateCount);
    LOG_VAR_DEBUG(state->accumulatedTime);
    LOG_VAR_DEBUG(state->lastEntity);
    
    EXPECT_EQ(state->updateCount, 3);
    EXPECT_FLOAT_NEAR(state->accumulatedTime, 0.45f, 1e-6f);
    EXPECT_EQ(state->lastEntity, testEntity + 2);
    LOG_FUNC_EXIT();
}

// Parameter validation tests
TEST_F(BehaviorComponentTest, Update_ZeroDeltaTime)
{
    LOG_FUNC_ENTER();
    bool updateCalled = false;
    float capturedDelta = -1.0f;
    
    auto updateFunc = [&](EntityID entity, float deltaTime) {
        updateCalled = true;
        capturedDelta = deltaTime;
    };
    
    behavior->SetUpdateFunction(updateFunc);
    behavior->Update(testEntity, 0.0f);
    
    LOG_VAR_DEBUG(updateCalled);
    LOG_VAR_DEBUG(capturedDelta);
    
    EXPECT_TRUE(updateCalled);
    EXPECT_FLOAT_NEAR(capturedDelta, 0.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, Update_NegativeDeltaTime)
{
    LOG_FUNC_ENTER();
    bool updateCalled = false;
    float capturedDelta = 0.0f;
    
    auto updateFunc = [&](EntityID entity, float deltaTime) {
        updateCalled = true;
        capturedDelta = deltaTime;
    };
    
    behavior->SetUpdateFunction(updateFunc);
    behavior->Update(testEntity, -0.1f);
    
    LOG_VAR_DEBUG(updateCalled);
    LOG_VAR_DEBUG(capturedDelta);
    
    EXPECT_TRUE(updateCalled);
    EXPECT_FLOAT_NEAR(capturedDelta, -0.1f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, Update_InvalidEntity)
{
    LOG_FUNC_ENTER();
    bool updateCalled = false;
    EntityID capturedEntity = testEntity;
    
    auto updateFunc = [&](EntityID entity, float deltaTime) {
        updateCalled = true;
        capturedEntity = entity;
    };
    
    behavior->SetUpdateFunction(updateFunc);
    behavior->Update(INVALID_ENTITY, 1.0f / 60.0f);
    
    LOG_VAR_DEBUG(updateCalled);
    LOG_VAR_DEBUG(capturedEntity);
    
    EXPECT_TRUE(updateCalled);
    EXPECT_EQ(capturedEntity, INVALID_ENTITY);
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, Update_DifferentEntities)
{
    LOG_FUNC_ENTER();
    std::vector<EntityID> capturedEntities;
    
    auto updateFunc = [&](EntityID entity, float deltaTime) {
        capturedEntities.push_back(entity);
        LOG_VAR_DEBUG(entity);
    };
    
    behavior->SetUpdateFunction(updateFunc);
    
    std::vector<EntityID> testEntities = {100u, 200u, 300u, 400u};
    
    for (EntityID entity : testEntities) {
        behavior->Update(entity, 1.0f / 60.0f);
    }
    
    LOG_VAR_DEBUG(capturedEntities.size());
    
    EXPECT_EQ(capturedEntities.size(), testEntities.size());
    for (size_t i = 0; i < testEntities.size(); ++i) {
        EXPECT_EQ(capturedEntities[i], testEntities[i]);
    }
    LOG_FUNC_EXIT();
}

// Complex lambda scenarios
TEST_F(BehaviorComponentTest, ComplexLambda_NestedCapture)
{
    LOG_FUNC_ENTER();
    class OuterState
    {
    public:
        int outerCounter = 0;
        class InnerState
        {
        public:
            int innerCounter = 0;
            float innerValue = 1.0f;
        };
        InnerState innerState;
    };
    
    auto outerState = std::make_shared<OuterState>();
    
    auto updateFunc = [outerState](EntityID entity, float deltaTime) {
        outerState->outerCounter++;
        outerState->innerState.innerCounter++;
        outerState->innerState.innerValue *= (1.0f + deltaTime);
        
        LOG_VAR_DEBUG(outerState->outerCounter);
        LOG_VAR_DEBUG(outerState->innerState.innerCounter);
        LOG_VAR_DEBUG(outerState->innerState.innerValue);
    };
    
    behavior->SetUpdateFunction(updateFunc);
    
    behavior->Update(testEntity, 0.1f);
    behavior->Update(testEntity, 0.2f);
    behavior->Update(testEntity, 0.3f);
    
    LOG_VAR_DEBUG(outerState->outerCounter);
    LOG_VAR_DEBUG(outerState->innerState.innerCounter);
    LOG_VAR_DEBUG(outerState->innerState.innerValue);
    
    EXPECT_EQ(outerState->outerCounter, 3);
    EXPECT_EQ(outerState->innerState.innerCounter, 3);
    EXPECT_GT(outerState->innerState.innerValue, 1.0f);
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, ComplexLambda_StdFunction)
{
    LOG_FUNC_ENTER();
    int callCount = 0;
    
    std::function<void(EntityID, float)> updateFunc = [&](EntityID entity, float deltaTime) {
        callCount++;
        LOG_VAR_DEBUG(callCount);
        LOG_VAR_DEBUG(entity);
        LOG_VAR_DEBUG(deltaTime);
    };
    
    behavior->SetUpdateFunction(updateFunc);
    
    behavior->Update(testEntity, 1.0f / 60.0f);
    behavior->Update(testEntity, 1.0f / 30.0f);
    
    LOG_VAR_DEBUG(callCount);
    EXPECT_EQ(callCount, 2);
    LOG_FUNC_EXIT();
}

// Performance and stress tests
TEST_F(BehaviorComponentTest, Performance_RapidUpdates)
{
    LOG_FUNC_ENTER();
    NyonTest::PERF_TIMER("BehaviorComponent_RapidUpdates");
    
    int callCount = 0;
    auto updateFunc = [&](EntityID entity, float deltaTime) {
        callCount++;
    };
    
    behavior->SetUpdateFunction(updateFunc);
    
    const int updateCount = 10000;
    float deltaTime = 1.0f / 60.0f;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < updateCount; ++i) {
        behavior->Update(testEntity, deltaTime);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    LOG_VAR_DEBUG(duration.count());
    LOG_VAR_DEBUG(callCount);
    
    EXPECT_EQ(callCount, updateCount);
    EXPECT_LT(duration.count(), 1000); // Should complete in less than 1 second
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, Performance_ComplexState)
{
    LOG_FUNC_ENTER();
    NyonTest::PERF_TIMER("BehaviorComponent_ComplexState");
    
    struct ComplexState
    {
        std::vector<float> values;
        std::map<int, std::string> mappings;
        int counter = 0;
        float accumulator = 0.0f;
    };
    
    auto state = std::make_shared<ComplexState>();
    
    // Pre-populate with some data
    for (int i = 0; i < 100; ++i) {
        state->values.push_back(static_cast<float>(i) * 0.1f);
        state->mappings[i] = "value_" + std::to_string(i);
    }
    
    auto updateFunc = [state](EntityID entity, float deltaTime) {
        state->counter++;
        state->accumulator += deltaTime;
        
        // Do some work with the complex state
        for (auto& value : state->values) {
            value += deltaTime * 0.01f;
        }
        
        // Access map
        [[maybe_unused]] auto it = state->mappings.find(state->counter % 100);
    };
    
    behavior->SetUpdateFunction(updateFunc);
    
    const int updateCount = 1000;
    for (int i = 0; i < updateCount; ++i) {
        behavior->Update(testEntity, 1.0f / 60.0f);
    }
    
    LOG_VAR_DEBUG(state->counter);
    LOG_VAR_DEBUG(state->accumulator);
    
    EXPECT_EQ(state->counter, updateCount);
    EXPECT_GT(state->accumulator, 0.0f);
    LOG_FUNC_EXIT();
}

// Edge case tests
TEST_F(BehaviorComponentTest, EdgeCase_RecursiveCall)
{
    LOG_FUNC_ENTER();
    int callDepth = 0;
    const int maxDepth = 5;
    
    std::function<void(EntityID, float)> recursiveFunc;
    recursiveFunc = [&](EntityID entity, float deltaTime) {
        callDepth++;
        LOG_VAR_DEBUG(callDepth);
        
        if (callDepth < maxDepth) {
            recursiveFunc(entity, deltaTime);
        }
    };
    
    behavior->SetUpdateFunction(recursiveFunc);
    
    behavior->Update(testEntity, 1.0f / 60.0f);
    
    LOG_VAR_DEBUG(callDepth);
    EXPECT_EQ(callDepth, maxDepth);
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, EdgeCase_ExceptionHandling)
{
    LOG_FUNC_ENTER();
    bool exceptionCaught = false;
    bool updateCompleted = false;
    
    auto updateFunc = [&](EntityID entity, float deltaTime) {
        try {
            // Simulate some operation that might throw
            if (deltaTime < 0) {
                throw std::runtime_error("Negative delta time");
            }
            updateCompleted = true;
        } catch (...) {
            exceptionCaught = true;
            throw; // Re-throw for testing
        }
    };
    
    behavior->SetUpdateFunction(updateFunc);
    
    // Test normal case
    EXPECT_NO_THROW(behavior->Update(testEntity, 1.0f / 60.0f));
    EXPECT_TRUE(updateCompleted);
    EXPECT_FALSE(exceptionCaught);
    
    // Test exceptional case
    updateCompleted = false;
    EXPECT_THROW(behavior->Update(testEntity, -1.0f), std::runtime_error);
    EXPECT_FALSE(updateCompleted);
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, EdgeCase_MoveSemantics)
{
    LOG_FUNC_ENTER();
    auto behavior1 = std::make_unique<BehaviorComponent>();
    int callCount = 0;
    
    auto updateFunc = [&](EntityID entity, float deltaTime) {
        callCount++;
    };
    
    behavior1->SetUpdateFunction(updateFunc);
    
    // Move the behavior
    auto behavior2 = std::move(behavior1);
    
    // Use the moved behavior
    behavior2->Update(testEntity, 1.0f / 60.0f);
    behavior2->Update(testEntity, 1.0f / 60.0f);
    
    LOG_VAR_DEBUG(callCount);
    EXPECT_EQ(callCount, 2);
    LOG_FUNC_EXIT();
}

// Real-world gaming scenarios
TEST_F(BehaviorComponentTest, GamingScenario_PlayerMovement)
{
    LOG_FUNC_ENTER();
    struct PlayerState
    {
        Nyon::Math::Vector2 position = {0.0f, 0.0f};
        Nyon::Math::Vector2 velocity = {0.0f, 0.0f};
        float speed = 300.0f;
        bool isJumping = false;
    };
    
    auto playerState = std::make_shared<PlayerState>();
    
    auto movementFunc = [playerState](EntityID entity, float deltaTime) {
        // Simulate input-based movement
        float targetSpeedX = 0.0f;
        if (/* input left */ false) targetSpeedX = -playerState->speed;
        if (/* input right */ true) targetSpeedX = playerState->speed; // Simulate right input
        
        // Apply acceleration
        float acceleration = 2000.0f;
        playerState->velocity.x += (targetSpeedX - playerState->velocity.x) * acceleration * deltaTime;
        
        // Apply velocity to position
        playerState->position.x += playerState->velocity.x * deltaTime;
        playerState->position.y += playerState->velocity.y * deltaTime;
        
        LOG_VAR_DEBUG(playerState->position.x);
        LOG_VAR_DEBUG(playerState->position.y);
        LOG_VAR_DEBUG(playerState->velocity.x);
    };
    
    behavior->SetUpdateFunction(movementFunc);
    
    // Simulate several frames of movement
    for (int i = 0; i < 60; ++i) { // 1 second at 60 FPS
        behavior->Update(testEntity, 1.0f / 60.0f);
    }
    
    LOG_VAR_DEBUG(playerState->position.x);
    LOG_VAR_DEBUG(playerState->position.y);
    
    // Player should have moved to the right
    EXPECT_GT(playerState->position.x, 0.0f);
    EXPECT_FLOAT_NEAR(playerState->position.y, 0.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, GamingScenario_EnemyAI)
{
    LOG_FUNC_ENTER();
    struct EnemyState
    {
        Nyon::Math::Vector2 position = {100.0f, 100.0f};
        int health = 100;
        float detectionRadius = 50.0f;
        enum State { PATROL, CHASE, ATTACK } aiState = PATROL;
        int stateTimer = 0;
    };
    
    auto enemyState = std::make_shared<EnemyState>();
    
    auto aiFunc = [enemyState](EntityID entity, float deltaTime) {
        enemyState->stateTimer++;
        
        switch (enemyState->aiState) {
            case EnemyState::PATROL:
                // Simple patrol behavior
                enemyState->position.x += std::sin(enemyState->stateTimer * 0.1f) * 10.0f * deltaTime;
                if (enemyState->stateTimer > 100) {
                    enemyState->aiState = EnemyState::CHASE;
                    enemyState->stateTimer = 0;
                }
                break;
                
            case EnemyState::CHASE:
                // Chase behavior would go here
                if (enemyState->stateTimer > 50) {
                    enemyState->aiState = EnemyState::ATTACK;
                    enemyState->stateTimer = 0;
                }
                break;
                
            case EnemyState::ATTACK:
                // Attack behavior would go here
                if (enemyState->stateTimer > 25) {
                    enemyState->aiState = EnemyState::PATROL;
                    enemyState->stateTimer = 0;
                }
                break;
        }
        
        LOG_VAR_DEBUG(static_cast<int>(enemyState->aiState));
        LOG_VAR_DEBUG(enemyState->position.x);
        LOG_VAR_DEBUG(enemyState->position.y);
        LOG_VAR_DEBUG(enemyState->stateTimer);
    };
    
    behavior->SetUpdateFunction(aiFunc);
    
    // Run AI for several cycles
    for (int i = 0; i < 200; ++i) {
        behavior->Update(testEntity, 1.0f / 60.0f);
    }
    
    // Should have gone through all states
    EXPECT_EQ(enemyState->aiState, EnemyState::PATROL);
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, GamingScenario_Projectile)
{
    LOG_FUNC_ENTER();
    struct ProjectileState
    {
        Nyon::Math::Vector2 position = {0.0f, 0.0f};
        Nyon::Math::Vector2 velocity = {500.0f, 0.0f}; // Moving right
        float lifetime = 5.0f;
        bool isActive = true;
    };
    
    auto projectileState = std::make_shared<ProjectileState>();
    
    auto projectileFunc = [projectileState](EntityID entity, float deltaTime) {
        if (!projectileState->isActive) return;
        
        // Update position
        projectileState->position.x += projectileState->velocity.x * deltaTime;
        projectileState->position.y += projectileState->velocity.y * deltaTime;
        
        // Update lifetime
        projectileState->lifetime -= deltaTime;
        
        // Deactivate when lifetime expires
        if (projectileState->lifetime <= 0.0f) {
            projectileState->isActive = false;
        }
        
        LOG_VAR_DEBUG(projectileState->position.x);
        LOG_VAR_DEBUG(projectileState->position.y);
        LOG_VAR_DEBUG(projectileState->lifetime);
        LOG_VAR_DEBUG(projectileState->isActive);
    };
    
    behavior->SetUpdateFunction(projectileFunc);
    
    // Update until projectile expires
    int frameCount = 0;
    while (projectileState->isActive && frameCount < 1000) {
        behavior->Update(testEntity, 1.0f / 60.0f);
        frameCount++;
    }
    
    LOG_VAR_DEBUG(frameCount);
    LOG_VAR_DEBUG(projectileState->position.x);
    
    // Projectile should have traveled and then expired
    EXPECT_FALSE(projectileState->isActive);
    EXPECT_GT(projectileState->position.x, 0.0f);
    EXPECT_LT(projectileState->lifetime, 0.0f);
    LOG_FUNC_EXIT();
}

// Memory management tests
TEST_F(BehaviorComponentTest, MemoryManagement_LambdaLifetime)
{
    LOG_FUNC_ENTER();
    auto externalData = std::make_shared<std::vector<int>>();
    externalData->resize(1000, 42); // Large data structure
    
    auto updateFunc = [externalData](EntityID entity, float deltaTime) {
        // Use the captured data
        externalData->at(0) = static_cast<int>(deltaTime * 1000);
        LOG_VAR_DEBUG(externalData->at(0));
    };
    
    behavior->SetUpdateFunction(updateFunc);
    
    // Multiple updates should work with captured data
    behavior->Update(testEntity, 0.1f);
    behavior->Update(testEntity, 0.2f);
    
    // External data should still be valid
    EXPECT_EQ(externalData->at(0), 200); // From last update
    LOG_FUNC_EXIT();
}

TEST_F(BehaviorComponentTest, MemoryManagement_FunctionReplacement)
{
    LOG_FUNC_ENTER();
    auto data1 = std::make_shared<int>(1);
    auto data2 = std::make_shared<int>(2);
    
    // First function
    auto func1 = [data1](EntityID entity, float deltaTime) {
        (*data1)++;
        LOG_VAR_DEBUG(*data1);
    };
    
    // Second function
    auto func2 = [data2](EntityID entity, float deltaTime) {
        (*data2) *= 2;
        LOG_VAR_DEBUG(*data2);
    };
    
    behavior->SetUpdateFunction(func1);
    behavior->Update(testEntity, 1.0f / 60.0f);
    behavior->Update(testEntity, 1.0f / 60.0f);
    
    // Replace function
    behavior->SetUpdateFunction(func2);
    behavior->Update(testEntity, 1.0f / 60.0f);
    behavior->Update(testEntity, 1.0f / 60.0f);
    behavior->Update(testEntity, 1.0f / 60.0f);
    
    LOG_VAR_DEBUG(*data1);
    LOG_VAR_DEBUG(*data2);
    
    // First data should be incremented twice, second should be doubled thrice
    EXPECT_EQ(*data1, 3);
    EXPECT_EQ(*data2, 16);
    LOG_FUNC_EXIT();
}