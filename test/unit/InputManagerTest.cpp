#include <gtest/gtest.h>
#include "nyon/utils/InputManager.h"
#include "TestHelpers.h"
#include <thread>
#include <chrono>

using namespace Nyon::Utils;

/**
 * @brief Unit tests for InputManager functionality.
 * 
 * Tests keyboard and mouse input handling, state tracking, edge cases,
 * null pointer safety, and real-world input scenarios.
 */
class InputManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        // Note: We can't create actual GLFW windows in unit tests
        // So we'll test the logic without window initialization
        LOG_FUNC_EXIT();
    }

    void TearDown() override
    {
        LOG_FUNC_ENTER();
        // Reset static state between tests
        LOG_FUNC_EXIT();
    }
};

// Initialization tests
TEST_F(InputManagerTest, Init_NullWindow)
{
    LOG_FUNC_ENTER();
    // Should handle null window gracefully
    EXPECT_NO_THROW(InputManager::Init(nullptr));
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, Init_ValidWindow)
{
    LOG_FUNC_ENTER();
    GLFWwindow* mockWindow = reinterpret_cast<GLFWwindow*>(0x12345678);
    
    InputManager::Init(mockWindow);
    
    // Should store the window pointer (we can't easily verify this without accessing private members)
    EXPECT_NO_THROW(InputManager::Update());
    LOG_FUNC_EXIT();
}

// Update tests
TEST_F(InputManagerTest, Update_WithoutInit)
{
    LOG_FUNC_ENTER();
    // Should handle update without initialization
    EXPECT_NO_THROW(InputManager::Update());
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, Update_NullWindow)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    // Should handle null window gracefully in update
    EXPECT_NO_THROW(InputManager::Update());
    LOG_FUNC_EXIT();
}

// Keyboard input tests
TEST_F(InputManagerTest, IsKeyDown_ValidKey)
{
    LOG_FUNC_ENTER();
    // Test with uninitialized window
    InputManager::Init(nullptr);
    
    bool result = InputManager::IsKeyDown(GLFW_KEY_A);
    
    LOG_VAR_DEBUG(result);
    // Should return false for any key when window is null
    EXPECT_FALSE(result);
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, IsKeyDown_InvalidKey)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    bool result = InputManager::IsKeyDown(-1); // Invalid key
    
    LOG_VAR_DEBUG(result);
    EXPECT_FALSE(result);
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, IsKeyDown_OutOfRangeKey)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    bool result = InputManager::IsKeyDown(GLFW_KEY_LAST + 100); // Way out of range
    
    LOG_VAR_DEBUG(result);
    EXPECT_FALSE(result);
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, IsKeyPressed_ValidKey)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    bool result = InputManager::IsKeyPressed(GLFW_KEY_SPACE);
    
    LOG_VAR_DEBUG(result);
    EXPECT_FALSE(result); // Should be false without actual input
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, IsKeyUp_ValidKey)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    bool result = InputManager::IsKeyUp(GLFW_KEY_ENTER);
    
    LOG_VAR_DEBUG(result);
    EXPECT_TRUE(result); // Should be true when key is not pressed
    LOG_FUNC_EXIT();
}

// Mouse input tests
TEST_F(InputManagerTest, IsMouseDown_ValidButton)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    bool result = InputManager::IsMouseDown(GLFW_MOUSE_BUTTON_LEFT);
    
    LOG_VAR_DEBUG(result);
    EXPECT_FALSE(result); // Should be false without actual input
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, IsMousePressed_ValidButton)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    bool result = InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT);
    
    LOG_VAR_DEBUG(result);
    EXPECT_FALSE(result);
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, IsMouseUp_ValidButton)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    bool result = InputManager::IsMouseUp(GLFW_MOUSE_BUTTON_MIDDLE);
    
    LOG_VAR_DEBUG(result);
    EXPECT_TRUE(result); // Should be true when button is not pressed
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, IsMouseDown_InvalidButton)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    bool result = InputManager::IsMouseDown(-1); // Invalid button
    
    LOG_VAR_DEBUG(result);
    EXPECT_FALSE(result);
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, IsMouseDown_OutOfRangeButton)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    bool result = InputManager::IsMouseDown(GLFW_MOUSE_BUTTON_LAST + 100); // Out of range
    
    LOG_VAR_DEBUG(result);
    EXPECT_FALSE(result);
    LOG_FUNC_EXIT();
}

// Mouse position tests
TEST_F(InputManagerTest, GetMousePosition_Uninitialized)
{
    LOG_FUNC_ENTER();
    double x, y;
    
    InputManager::GetMousePosition(x, y);
    
    LOG_VAR_DEBUG(x);
    LOG_VAR_DEBUG(y);
    // Should return zeros when uninitialized
    EXPECT_DOUBLE_EQ(x, 0.0);
    EXPECT_DOUBLE_EQ(y, 0.0);
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, GetMousePosition_NullWindow)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    double x, y;
    InputManager::GetMousePosition(x, y);
    
    LOG_VAR_DEBUG(x);
    LOG_VAR_DEBUG(y);
    // Should return zeros when window is null
    EXPECT_DOUBLE_EQ(x, 0.0);
    EXPECT_DOUBLE_EQ(y, 0.0);
    LOG_FUNC_EXIT();
}

// State transition tests
TEST_F(InputManagerTest, KeyStateTransitions)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    // Test the logic of key state transitions
    // Since we can't simulate actual key presses, we test the API behavior
    
    bool initiallyDown = InputManager::IsKeyDown(GLFW_KEY_W);
    bool initiallyPressed = InputManager::IsKeyPressed(GLFW_KEY_W);
    bool initiallyUp = InputManager::IsKeyUp(GLFW_KEY_W);
    
    LOG_VAR_DEBUG(initiallyDown);
    LOG_VAR_DEBUG(initiallyPressed);
    LOG_VAR_DEBUG(initiallyUp);
    
    // Without actual input, key should not be down or pressed
    EXPECT_FALSE(initiallyDown);
    EXPECT_FALSE(initiallyPressed);
    EXPECT_TRUE(initiallyUp);
    
    // Update shouldn't change this without actual input
    InputManager::Update();
    
    bool afterUpdateDown = InputManager::IsKeyDown(GLFW_KEY_W);
    bool afterUpdatePressed = InputManager::IsKeyPressed(GLFW_KEY_W);
    
    LOG_VAR_DEBUG(afterUpdateDown);
    LOG_VAR_DEBUG(afterUpdatePressed);
    
    EXPECT_FALSE(afterUpdateDown);
    EXPECT_FALSE(afterUpdatePressed);
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, MouseButtonStateTransitions)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    bool initiallyDown = InputManager::IsMouseDown(GLFW_MOUSE_BUTTON_LEFT);
    bool initiallyPressed = InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT);
    bool initiallyUp = InputManager::IsMouseUp(GLFW_MOUSE_BUTTON_LEFT);
    
    LOG_VAR_DEBUG(initiallyDown);
    LOG_VAR_DEBUG(initiallyPressed);
    LOG_VAR_DEBUG(initiallyUp);
    
    EXPECT_FALSE(initiallyDown);
    EXPECT_FALSE(initiallyPressed);
    EXPECT_TRUE(initiallyUp);
    
    InputManager::Update();
    
    bool afterUpdateDown = InputManager::IsMouseDown(GLFW_MOUSE_BUTTON_LEFT);
    bool afterUpdatePressed = InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT);
    
    LOG_VAR_DEBUG(afterUpdateDown);
    LOG_VAR_DEBUG(afterUpdatePressed);
    
    EXPECT_FALSE(afterUpdateDown);
    EXPECT_FALSE(afterUpdatePressed);
    LOG_FUNC_EXIT();
}

// Edge case tests
TEST_F(InputManagerTest, MultipleUpdates)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    // Multiple rapid updates should not cause issues
    for (int i = 0; i < 100; ++i) {
        InputManager::Update();
    }
    
    // Should not crash and should maintain consistent state
    bool keyState = InputManager::IsKeyDown(GLFW_KEY_A);
    bool mouseState = InputManager::IsMouseDown(GLFW_MOUSE_BUTTON_LEFT);
    
    LOG_VAR_DEBUG(keyState);
    LOG_VAR_DEBUG(mouseState);
    
    EXPECT_FALSE(keyState);
    EXPECT_FALSE(mouseState);
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, RapidKeyQueries)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    // Rapid querying of the same key should be consistent
    bool states[10];
    for (int i = 0; i < 10; ++i) {
        states[i] = InputManager::IsKeyDown(GLFW_KEY_SPACE);
    }
    
    // All queries should return the same result
    for (int i = 1; i < 10; ++i) {
        EXPECT_EQ(states[0], states[i]) << "Inconsistent state at index " << i;
    }
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, MixedInputTypes)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    // Mix keyboard and mouse queries
    bool keyDown = InputManager::IsKeyDown(GLFW_KEY_W);
    bool mouseDown = InputManager::IsMouseDown(GLFW_MOUSE_BUTTON_LEFT);
    bool keyPressed = InputManager::IsKeyPressed(GLFW_KEY_SPACE);
    bool mousePressed = InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT);
    
    LOG_VAR_DEBUG(keyDown);
    LOG_VAR_DEBUG(mouseDown);
    LOG_VAR_DEBUG(keyPressed);
    LOG_VAR_DEBUG(mousePressed);
    
    // All should return consistent false values without actual input
    EXPECT_FALSE(keyDown);
    EXPECT_FALSE(mouseDown);
    EXPECT_FALSE(keyPressed);
    EXPECT_FALSE(mousePressed);
    LOG_FUNC_EXIT();
}

// Thread safety tests (basic)
TEST_F(InputManagerTest, ThreadSafety_Basic)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    // Basic thread safety test - multiple threads calling input functions
    std::vector<std::thread> threads;
    std::atomic<int> successCount(0);
    
    auto testFunction = [&successCount]() {
        for (int i = 0; i < 100; ++i) {
            bool result = InputManager::IsKeyDown(GLFW_KEY_A);
            if (!result) { // Expected false
                successCount++;
            }
        }
    };
    
    // Launch multiple threads
    for (int i = 0; i < 4; ++i) {
        threads.emplace_back(testFunction);
    }
    
    // Wait for all threads
    for (auto& thread : threads) {
        thread.join();
    }
    
    LOG_VAR_DEBUG(successCount.load());
    // All operations should succeed (return false as expected)
    EXPECT_EQ(successCount.load(), 400);
    LOG_FUNC_EXIT();
}

// Null pointer safety tests
TEST_F(InputManagerTest, NullPointerSafety_AllFunctions)
{
    LOG_FUNC_ENTER();
    // Test all functions with null window to ensure no crashes
    InputManager::Init(nullptr);
    
    EXPECT_NO_THROW(({
        InputManager::Update();
        InputManager::IsKeyPressed(GLFW_KEY_A);
        InputManager::IsKeyDown(GLFW_KEY_B);
        InputManager::IsKeyUp(GLFW_KEY_C);
        InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT);
        InputManager::IsMouseDown(GLFW_MOUSE_BUTTON_RIGHT);
        InputManager::IsMouseUp(GLFW_MOUSE_BUTTON_MIDDLE);
        
        double x, y;
        InputManager::GetMousePosition(x, y);
    }));
    LOG_FUNC_EXIT();
}

// Boundary value tests
TEST_F(InputManagerTest, BoundaryKeys_ValidRange)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    // Test boundary keys in valid range
    bool result1 = InputManager::IsKeyDown(0); // First valid key
    bool result2 = InputManager::IsKeyDown(GLFW_KEY_LAST - 1); // Last valid key
    
    LOG_VAR_DEBUG(result1);
    LOG_VAR_DEBUG(result2);
    
    EXPECT_FALSE(result1);
    EXPECT_FALSE(result2);
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, BoundaryMouseButtons_ValidRange)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    // Test boundary mouse buttons in valid range
    bool result1 = InputManager::IsMouseDown(0); // First valid button
    bool result2 = InputManager::IsMouseDown(GLFW_MOUSE_BUTTON_LAST - 1); // Last valid button
    
    LOG_VAR_DEBUG(result1);
    LOG_VAR_DEBUG(result2);
    
    EXPECT_FALSE(result1);
    EXPECT_FALSE(result2);
    LOG_FUNC_EXIT();
}

// Performance tests
TEST_F(InputManagerTest, Performance_RapidUpdates)
{
    LOG_FUNC_ENTER();
    NyonTest::PERF_TIMER("InputManager_RapidUpdates");
    
    InputManager::Init(nullptr);
    
    const int updateCount = 10000;
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < updateCount; ++i) {
        InputManager::Update();
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    LOG_VAR_DEBUG(duration.count());
    LOG_VAR_DEBUG(updateCount);
    
    // Should complete quickly
    EXPECT_LT(duration.count(), 1000000); // Less than 1 second for 10k updates
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, Performance_RapidQueries)
{
    LOG_FUNC_ENTER();
    NyonTest::PERF_TIMER("InputManager_RapidQueries");
    
    InputManager::Init(nullptr);
    
    const int queryCount = 10000;
    int falseCount = 0;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < queryCount; ++i) {
        if (!InputManager::IsKeyDown(GLFW_KEY_A)) {
            falseCount++;
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    LOG_VAR_DEBUG(duration.count());
    LOG_VAR_DEBUG(falseCount);
    
    EXPECT_EQ(falseCount, queryCount); // All should return false
    EXPECT_LT(duration.count(), 1000000); // Less than 1 second
    LOG_FUNC_EXIT();
}

// Integration scenario tests
TEST_F(InputManagerTest, GamingScenario_JumpInput)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    // Simulate checking for jump input (spacebar)
    bool jumpPressed = InputManager::IsKeyPressed(GLFW_KEY_SPACE);
    bool jumpDown = InputManager::IsKeyDown(GLFW_KEY_SPACE);
    bool jumpUp = InputManager::IsKeyUp(GLFW_KEY_SPACE);
    
    LOG_VAR_DEBUG(jumpPressed);
    LOG_VAR_DEBUG(jumpDown);
    LOG_VAR_DEBUG(jumpUp);
    
    // Without actual input, should return expected default states
    EXPECT_FALSE(jumpPressed);
    EXPECT_FALSE(jumpDown);
    EXPECT_TRUE(jumpUp);
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, GamingScenario_MovementInput)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    // Simulate WASD movement input checking
    bool wDown = InputManager::IsKeyDown(GLFW_KEY_W);
    bool aDown = InputManager::IsKeyDown(GLFW_KEY_A);
    bool sDown = InputManager::IsKeyDown(GLFW_KEY_S);
    bool dDown = InputManager::IsKeyDown(GLFW_KEY_D);
    
    LOG_VAR_DEBUG(wDown);
    LOG_VAR_DEBUG(aDown);
    LOG_VAR_DEBUG(sDown);
    LOG_VAR_DEBUG(dDown);
    
    // All movement keys should be up without actual input
    EXPECT_FALSE(wDown);
    EXPECT_FALSE(aDown);
    EXPECT_FALSE(sDown);
    EXPECT_FALSE(dDown);
    LOG_FUNC_EXIT();
}

TEST_F(InputManagerTest, GamingScenario_MouseAim)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    // Simulate mouse aim input
    bool leftClick = InputManager::IsMouseDown(GLFW_MOUSE_BUTTON_LEFT);
    bool rightClick = InputManager::IsMouseDown(GLFW_MOUSE_BUTTON_RIGHT);
    
    double mouseX, mouseY;
    InputManager::GetMousePosition(mouseX, mouseY);
    
    LOG_VAR_DEBUG(leftClick);
    LOG_VAR_DEBUG(rightClick);
    LOG_VAR_DEBUG(mouseX);
    LOG_VAR_DEBUG(mouseY);
    
    // Mouse should be at origin and buttons up without actual input
    EXPECT_FALSE(leftClick);
    EXPECT_FALSE(rightClick);
    EXPECT_DOUBLE_EQ(mouseX, 0.0);
    EXPECT_DOUBLE_EQ(mouseY, 0.0);
    LOG_FUNC_EXIT();
}

// Error recovery tests
TEST_F(InputManagerTest, ErrorRecovery_InvalidOperations)
{
    LOG_FUNC_ENTER();
    // Test recovery from various error conditions
    
    // Initialize with null, then try operations
    InputManager::Init(nullptr);
    
    // These should not cause crashes or undefined behavior
    EXPECT_NO_THROW(({
        InputManager::Update();
        InputManager::IsKeyPressed(GLFW_KEY_A);
        InputManager::IsKeyDown(GLFW_KEY_B);
        InputManager::Update();
        InputManager::IsKeyUp(GLFW_KEY_C);
    }));
    
    // Re-initialize and test again
    GLFWwindow* mockWindow = reinterpret_cast<GLFWwindow*>(0x87654321);
    InputManager::Init(mockWindow);
    
    EXPECT_NO_THROW(({
        InputManager::Update();
        InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT);
    }));
    LOG_FUNC_EXIT();
}

// Memory safety tests
TEST_F(InputManagerTest, MemorySafety_ArrayBounds)
{
    LOG_FUNC_ENTER();
    InputManager::Init(nullptr);
    
    // Test that array access is within bounds
    EXPECT_NO_THROW(({
        // Access all valid key indices
        for (int i = 0; i < GLFW_KEY_LAST; ++i) {
            [[maybe_unused]] bool state = InputManager::IsKeyDown(i);
        }
        
        // Access all valid mouse button indices
        for (int i = 0; i < GLFW_MOUSE_BUTTON_LAST; ++i) {
            [[maybe_unused]] bool state = InputManager::IsMouseDown(i);
        }
    }));
    LOG_FUNC_EXIT();
}