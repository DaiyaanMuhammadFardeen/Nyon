#pragma once

#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include "TestLogger.h"
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"

/**
 * @brief Custom assertions and helper functions for Nyon engine testing.
 * 
 * Provides specialized assertions for vector comparison, floating-point equality,
 * and ECS component testing with detailed error reporting.
 */
namespace NyonTest
{
    // Floating point comparison with epsilon
    inline ::testing::AssertionResult FloatNear(float a, float b, float epsilon = 1e-5f)
    {
        float diff = std::abs(a - b);
        if (diff <= epsilon) {
            return ::testing::AssertionSuccess();
        } else {
            return ::testing::AssertionFailure() 
                << "Expected: " << a << " (difference: " << diff << ")"
                << " Actual: " << b 
                << " Epsilon: " << epsilon;
        }
    }

    // Vector2 comparison
    inline ::testing::AssertionResult Vector2Near(const Nyon::Math::Vector2& a, 
                                                 const Nyon::Math::Vector2& b, 
                                                 float epsilon = 1e-5f)
    {
        if (FloatNear(a.x, b.x, epsilon) && FloatNear(a.y, b.y, epsilon)) {
            return ::testing::AssertionSuccess();
        } else {
            return ::testing::AssertionFailure() 
                << "Vector2 mismatch:\n"
                << "  Expected: (" << a.x << ", " << a.y << ")\n"
                << "  Actual:   (" << b.x << ", " << b.y << ")\n"
                << "  Epsilon: " << epsilon;
        }
    }

    // Vector3 comparison
    inline ::testing::AssertionResult Vector3Near(const Nyon::Math::Vector3& a, 
                                                 const Nyon::Math::Vector3& b, 
                                                 float epsilon = 1e-5f)
    {
        if (FloatNear(a.x, b.x, epsilon) && 
            FloatNear(a.y, b.y, epsilon) && 
            FloatNear(a.z, b.z, epsilon)) {
            return ::testing::AssertionSuccess();
        } else {
            return ::testing::AssertionFailure() 
                << "Vector3 mismatch:\n"
                << "  Expected: (" << a.x << ", " << a.y << ", " << a.z << ")\n"
                << "  Actual:   (" << b.x << ", " << b.y << ", " << b.z << ")\n"
                << "  Epsilon: " << epsilon;
        }
    }

    // Utility function to create test entities
    // Commented out due to namespace issues
    /*
    inline Nyon::ECS::EntityID CreateTestEntity(Nyon::ECS::EntityManager& entityManager)
    {
        LOG_FUNC_ENTER();
        Nyon::ECS::EntityID entity = entityManager.CreateEntity();
        LOG_VAR_DEBUG(entity);
        LOG_FUNC_EXIT();
        return entity;
    }
    */

    // Performance measurement helper
    class PerformanceTimer
    {
    public:
        PerformanceTimer(const std::string& testName) : m_TestName(testName)
        {
            m_Start = std::chrono::high_resolution_clock::now();
            LOG_INFO("Starting performance test: " + testName);
        }

        ~PerformanceTimer()
        {
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - m_Start);
            LOG_INFO("Performance test '" + m_TestName + "' completed in " + 
                    std::to_string(duration.count()) + " microseconds");
        }

    private:
        std::string m_TestName;
        std::chrono::high_resolution_clock::time_point m_Start;
    };

    #define PERF_TIMER(name) PerformanceTimer timer__(name)
}

// Custom assertion macros
#define ASSERT_FLOAT_NEAR(a, b, eps) ASSERT_TRUE(NyonTest::FloatNear(a, b, eps))
#define EXPECT_FLOAT_NEAR(a, b, eps) EXPECT_TRUE(NyonTest::FloatNear(a, b, eps))
#define ASSERT_VECTOR2_NEAR(a, b, eps) ASSERT_TRUE(NyonTest::Vector2Near(a, b, eps))
#define EXPECT_VECTOR2_NEAR(a, b, eps) EXPECT_TRUE(NyonTest::Vector2Near(a, b, eps))
#define ASSERT_VECTOR3_NEAR(a, b, eps) ASSERT_TRUE(NyonTest::Vector3Near(a, b, eps))
#define EXPECT_VECTOR3_NEAR(a, b, eps) EXPECT_TRUE(NyonTest::Vector3Near(a, b, eps))