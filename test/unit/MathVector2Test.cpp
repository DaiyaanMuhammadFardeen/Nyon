#include <gtest/gtest.h>
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include "TestHelpers.h"

using namespace Nyon::Math;

/**
 * @brief Unit tests for Vector2 mathematical operations.
 * 
 * Tests cover basic arithmetic, vector operations, normalization,
 * and edge cases to ensure mathematical correctness.
 */
class Vector2Test : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        // Initialize test vectors
        zeroVec = Vector2(0.0f, 0.0f);
        unitX = Vector2(1.0f, 0.0f);
        unitY = Vector2(0.0f, 1.0f);
        testVec1 = Vector2(3.0f, 4.0f);
        testVec2 = Vector2(-2.0f, 5.0f);
        LOG_FUNC_EXIT();
    }

    Vector2 zeroVec, unitX, unitY, testVec1, testVec2;
};

// Constructor tests
TEST_F(Vector2Test, Constructor_Default)
{
    LOG_FUNC_ENTER();
    Vector2 v;
    LOG_VAR_DEBUG(v.x);
    LOG_VAR_DEBUG(v.y);
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector2Test, Constructor_Parameters)
{
    LOG_FUNC_ENTER();
    Vector2 v(3.5f, -2.1f);
    LOG_VAR_DEBUG(v.x);
    LOG_VAR_DEBUG(v.y);
    EXPECT_FLOAT_EQ(v.x, 3.5f);
    EXPECT_FLOAT_EQ(v.y, -2.1f);
    LOG_FUNC_EXIT();
}

// Arithmetic operations
TEST_F(Vector2Test, Addition)
{
    LOG_FUNC_ENTER();
    Vector2 result = testVec1 + testVec2;
    LOG_VAR_DEBUG(result.x);
    LOG_VAR_DEBUG(result.y);
    EXPECT_FLOAT_NEAR(result.x, 1.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(result.y, 9.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector2Test, Subtraction)
{
    LOG_FUNC_ENTER();
    Vector2 result = testVec1 - testVec2;
    LOG_VAR_DEBUG(result.x);
    LOG_VAR_DEBUG(result.y);
    EXPECT_FLOAT_NEAR(result.x, 5.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(result.y, -1.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector2Test, ScalarMultiplication)
{
    LOG_FUNC_ENTER();
    Vector2 result = testVec1 * 2.5f;
    LOG_VAR_DEBUG(result.x);
    LOG_VAR_DEBUG(result.y);
    EXPECT_FLOAT_NEAR(result.x, 7.5f, 1e-6f);
    EXPECT_FLOAT_NEAR(result.y, 10.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector2Test, ScalarDivision)
{
    LOG_FUNC_ENTER();
    Vector2 result = testVec1 / 2.0f;
    LOG_VAR_DEBUG(result.x);
    LOG_VAR_DEBUG(result.y);
    EXPECT_FLOAT_NEAR(result.x, 1.5f, 1e-6f);
    EXPECT_FLOAT_NEAR(result.y, 2.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

// Compound assignment operators
TEST_F(Vector2Test, AdditionAssignment)
{
    LOG_FUNC_ENTER();
    Vector2 v = testVec1;
    v += testVec2;
    LOG_VAR_DEBUG(v.x);
    LOG_VAR_DEBUG(v.y);
    EXPECT_FLOAT_NEAR(v.x, 1.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(v.y, 9.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector2Test, SubtractionAssignment)
{
    LOG_FUNC_ENTER();
    Vector2 v = testVec1;
    v -= testVec2;
    LOG_VAR_DEBUG(v.x);
    LOG_VAR_DEBUG(v.y);
    EXPECT_FLOAT_NEAR(v.x, 5.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(v.y, -1.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

// Vector operations
TEST_F(Vector2Test, Length)
{
    LOG_FUNC_ENTER();
    float length = testVec1.Length();
    LOG_VAR_DEBUG(length);
    EXPECT_FLOAT_NEAR(length, 5.0f, 1e-6f); // 3-4-5 triangle
    LOG_FUNC_EXIT();
}

TEST_F(Vector2Test, LengthSquared)
{
    LOG_FUNC_ENTER();
    float lengthSq = testVec1.LengthSquared();
    LOG_VAR_DEBUG(lengthSq);
    EXPECT_FLOAT_NEAR(lengthSq, 25.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector2Test, Normalize)
{
    LOG_FUNC_ENTER();
    Vector2 normalized = testVec1.Normalize();
    LOG_VAR_DEBUG(normalized.x);
    LOG_VAR_DEBUG(normalized.y);
    float length = normalized.Length();
    LOG_VAR_DEBUG(length);
    EXPECT_FLOAT_NEAR(length, 1.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(normalized.x, 0.6f, 1e-6f);
    EXPECT_FLOAT_NEAR(normalized.y, 0.8f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector2Test, Normalize_ZeroVector)
{
    LOG_FUNC_ENTER();
    Vector2 normalized = zeroVec.Normalize();
    LOG_VAR_DEBUG(normalized.x);
    LOG_VAR_DEBUG(normalized.y);
    EXPECT_FLOAT_EQ(normalized.x, 0.0f);
    EXPECT_FLOAT_EQ(normalized.y, 0.0f);
    LOG_FUNC_EXIT();
}

// Edge cases
TEST_F(Vector2Test, LargeNumbers)
{
    LOG_FUNC_ENTER();
    Vector2 large(1e6f, 1e6f);
    float length = large.Length();
    LOG_VAR_DEBUG(length);
    EXPECT_FLOAT_NEAR(length, 1e6f * std::sqrt(2.0f), 1e-1f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector2Test, SmallNumbers)
{
    LOG_FUNC_ENTER();
    Vector2 small(1e-6f, 1e-6f);
    Vector2 normalized = small.Normalize();
    LOG_VAR_DEBUG(normalized.x);
    LOG_VAR_DEBUG(normalized.y);
    float length = normalized.Length();
    LOG_VAR_DEBUG(length);
    EXPECT_FLOAT_NEAR(length, 1.0f, 1e-3f);
    LOG_FUNC_EXIT();
}