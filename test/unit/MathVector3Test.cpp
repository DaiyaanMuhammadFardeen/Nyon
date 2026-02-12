#include <gtest/gtest.h>
#include "nyon/math/Vector3.h"
#include "TestHelpers.h"

using namespace Nyon::Math;

/**
 * @brief Unit tests for Vector3 mathematical operations.
 * 
 * Tests cover 3D vector arithmetic, cross products, dot products,
 * and normalization operations with comprehensive edge case coverage.
 */
class Vector3Test : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        // Initialize test vectors
        zeroVec = Vector3(0.0f, 0.0f, 0.0f);
        unitX = Vector3(1.0f, 0.0f, 0.0f);
        unitY = Vector3(0.0f, 1.0f, 0.0f);
        unitZ = Vector3(0.0f, 0.0f, 1.0f);
        testVec1 = Vector3(1.0f, 2.0f, 3.0f);
        testVec2 = Vector3(4.0f, 5.0f, 6.0f);
        LOG_FUNC_EXIT();
    }

    Vector3 zeroVec, unitX, unitY, unitZ, testVec1, testVec2;
};

// Constructor tests
TEST_F(Vector3Test, Constructor_Default)
{
    LOG_FUNC_ENTER();
    Vector3 v;
    LOG_VAR_DEBUG(v.x);
    LOG_VAR_DEBUG(v.y);
    LOG_VAR_DEBUG(v.z);
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector3Test, Constructor_Parameters)
{
    LOG_FUNC_ENTER();
    Vector3 v(1.5f, -2.3f, 3.7f);
    LOG_VAR_DEBUG(v.x);
    LOG_VAR_DEBUG(v.y);
    LOG_VAR_DEBUG(v.z);
    EXPECT_FLOAT_EQ(v.x, 1.5f);
    EXPECT_FLOAT_EQ(v.y, -2.3f);
    EXPECT_FLOAT_EQ(v.z, 3.7f);
    LOG_FUNC_EXIT();
}

// Arithmetic operations
TEST_F(Vector3Test, Addition)
{
    LOG_FUNC_ENTER();
    Vector3 result = testVec1 + testVec2;
    LOG_VAR_DEBUG(result.x);
    LOG_VAR_DEBUG(result.y);
    LOG_VAR_DEBUG(result.z);
    EXPECT_FLOAT_NEAR(result.x, 5.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(result.y, 7.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(result.z, 9.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector3Test, Subtraction)
{
    LOG_FUNC_ENTER();
    Vector3 result = testVec2 - testVec1;
    LOG_VAR_DEBUG(result.x);
    LOG_VAR_DEBUG(result.y);
    LOG_VAR_DEBUG(result.z);
    EXPECT_FLOAT_NEAR(result.x, 3.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(result.y, 3.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(result.z, 3.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector3Test, ScalarMultiplication)
{
    LOG_FUNC_ENTER();
    Vector3 result = testVec1 * 2.0f;
    LOG_VAR_DEBUG(result.x);
    LOG_VAR_DEBUG(result.y);
    LOG_VAR_DEBUG(result.z);
    EXPECT_FLOAT_NEAR(result.x, 2.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(result.y, 4.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(result.z, 6.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

// Vector operations
TEST_F(Vector3Test, Length)
{
    LOG_FUNC_ENTER();
    float length = testVec1.Length();
    LOG_VAR_DEBUG(length);
    // sqrt(1² + 2² + 3²) = sqrt(14)
    EXPECT_FLOAT_NEAR(length, std::sqrt(14.0f), 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector3Test, LengthSquared)
{
    LOG_FUNC_ENTER();
    float lengthSq = testVec1.LengthSquared();
    LOG_VAR_DEBUG(lengthSq);
    EXPECT_FLOAT_NEAR(lengthSq, 14.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector3Test, Normalize)
{
    LOG_FUNC_ENTER();
    Vector3 normalized = testVec1.Normalize();
    LOG_VAR_DEBUG(normalized.x);
    LOG_VAR_DEBUG(normalized.y);
    LOG_VAR_DEBUG(normalized.z);
    float length = normalized.Length();
    LOG_VAR_DEBUG(length);
    EXPECT_FLOAT_NEAR(length, 1.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(normalized.x, 1.0f/std::sqrt(14.0f), 1e-6f);
    EXPECT_FLOAT_NEAR(normalized.y, 2.0f/std::sqrt(14.0f), 1e-6f);
    EXPECT_FLOAT_NEAR(normalized.z, 3.0f/std::sqrt(14.0f), 1e-6f);
    LOG_FUNC_EXIT();
}

// Cross product tests
TEST_F(Vector3Test, CrossProduct_Standard)
{
    LOG_FUNC_ENTER();
    Vector3 cross = unitX.Cross(unitY);
    LOG_VAR_DEBUG(cross.x);
    LOG_VAR_DEBUG(cross.y);
    LOG_VAR_DEBUG(cross.z);
    EXPECT_VECTOR3_NEAR(cross, unitZ, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector3Test, CrossProduct_AntiCommutative)
{
    LOG_FUNC_ENTER();
    Vector3 cross1 = unitX.Cross(unitY);
    Vector3 cross2 = unitY.Cross(unitX);
    LOG_VAR_DEBUG(cross1.x);
    LOG_VAR_DEBUG(cross1.y);
    LOG_VAR_DEBUG(cross1.z);
    LOG_VAR_DEBUG(cross2.x);
    LOG_VAR_DEBUG(cross2.y);
    LOG_VAR_DEBUG(cross2.z);
    EXPECT_VECTOR3_NEAR(cross1, -cross2, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector3Test, CrossProduct_Self)
{
    LOG_FUNC_ENTER();
    Vector3 cross = testVec1.Cross(testVec1);
    LOG_VAR_DEBUG(cross.x);
    LOG_VAR_DEBUG(cross.y);
    LOG_VAR_DEBUG(cross.z);
    EXPECT_VECTOR3_NEAR(cross, zeroVec, 1e-6f);
    LOG_FUNC_EXIT();
}

// Dot product tests
TEST_F(Vector3Test, DotProduct_Orthogonal)
{
    LOG_FUNC_ENTER();
    float dot = unitX.Dot(unitY);
    LOG_VAR_DEBUG(dot);
    EXPECT_FLOAT_NEAR(dot, 0.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector3Test, DotProduct_Parallel)
{
    LOG_FUNC_ENTER();
    float dot = unitX.Dot(unitX);
    LOG_VAR_DEBUG(dot);
    EXPECT_FLOAT_NEAR(dot, 1.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector3Test, DotProduct_Angle)
{
    LOG_FUNC_ENTER();
    Vector3 v1(1.0f, 0.0f, 0.0f);
    Vector3 v2(1.0f, 1.0f, 0.0f);
    float dot = v1.Dot(v2.Normalize());
    LOG_VAR_DEBUG(dot);
    EXPECT_FLOAT_NEAR(dot, std::cos(M_PI/4.0f), 1e-6f); // 45 degrees
    LOG_FUNC_EXIT();
}

// Edge cases
TEST_F(Vector3Test, Normalize_ZeroVector)
{
    LOG_FUNC_ENTER();
    Vector3 normalized = zeroVec.Normalize();
    LOG_VAR_DEBUG(normalized.x);
    LOG_VAR_DEBUG(normalized.y);
    LOG_VAR_DEBUG(normalized.z);
    EXPECT_FLOAT_EQ(normalized.x, 0.0f);
    EXPECT_FLOAT_EQ(normalized.y, 0.0f);
    EXPECT_FLOAT_EQ(normalized.z, 0.0f);
    LOG_FUNC_EXIT();
}

TEST_F(Vector3Test, LargeNumbers)
{
    LOG_FUNC_ENTER();
    Vector3 large(1e6f, 1e6f, 1e6f);
    float length = large.Length();
    LOG_VAR_DEBUG(length);
    EXPECT_FLOAT_NEAR(length, 1e6f * std::sqrt(3.0f), 1e-1f);
    LOG_FUNC_EXIT();
}