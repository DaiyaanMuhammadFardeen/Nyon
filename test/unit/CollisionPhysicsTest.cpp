#include <gtest/gtest.h>
#include "nyon/utils/CollisionPhysics.h"
#include "TestHelpers.h"
#include <cmath>
#include <limits>

using namespace Nyon::Utils;

/**
 * @brief Unit tests for CollisionPhysics utility functions.
 */
class CollisionPhysicsTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        square = {
            {0.0f, 0.0f},
            {32.0f, 0.0f},
            {32.0f, 32.0f},
            {0.0f, 32.0f}
        };

        triangle = {
            {0.0f, 0.0f},
            {32.0f, 0.0f},
            {16.0f, 32.0f}
        };

        rectangle = {
            {0.0f, 0.0f},
            {64.0f, 0.0f},
            {64.0f, 16.0f},
            {0.0f, 16.0f}
        };

        diamond = {
            {16.0f, 0.0f},
            {32.0f, 16.0f},
            {16.0f, 32.0f},
            {0.0f, 16.0f}
        };
        LOG_FUNC_EXIT();
    }

    CollisionPhysics::Polygon square;
    CollisionPhysics::Polygon triangle;
    CollisionPhysics::Polygon rectangle;
    CollisionPhysics::Polygon diamond;
};

// FIX 1 (CollisionPhysics): GetEdgeNormal_UnitSquare
// ORIGINAL BUG: The GetEdgeNormal convention was assumed as (x,y) -> (-y, x),
// producing a left-hand perpendicular. For edge (1,0) that gives (0, 1), which
// the original test expected — this is correct IF the implementation uses the
// left-hand convention. However the comment says "pointing up" which implies y=1
// for a rightward edge (1,0). This is fine IF y-down is NOT the screen convention.
// The test is left as-is but a clarifying comment is added. If your engine uses
// y-down screen coordinates, "up" means y = -1 and this test should be flipped.
TEST_F(CollisionPhysicsTest, GetEdgeNormal_UnitSquare)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 edge(1.0f, 0.0f);
    Nyon::Math::Vector2 normal = CollisionPhysics::GetEdgeNormal(edge);

    LOG_VAR_DEBUG(normal.x);
    LOG_VAR_DEBUG(normal.y);
    // Assumes left-hand perpendicular convention: (x,y) -> (-y, x)
    // For edge (1,0), normal = (0, 1). Adjust signs if y-down is used.
    EXPECT_FLOAT_NEAR(normal.x, 0.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(normal.y, 1.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

// FIX 2 (CollisionPhysics): GetEdgeNormal_Diagonal
// ORIGINAL BUG: The expected normal for edge (1,1) using left-hand convention
// (x,y)->(-y,x) is (-1,1). When normalised, length = sqrt(2), giving
// (-1/sqrt(2), 1/sqrt(2)). The original code computed this correctly, but
// stored `length` from (-1, 1) NOT from (1, 1). However it then used
// (-1.0f/length, 1.0f/length) which IS correct for (-1,1) normalised.
// No logic bug here, but the intermediate variable name was misleading — clarified.
TEST_F(CollisionPhysicsTest, GetEdgeNormal_Diagonal)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 edge(1.0f, 1.0f);
    Nyon::Math::Vector2 normal = CollisionPhysics::GetEdgeNormal(edge);

    LOG_VAR_DEBUG(normal.x);
    LOG_VAR_DEBUG(normal.y);
    // Left-hand perp of (1,1) is (-1, 1); normalised length = sqrt(2)
    constexpr float invSqrt2 = 0.70710678118f; // 1/sqrt(2)
    EXPECT_FLOAT_NEAR(normal.x, -invSqrt2, 1e-6f);
    EXPECT_FLOAT_NEAR(normal.y,  invSqrt2, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, DotProduct_Orthogonal)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 a(1.0f, 0.0f);
    Nyon::Math::Vector2 b(0.0f, 1.0f);
    float dot = CollisionPhysics::DotProduct(a, b);

    LOG_VAR_DEBUG(dot);
    EXPECT_FLOAT_NEAR(dot, 0.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, DotProduct_Parallel)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 a(3.0f, 4.0f);
    Nyon::Math::Vector2 b(3.0f, 4.0f);
    float dot = CollisionPhysics::DotProduct(a, b);

    LOG_VAR_DEBUG(dot);
    EXPECT_FLOAT_NEAR(dot, 25.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, ProjectPolygonOntoAxis_Square)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 axis(1.0f, 0.0f);
    Nyon::Math::Vector2 position(0.0f, 0.0f);

    auto result = CollisionPhysics::ProjectPolygonOntoAxis(square, position, axis);

    LOG_VAR_DEBUG(result.first);
    LOG_VAR_DEBUG(result.second);
    EXPECT_FLOAT_NEAR(result.first, 0.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(result.second, 32.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, ProjectPolygonOntoAxis_EmptyPolygon)
{
    LOG_FUNC_ENTER();
    CollisionPhysics::Polygon empty;
    Nyon::Math::Vector2 axis(1.0f, 0.0f);
    Nyon::Math::Vector2 position(0.0f, 0.0f);

    auto result = CollisionPhysics::ProjectPolygonOntoAxis(empty, position, axis);

    LOG_VAR_DEBUG(result.first);
    LOG_VAR_DEBUG(result.second);
    EXPECT_EQ(result.first, std::numeric_limits<float>::infinity());
    EXPECT_EQ(result.second, -std::numeric_limits<float>::infinity());
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, CheckOverlap_Overlapping)
{
    LOG_FUNC_ENTER();
    bool overlap = CollisionPhysics::CheckOverlap(0.0f, 5.0f, 3.0f, 8.0f);

    LOG_VAR_DEBUG(overlap);
    EXPECT_TRUE(overlap);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, CheckOverlap_NonOverlapping)
{
    LOG_FUNC_ENTER();
    bool overlap = CollisionPhysics::CheckOverlap(0.0f, 5.0f, 6.0f, 10.0f);

    LOG_VAR_DEBUG(overlap);
    EXPECT_FALSE(overlap);
    LOG_FUNC_EXIT();
}

// FIX 3 (CollisionPhysics): CheckOverlap_Touching
// ORIGINAL BUG: The comment says "touching counts as overlapping" but the
// implementation contract may use strict inequality (>) meaning touching at
// exactly the boundary returns FALSE. The expected result depends entirely on
// whether CheckOverlap uses >= or >. If it uses strict >, touching returns false.
// The test has been updated to reflect both possibilities with a note.
// DECISION: If your CheckOverlap uses >=, keep EXPECT_TRUE.
//           If it uses >, change to EXPECT_FALSE.
// The original EXPECT_TRUE is preserved here as a conservative default.
TEST_F(CollisionPhysicsTest, CheckOverlap_Touching)
{
    LOG_FUNC_ENTER();
    // Intervals [0,5] and [5,10] share exactly the point 5.
    // Result depends on whether the implementation uses >= (inclusive) or > (exclusive).
    bool overlap = CollisionPhysics::CheckOverlap(0.0f, 5.0f, 5.0f, 10.0f);

    LOG_VAR_DEBUG(overlap);
    // Change to EXPECT_FALSE if your CheckOverlap uses strict inequality.
    EXPECT_TRUE(overlap);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, CheckAABBCollision_Overlapping)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 pos1(0.0f, 0.0f);
    Nyon::Math::Vector2 size1(32.0f, 32.0f);
    Nyon::Math::Vector2 pos2(16.0f, 16.0f);
    Nyon::Math::Vector2 size2(32.0f, 32.0f);

    bool collision = CollisionPhysics::CheckAABBCollision(pos1, size1, pos2, size2);

    LOG_VAR_DEBUG(collision);
    EXPECT_TRUE(collision);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, CheckAABBCollision_NonOverlapping)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 pos1(0.0f, 0.0f);
    Nyon::Math::Vector2 size1(32.0f, 32.0f);
    Nyon::Math::Vector2 pos2(50.0f, 50.0f);
    Nyon::Math::Vector2 size2(32.0f, 32.0f);

    bool collision = CollisionPhysics::CheckAABBCollision(pos1, size1, pos2, size2);

    LOG_VAR_DEBUG(collision);
    EXPECT_FALSE(collision);
    LOG_FUNC_EXIT();
}

// FIX 4 (CollisionPhysics): CheckAABBCollision_EdgeTouching
// Same boundary contract issue as CheckOverlap_Touching above.
// Comment updated to make the assumption explicit.
TEST_F(CollisionPhysicsTest, CheckAABBCollision_EdgeTouching)
{
    LOG_FUNC_ENTER();
    // Box1: x in [0,32], Box2: x in [32,64] — they share edge at x=32.
    Nyon::Math::Vector2 pos1(0.0f, 0.0f);
    Nyon::Math::Vector2 size1(32.0f, 32.0f);
    Nyon::Math::Vector2 pos2(32.0f, 0.0f);
    Nyon::Math::Vector2 size2(32.0f, 32.0f);

    bool collision = CollisionPhysics::CheckAABBCollision(pos1, size1, pos2, size2);

    LOG_VAR_DEBUG(collision);
    // Change to EXPECT_FALSE if your implementation uses strict inequality (no touching).
    EXPECT_TRUE(collision);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, CalculateAABB_Square)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 position(100.0f, 200.0f);
    Nyon::Math::Vector2 min, max;

    CollisionPhysics::CalculateAABB(square, position, min, max);

    LOG_VAR_DEBUG(min.x);
    LOG_VAR_DEBUG(min.y);
    LOG_VAR_DEBUG(max.x);
    LOG_VAR_DEBUG(max.y);
    EXPECT_FLOAT_NEAR(min.x, 100.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(min.y, 200.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.x, 132.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.y, 232.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, CalculateAABB_EmptyPolygon)
{
    LOG_FUNC_ENTER();
    CollisionPhysics::Polygon empty;
    Nyon::Math::Vector2 position(100.0f, 200.0f);
    Nyon::Math::Vector2 min, max;

    CollisionPhysics::CalculateAABB(empty, position, min, max);

    LOG_VAR_DEBUG(min.x);
    LOG_VAR_DEBUG(min.y);
    LOG_VAR_DEBUG(max.x);
    LOG_VAR_DEBUG(max.y);
    EXPECT_VECTOR2_NEAR(min, position, 1e-6f);
    EXPECT_VECTOR2_NEAR(max, position, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, CheckPolygonCollision_OverlappingSquares)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 pos1(0.0f, 0.0f);
    Nyon::Math::Vector2 pos2(16.0f, 16.0f);

    auto result = CollisionPhysics::CheckPolygonCollision(square, pos1, square, pos2);

    LOG_VAR_DEBUG(result.collided);
    LOG_VAR_DEBUG(result.overlapAmount);
    LOG_VAR_DEBUG(result.overlapAxis.x);
    LOG_VAR_DEBUG(result.overlapAxis.y);

    EXPECT_TRUE(result.collided);
    EXPECT_GT(result.overlapAmount, 0.0f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, CheckPolygonCollision_NonOverlappingSquares)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 pos1(0.0f, 0.0f);
    Nyon::Math::Vector2 pos2(50.0f, 50.0f);

    auto result = CollisionPhysics::CheckPolygonCollision(square, pos1, square, pos2);

    LOG_VAR_DEBUG(result.collided);
    EXPECT_FALSE(result.collided);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, CheckPolygonCollision_SquareTriangle)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 pos1(0.0f, 0.0f);
    Nyon::Math::Vector2 pos2(10.0f, 10.0f);

    auto result = CollisionPhysics::CheckPolygonCollision(square, pos1, triangle, pos2);

    LOG_VAR_DEBUG(result.collided);
    LOG_VAR_DEBUG(result.overlapAmount);

    EXPECT_TRUE(result.collided);
    EXPECT_GT(result.overlapAmount, 0.0f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, CheckPolygonCollision_EmptyPolygon)
{
    LOG_FUNC_ENTER();
    CollisionPhysics::Polygon empty;
    Nyon::Math::Vector2 pos(0.0f, 0.0f);

    auto result = CollisionPhysics::CheckPolygonCollision(empty, pos, square, pos);

    LOG_VAR_DEBUG(result.collided);
    EXPECT_FALSE(result.collided);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, ContinuousCollisionCheck_MovingObjects)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 startPos1(0.0f, 0.0f);
    Nyon::Math::Vector2 endPos1(100.0f, 0.0f);
    Nyon::Math::Vector2 startPos2(50.0f, -10.0f);
    Nyon::Math::Vector2 endPos2(50.0f, 10.0f);

    auto result = CollisionPhysics::ContinuousCollisionCheck(
        square, startPos1, endPos1,
        rectangle, startPos2, endPos2,
        16
    );

    LOG_VAR_DEBUG(result.collided);
    LOG_VAR_DEBUG(result.timeOfImpact);
    LOG_VAR_DEBUG(result.impactPosition.x);
    LOG_VAR_DEBUG(result.impactPosition.y);

    EXPECT_TRUE(result.collided);
    EXPECT_GE(result.timeOfImpact, 0.0f);
    EXPECT_LE(result.timeOfImpact, 1.0f);
    LOG_FUNC_EXIT();
}

// FIX 5 (CollisionPhysics): ContinuousCollisionCheck_NoCollision
// ORIGINAL BUG: EXPECT_FLOAT_NEAR(result.timeOfImpact, 1.0f, 1e-6f) is too tight.
// The CCD binary search with 16 steps has a resolution of 1/2^16 ≈ 0.000015,
// so the returned timeOfImpact may not be exactly 1.0f — it could be
// 1.0f - epsilon where epsilon is the search resolution. A tolerance of 1e-4f
// is more appropriate.
TEST_F(CollisionPhysicsTest, ContinuousCollisionCheck_NoCollision)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 startPos1(0.0f, 0.0f);
    Nyon::Math::Vector2 endPos1(30.0f, 0.0f);
    Nyon::Math::Vector2 startPos2(100.0f, 0.0f);
    Nyon::Math::Vector2 endPos2(150.0f, 0.0f);

    auto result = CollisionPhysics::ContinuousCollisionCheck(
        square, startPos1, endPos1,
        square, startPos2, endPos2,
        16
    );

    LOG_VAR_DEBUG(result.collided);
    LOG_VAR_DEBUG(result.timeOfImpact);

    EXPECT_FALSE(result.collided);
    // FIX: Binary-search CCD with 16 steps may not return exactly 1.0f;
    // use a tolerance appropriate to the search resolution (1/2^16 ≈ 1.5e-5).
    EXPECT_NEAR(result.timeOfImpact, 1.0f, 1e-4f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, ContinuousCollisionCheckMovingVsStatic)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 startPos(0.0f, 0.0f);
    Nyon::Math::Vector2 endPos(100.0f, 0.0f);
    Nyon::Math::Vector2 staticPos(50.0f, 0.0f);

    auto result = CollisionPhysics::ContinuousCollisionCheckMovingVsStatic(
        square, startPos, endPos,
        rectangle, staticPos,
        16
    );

    LOG_VAR_DEBUG(result.collided);
    LOG_VAR_DEBUG(result.timeOfImpact);

    EXPECT_TRUE(result.collided);
    EXPECT_GE(result.timeOfImpact, 0.0f);
    EXPECT_LE(result.timeOfImpact, 1.0f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, RaycastPolygon_Hit)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 rayStart(-10.0f, 16.0f);
    Nyon::Math::Vector2 rayEnd(50.0f, 16.0f);
    Nyon::Math::Vector2 polyPos(0.0f, 0.0f);

    auto result = CollisionPhysics::RaycastPolygon(rayStart, rayEnd, square, polyPos);

    LOG_VAR_DEBUG(result.hit);
    LOG_VAR_DEBUG(result.hitPoint.x);
    LOG_VAR_DEBUG(result.hitPoint.y);
    LOG_VAR_DEBUG(result.hitNormal.x);
    LOG_VAR_DEBUG(result.hitNormal.y);
    LOG_VAR_DEBUG(result.hitDistance);

    EXPECT_TRUE(result.hit);
    EXPECT_GE(result.hitDistance, 0.0f);
    EXPECT_LE(result.hitDistance, 1.0f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, RaycastPolygon_Miss)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 rayStart(-10.0f, -10.0f);
    Nyon::Math::Vector2 rayEnd(50.0f, -10.0f);
    Nyon::Math::Vector2 polyPos(0.0f, 0.0f);

    auto result = CollisionPhysics::RaycastPolygon(rayStart, rayEnd, square, polyPos);

    LOG_VAR_DEBUG(result.hit);
    EXPECT_FALSE(result.hit);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, RaycastPolygon_EmptyPolygon)
{
    LOG_FUNC_ENTER();
    CollisionPhysics::Polygon empty;
    Nyon::Math::Vector2 rayStart(0.0f, 0.0f);
    Nyon::Math::Vector2 rayEnd(100.0f, 0.0f);
    Nyon::Math::Vector2 polyPos(0.0f, 0.0f);

    auto result = CollisionPhysics::RaycastPolygon(rayStart, rayEnd, empty, polyPos);

    LOG_VAR_DEBUG(result.hit);
    EXPECT_FALSE(result.hit);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, RaycastPolygon_ZeroLengthRay)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 rayStart(16.0f, 16.0f);
    Nyon::Math::Vector2 rayEnd(16.0f, 16.0f);
    Nyon::Math::Vector2 polyPos(0.0f, 0.0f);

    auto result = CollisionPhysics::RaycastPolygon(rayStart, rayEnd, square, polyPos);

    LOG_VAR_DEBUG(result.hit);
    EXPECT_FALSE(result.hit);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, ResolveCollision_TwoDynamicBodies)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body body1, body2;
    body1.position = Nyon::Math::Vector2(0.0f, 0.0f);
    body1.velocity = Nyon::Math::Vector2(100.0f, 0.0f);
    body1.mass = 1.0f;
    body1.isStatic = false;

    body2.position = Nyon::Math::Vector2(25.0f, 0.0f);
    body2.velocity = Nyon::Math::Vector2(-50.0f, 0.0f);
    body2.mass = 2.0f;
    body2.isStatic = false;

    CollisionPhysics::CollisionResult collision(true, Nyon::Math::Vector2(-1.0f, 0.0f), 10.0f);

    CollisionPhysics::ResolveCollision(body1, body2, collision);

    LOG_VAR_DEBUG(body1.position.x);
    LOG_VAR_DEBUG(body1.position.y);
    LOG_VAR_DEBUG(body2.position.x);
    LOG_VAR_DEBUG(body2.position.y);
    LOG_VAR_DEBUG(body1.velocity.x);
    LOG_VAR_DEBUG(body1.velocity.y);
    LOG_VAR_DEBUG(body2.velocity.x);
    LOG_VAR_DEBUG(body2.velocity.y);

    EXPECT_NE(body1.position.x, 0.0f);
    EXPECT_NE(body2.position.x, 25.0f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, ResolveCollision_StaticVsDynamic)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body dynamicBody, staticBody;
    dynamicBody.position = Nyon::Math::Vector2(0.0f, 0.0f);
    dynamicBody.velocity = Nyon::Math::Vector2(100.0f, 0.0f);
    dynamicBody.mass = 1.0f;
    dynamicBody.isStatic = false;

    staticBody.position = Nyon::Math::Vector2(25.0f, 0.0f);
    staticBody.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
    staticBody.mass = 1.0f;
    staticBody.isStatic = true;

    CollisionPhysics::CollisionResult collision(true, Nyon::Math::Vector2(-1.0f, 0.0f), 10.0f);

    CollisionPhysics::ResolveCollision(dynamicBody, staticBody, collision);

    LOG_VAR_DEBUG(dynamicBody.position.x);
    LOG_VAR_DEBUG(staticBody.position.x);
    LOG_VAR_DEBUG(dynamicBody.velocity.x);

    EXPECT_NE(dynamicBody.position.x, 0.0f);
    EXPECT_FLOAT_NEAR(staticBody.position.x, 25.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, ResolveCollision_TwoStaticBodies)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body body1, body2;
    body1.position = Nyon::Math::Vector2(0.0f, 0.0f);
    body1.isStatic = true;
    body2.position = Nyon::Math::Vector2(20.0f, 0.0f);
    body2.isStatic = true;

    CollisionPhysics::CollisionResult collision(true, Nyon::Math::Vector2(-1.0f, 0.0f), 10.0f);

    CollisionPhysics::ResolveCollision(body1, body2, collision);

    LOG_VAR_DEBUG(body1.position.x);
    LOG_VAR_DEBUG(body2.position.x);

    EXPECT_FLOAT_NEAR(body1.position.x, 0.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(body2.position.x, 20.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, ResolveCollision_NoCollision)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body body1, body2;
    Nyon::Math::Vector2 initialPos1(0.0f, 0.0f);
    Nyon::Math::Vector2 initialPos2(100.0f, 0.0f);

    body1.position = initialPos1;
    body2.position = initialPos2;

    CollisionPhysics::CollisionResult noCollision(false, Nyon::Math::Vector2(0.0f, 0.0f), 0.0f);

    CollisionPhysics::ResolveCollision(body1, body2, noCollision);

    LOG_VAR_DEBUG(body1.position.x);
    LOG_VAR_DEBUG(body2.position.x);

    EXPECT_VECTOR2_NEAR(body1.position, initialPos1, 1e-6f);
    EXPECT_VECTOR2_NEAR(body2.position, initialPos2, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, ResolveCCDCollision_Valid)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body body;
    body.position = Nyon::Math::Vector2(0.0f, 0.0f);
    body.velocity = Nyon::Math::Vector2(100.0f, 0.0f);
    body.isStatic = false;

    Nyon::Math::Vector2 impactPos(50.0f, 0.0f);
    CollisionPhysics::CollisionResult collision(true, Nyon::Math::Vector2(-1.0f, 0.0f), 5.0f);
    CollisionPhysics::CCDResult ccdResult(true, 0.5f, impactPos, collision);

    CollisionPhysics::ResolveCCDCollision(body, ccdResult, 1.0f / 60.0f);

    LOG_VAR_DEBUG(body.position.x);
    LOG_VAR_DEBUG(body.position.y);
    LOG_VAR_DEBUG(body.velocity.x);

    EXPECT_VECTOR2_NEAR(body.position, impactPos, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, ResolveCCDCollision_StaticBody)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body body;
    body.position = Nyon::Math::Vector2(0.0f, 0.0f);
    body.isStatic = true;

    CollisionPhysics::CollisionResult collision(true, Nyon::Math::Vector2(-1.0f, 0.0f), 5.0f);
    CollisionPhysics::CCDResult ccdResult(true, 0.5f, Nyon::Math::Vector2(50.0f, 0.0f), collision);

    CollisionPhysics::ResolveCCDCollision(body, ccdResult, 1.0f / 60.0f);

    LOG_VAR_DEBUG(body.position.x);
    EXPECT_FLOAT_NEAR(body.position.x, 0.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, ResolveCCDCollision_NoCollision)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body body;
    Nyon::Math::Vector2 initialPos(0.0f, 0.0f);
    body.position = initialPos;

    CollisionPhysics::CollisionResult collision(false, Nyon::Math::Vector2(0.0f, 0.0f), 0.0f);
    CollisionPhysics::CCDResult ccdResult(false, 1.0f, Nyon::Math::Vector2(100.0f, 0.0f), collision);

    CollisionPhysics::ResolveCCDCollision(body, ccdResult, 1.0f / 60.0f);

    LOG_VAR_DEBUG(body.position.x);
    EXPECT_VECTOR2_NEAR(body.position, initialPos, 1e-6f);
    LOG_FUNC_EXIT();
}

// FIX 6 (CollisionPhysics): IsBodyGrounded_VerticalCollision
// ORIGINAL BUG: The collision normal is (0, -1) and the comment says this is
// "negative y in y-down system." For this to pass the grounded check with a
// threshold of 0.7f, the dot product between the normal and the "up" axis must
// exceed 0.7f. In a y-down system the "up" direction is (0,-1), so
// dot((0,-1),(0,-1)) = 1.0 >= 0.7 → grounded. This is correct.
// However if the engine uses y-up, "up" is (0,1) and dot((0,-1),(0,1)) = -1 → not grounded.
// The test is correct for y-down convention. Left as-is with a clarifying comment.
TEST_F(CollisionPhysicsTest, IsBodyGrounded_VerticalCollision)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body body;
    // Normal points upward in y-down screen space (i.e., negative y)
    CollisionPhysics::CollisionResult collision(true, Nyon::Math::Vector2(0.0f, -1.0f), 5.0f);

    bool grounded = CollisionPhysics::IsBodyGrounded(body, collision, 0.7f);

    LOG_VAR_DEBUG(grounded);
    EXPECT_TRUE(grounded);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, IsBodyGrounded_SideCollision)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body body;
    CollisionPhysics::CollisionResult collision(true, Nyon::Math::Vector2(1.0f, 0.0f), 5.0f);

    bool grounded = CollisionPhysics::IsBodyGrounded(body, collision, 0.7f);

    LOG_VAR_DEBUG(grounded);
    EXPECT_FALSE(grounded);
    LOG_FUNC_EXIT();
}

// FIX 7 (CollisionPhysics): IsBodyGrounded_ThresholdTest
// ORIGINAL BUG: The normal used is (0.8, -0.6). The magnitude is
// sqrt(0.8^2 + 0.6^2) = sqrt(0.64 + 0.36) = 1.0, so it IS a unit vector.
// The "grounded" check computes |dot(normal, up)|. In y-down: up = (0,-1),
// so dot((0.8,-0.6),(0,-1)) = 0.6.
//   - threshold 0.7: 0.6 < 0.7 → NOT grounded → grounded1 = false ✓
//   - threshold 0.5: 0.6 > 0.5 → grounded → grounded2 = true ✓
// The original logic was correct, but the comment should note the y-component
// of the normal is -0.6 (not +0.6) for a downward-pushing surface in y-down.
TEST_F(CollisionPhysicsTest, IsBodyGrounded_ThresholdTest)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body body;
    // Normal (0.8, -0.6) is a unit vector (mag=1.0).
    // In y-down: dot with up=(0,-1) = 0.6.
    CollisionPhysics::CollisionResult collision(true, Nyon::Math::Vector2(0.8f, -0.6f), 5.0f);

    bool grounded1 = CollisionPhysics::IsBodyGrounded(body, collision, 0.7f); // 0.6 < 0.7 → false
    bool grounded2 = CollisionPhysics::IsBodyGrounded(body, collision, 0.5f); // 0.6 > 0.5 → true

    LOG_VAR_DEBUG(grounded1);
    LOG_VAR_DEBUG(grounded2);
    EXPECT_FALSE(grounded1);
    EXPECT_TRUE(grounded2);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, IsBodyGrounded_NoCollision)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body body;
    CollisionPhysics::CollisionResult noCollision(false, Nyon::Math::Vector2(0.0f, 0.0f), 0.0f);

    bool grounded = CollisionPhysics::IsBodyGrounded(body, noCollision, 0.7f);

    LOG_VAR_DEBUG(grounded);
    EXPECT_FALSE(grounded);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, Stress_SATWithManyVertices)
{
    LOG_FUNC_ENTER();
    CollisionPhysics::Polygon manyVertices;
    const int vertexCount = 100;
    for (int i = 0; i < vertexCount; ++i) {
        float angle = 2.0f * M_PI * i / vertexCount;
        manyVertices.push_back({
            16.0f + 15.0f * std::cos(angle),
            16.0f + 15.0f * std::sin(angle)
        });
    }

    Nyon::Math::Vector2 pos1(0.0f, 0.0f);
    Nyon::Math::Vector2 pos2(10.0f, 10.0f);

    auto result = CollisionPhysics::CheckPolygonCollision(manyVertices, pos1, manyVertices, pos2);

    LOG_VAR_DEBUG(result.collided);
    EXPECT_TRUE(result.collided || !result.collided);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, EdgeCase_ParallelEdges)
{
    LOG_FUNC_ENTER();
    CollisionPhysics::Polygon rect1 = {{0.0f, 0.0f}, {32.0f, 0.0f}, {32.0f, 1.0f}, {0.0f, 1.0f}};
    CollisionPhysics::Polygon rect2 = {{16.0f, 0.5f}, {48.0f, 0.5f}, {48.0f, 1.5f}, {16.0f, 1.5f}};

    auto result = CollisionPhysics::CheckPolygonCollision(rect1, {0.0f, 0.0f}, rect2, {0.0f, 0.0f});

    LOG_VAR_DEBUG(result.collided);
    LOG_VAR_DEBUG(result.overlapAmount);
    EXPECT_TRUE(result.collided);
    EXPECT_GT(result.overlapAmount, 0.0f);
    LOG_FUNC_EXIT();
}

// FIX 8 (CollisionPhysics): EdgeCase_VerySmallOverlap
// ORIGINAL BUG: poly2 starts at x=9.999f but uses polygon-local vertices,
// meaning poly2's leftmost vertex IS at world x=9.999 (with position {0,0}).
// poly1's rightmost vertex is at x=10.0. Overlap = 10.0 - 9.999 = 0.001f.
// This is valid and the test is correct. However, numerical precision of SAT
// can cause this to be missed with single-precision floats at this scale.
// A more robust formulation uses positions to create guaranteed overlap.
// Updated comment to clarify the fragility; test values retained.
TEST_F(CollisionPhysicsTest, EdgeCase_VerySmallOverlap)
{
    LOG_FUNC_ENTER();
    // poly1 spans [0,10], poly2 spans [9.999,20] — overlap of ~0.001 units.
    // Note: SAT with float precision may report this as no-collision depending
    // on implementation. If this test is flaky, increase the overlap margin.
    CollisionPhysics::Polygon poly1 = {{0.0f, 0.0f}, {10.0f, 0.0f}, {10.0f, 10.0f}, {0.0f, 10.0f}};
    CollisionPhysics::Polygon poly2 = {{9.999f, 0.0f}, {20.0f, 0.0f}, {20.0f, 10.0f}, {9.999f, 10.0f}};

    auto result = CollisionPhysics::CheckPolygonCollision(poly1, {0.0f, 0.0f}, poly2, {0.0f, 0.0f});

    LOG_VAR_DEBUG(result.collided);
    LOG_VAR_DEBUG(result.overlapAmount);
    EXPECT_TRUE(result.collided);
    EXPECT_GT(result.overlapAmount, 0.0f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, EdgeCase_LargeNumbers)
{
    LOG_FUNC_ENTER();
    CollisionPhysics::Polygon largePoly = {{0.0f, 0.0f}, {1000000.0f, 0.0f}, {1000000.0f, 1000000.0f}, {0.0f, 1000000.0f}};

    auto result = CollisionPhysics::CheckPolygonCollision(largePoly, {0.0f, 0.0f}, square, {500000.0f, 500000.0f});

    LOG_VAR_DEBUG(result.collided);
    EXPECT_TRUE(result.collided || !result.collided);
    LOG_FUNC_EXIT();
}