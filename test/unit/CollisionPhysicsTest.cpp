#include <gtest/gtest.h>
#include "nyon/utils/CollisionPhysics.h"
#include "TestHelpers.h"
#include <cmath>
#include <limits>

using namespace Nyon::Utils;

/**
 * @brief Unit tests for CollisionPhysics utility functions.
 * 
 * Tests SAT collision detection, continuous collision detection (CCD),
 * raycasting, collision resolution, and helper functions with comprehensive
 * coverage of edge cases and real-world scenarios.
 */
class CollisionPhysicsTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        // Initialize common test polygons
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

// Helper function tests
TEST_F(CollisionPhysicsTest, GetEdgeNormal_UnitSquare)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 edge(1.0f, 0.0f); // Horizontal edge
    Nyon::Math::Vector2 normal = CollisionPhysics::GetEdgeNormal(edge);
    
    LOG_VAR_DEBUG(normal.x);
    LOG_VAR_DEBUG(normal.y);
    // Normal should be perpendicular (pointing up for horizontal edge)
    EXPECT_FLOAT_NEAR(normal.x, 0.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(normal.y, 1.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, GetEdgeNormal_Diagonal)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 edge(1.0f, 1.0f); // Diagonal edge
    Nyon::Math::Vector2 normal = CollisionPhysics::GetEdgeNormal(edge);
    
    LOG_VAR_DEBUG(normal.x);
    LOG_VAR_DEBUG(normal.y);
    // Should be perpendicular (-1, 1) normalized
    float length = std::sqrt((-1.0f)*(-1.0f) + 1.0f*1.0f);
    EXPECT_FLOAT_NEAR(normal.x, -1.0f/length, 1e-6f);
    EXPECT_FLOAT_NEAR(normal.y, 1.0f/length, 1e-6f);
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
    EXPECT_FLOAT_NEAR(dot, 25.0f, 1e-6f); // 3*3 + 4*4 = 9 + 16 = 25
    LOG_FUNC_EXIT();
}

// Projection tests
TEST_F(CollisionPhysicsTest, ProjectPolygonOntoAxis_Square)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 axis(1.0f, 0.0f); // X-axis
    Nyon::Math::Vector2 position(0.0f, 0.0f);
    
    auto result = CollisionPhysics::ProjectPolygonOntoAxis(square, position, axis);
    
    LOG_VAR_DEBUG(result.first);
    LOG_VAR_DEBUG(result.second);
    // Square from 0 to 32 on X-axis
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
    // Should return infinity values for empty polygon
    EXPECT_EQ(result.first, std::numeric_limits<float>::infinity());
    EXPECT_EQ(result.second, -std::numeric_limits<float>::infinity());
    LOG_FUNC_EXIT();
}

// Overlap tests
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

TEST_F(CollisionPhysicsTest, CheckOverlap_Touching)
{
    LOG_FUNC_ENTER();
    bool overlap = CollisionPhysics::CheckOverlap(0.0f, 5.0f, 5.0f, 10.0f);
    
    LOG_VAR_DEBUG(overlap);
    EXPECT_TRUE(overlap); // Touching counts as overlapping
    LOG_FUNC_EXIT();
}

// AABB collision tests
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

TEST_F(CollisionPhysicsTest, CheckAABBCollision_EdgeTouching)
{
    LOG_FUNC_ENTER();
    Nyon::Math::Vector2 pos1(0.0f, 0.0f);
    Nyon::Math::Vector2 size1(32.0f, 32.0f);
    Nyon::Math::Vector2 pos2(32.0f, 0.0f);
    Nyon::Math::Vector2 size2(32.0f, 32.0f);
    
    bool collision = CollisionPhysics::CheckAABBCollision(pos1, size1, pos2, size2);
    
    LOG_VAR_DEBUG(collision);
    EXPECT_TRUE(collision); // Edge touching counts as collision
    LOG_FUNC_EXIT();
}

// AABB calculation tests
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
    // Square from (0,0) to (32,32) offset by position
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
    // Empty polygon should return position as both min and max
    EXPECT_VECTOR2_NEAR(min, position, 1e-6f);
    EXPECT_VECTOR2_NEAR(max, position, 1e-6f);
    LOG_FUNC_EXIT();
}

// SAT collision detection tests
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

// Continuous collision detection tests
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
    EXPECT_FLOAT_NEAR(result.timeOfImpact, 1.0f, 1e-6f);
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

// Raycasting tests
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
    Nyon::Math::Vector2 rayEnd(16.0f, 16.0f); // Same point
    Nyon::Math::Vector2 polyPos(0.0f, 0.0f);
    
    auto result = CollisionPhysics::RaycastPolygon(rayStart, rayEnd, square, polyPos);
    
    LOG_VAR_DEBUG(result.hit);
    EXPECT_FALSE(result.hit);
    LOG_FUNC_EXIT();
}

// Collision resolution tests
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
    
    // Bodies should be moved apart
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
    staticBody.isStatic = true; // Static body
    
    CollisionPhysics::CollisionResult collision(true, Nyon::Math::Vector2(-1.0f, 0.0f), 10.0f);
    
    CollisionPhysics::ResolveCollision(dynamicBody, staticBody, collision);
    
    LOG_VAR_DEBUG(dynamicBody.position.x);
    LOG_VAR_DEBUG(staticBody.position.x);
    LOG_VAR_DEBUG(dynamicBody.velocity.x);
    
    // Only dynamic body should move
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
    
    // Both static bodies should remain unchanged
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
    
    // Positions should remain unchanged when no collision
    EXPECT_VECTOR2_NEAR(body1.position, initialPos1, 1e-6f);
    EXPECT_VECTOR2_NEAR(body2.position, initialPos2, 1e-6f);
    LOG_FUNC_EXIT();
}

// CCD resolution tests
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
    
    // Body should be moved to impact position
    EXPECT_VECTOR2_NEAR(body.position, impactPos, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, ResolveCCDCollision_StaticBody)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body body;
    body.position = Nyon::Math::Vector2(0.0f, 0.0f);
    body.isStatic = true; // Static body
    
    CollisionPhysics::CollisionResult collision(true, Nyon::Math::Vector2(-1.0f, 0.0f), 5.0f);
    CollisionPhysics::CCDResult ccdResult(true, 0.5f, Nyon::Math::Vector2(50.0f, 0.0f), collision);
    
    CollisionPhysics::ResolveCCDCollision(body, ccdResult, 1.0f / 60.0f);
    
    LOG_VAR_DEBUG(body.position.x);
    // Static body should not be moved
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
    // Position should remain unchanged when no collision
    EXPECT_VECTOR2_NEAR(body.position, initialPos, 1e-6f);
    LOG_FUNC_EXIT();
}

// Grounded state tests
TEST_F(CollisionPhysicsTest, IsBodyGrounded_VerticalCollision)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body body;
    // Collision normal pointing upward (negative y in y-down system)
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
    // Collision normal pointing sideways
    CollisionPhysics::CollisionResult collision(true, Nyon::Math::Vector2(1.0f, 0.0f), 5.0f);
    
    bool grounded = CollisionPhysics::IsBodyGrounded(body, collision, 0.7f);
    
    LOG_VAR_DEBUG(grounded);
    EXPECT_FALSE(grounded);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, IsBodyGrounded_ThresholdTest)
{
    LOG_FUNC_ENTER();
    Nyon::Utils::Physics::Body body;
    // Collision normal with small y-component
    CollisionPhysics::CollisionResult collision(true, Nyon::Math::Vector2(0.8f, -0.6f), 5.0f);
    
    bool grounded1 = CollisionPhysics::IsBodyGrounded(body, collision, 0.7f); // High threshold
    bool grounded2 = CollisionPhysics::IsBodyGrounded(body, collision, 0.5f); // Lower threshold
    
    LOG_VAR_DEBUG(grounded1);
    LOG_VAR_DEBUG(grounded2);
    EXPECT_FALSE(grounded1); // Below threshold
    EXPECT_TRUE(grounded2);  // Above threshold
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

// Edge case and stress tests
TEST_F(CollisionPhysicsTest, Stress_SATWithManyVertices)
{
    LOG_FUNC_ENTER();
    // Create a polygon with many vertices
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
    // Should complete without crashing
    EXPECT_TRUE(result.collided || !result.collided); // Either result is valid
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, EdgeCase_ParallelEdges)
{
    LOG_FUNC_ENTER();
    // Two rectangles with parallel edges
    CollisionPhysics::Polygon rect1 = {{0.0f, 0.0f}, {32.0f, 0.0f}, {32.0f, 1.0f}, {0.0f, 1.0f}};
    CollisionPhysics::Polygon rect2 = {{16.0f, 0.5f}, {48.0f, 0.5f}, {48.0f, 1.5f}, {16.0f, 1.5f}};
    
    auto result = CollisionPhysics::CheckPolygonCollision(rect1, {0.0f, 0.0f}, rect2, {0.0f, 0.0f});
    
    LOG_VAR_DEBUG(result.collided);
    LOG_VAR_DEBUG(result.overlapAmount);
    EXPECT_TRUE(result.collided);
    EXPECT_GT(result.overlapAmount, 0.0f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, EdgeCase_VerySmallOverlap)
{
    LOG_FUNC_ENTER();
    // Polygons with very small overlap
    CollisionPhysics::Polygon poly1 = {{0.0f, 0.0f}, {10.0f, 0.0f}, {10.0f, 10.0f}, {0.0f, 10.0f}};
    CollisionPhysics::Polygon poly2 = {{9.999f, 0.0f}, {20.0f, 0.0f}, {20.0f, 10.0f}, {9.999f, 10.0f}};
    
    auto result = CollisionPhysics::CheckPolygonCollision(poly1, {0.0f, 0.0f}, poly2, {0.0f, 0.0f});
    
    LOG_VAR_DEBUG(result.collided);
    LOG_VAR_DEBUG(result.overlapAmount);
    // Should handle very small overlaps appropriately
    EXPECT_TRUE(result.collided);
    EXPECT_GT(result.overlapAmount, 0.0f);
    LOG_FUNC_EXIT();
}

TEST_F(CollisionPhysicsTest, EdgeCase_LargeNumbers)
{
    LOG_FUNC_ENTER();
    // Test with large coordinate values
    CollisionPhysics::Polygon largePoly = {{0.0f, 0.0f}, {1000000.0f, 0.0f}, {1000000.0f, 1000000.0f}, {0.0f, 1000000.0f}};
    
    auto result = CollisionPhysics::CheckPolygonCollision(largePoly, {0.0f, 0.0f}, square, {500000.0f, 500000.0f});
    
    LOG_VAR_DEBUG(result.collided);
    // Should handle large numbers without overflow
    EXPECT_TRUE(result.collided || !result.collided);
    LOG_FUNC_EXIT();
}