#include <gtest/gtest.h>
#include "nyon/ecs/components/ColliderComponent.h"
#include "TestHelpers.h"
#include <cmath>

using namespace Nyon::ECS;
using namespace Nyon::Math;

/**
 * @brief Unit tests for ColliderComponent functionality.
 * 
 * Tests shape calculations, AABB bounds, polygon operations, circle collision,
 * and real-world collision scenarios for the collider component system.
 */
class ColliderComponentTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        LOG_FUNC_ENTER();
        // Initialize common test shapes
        squarePolygon = {
            {0.0f, 0.0f},
            {32.0f, 0.0f},
            {32.0f, 32.0f},
            {0.0f, 32.0f}
        };
        
        trianglePolygon = {
            {0.0f, 0.0f},
            {32.0f, 0.0f},
            {16.0f, 32.0f}
        };
        
        rectanglePolygon = {
            {0.0f, 0.0f},
            {64.0f, 0.0f},
            {64.0f, 16.0f},
            {0.0f, 16.0f}
        };
        LOG_FUNC_EXIT();
    }

    ColliderComponent::PolygonShape squarePolygon;
    ColliderComponent::PolygonShape trianglePolygon;
    ColliderComponent::PolygonShape rectanglePolygon;
};

// Constructor tests
TEST_F(ColliderComponentTest, Constructor_Default)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider;
    
    LOG_VAR_DEBUG(static_cast<int>(collider.type));
    LOG_VAR_DEBUG(collider.color.x);
    LOG_VAR_DEBUG(collider.color.y);
    LOG_VAR_DEBUG(collider.color.z);
    
    EXPECT_EQ(collider.type, ColliderComponent::ShapeType::Polygon);
    EXPECT_VECTOR3_NEAR(collider.color, Vector3(1.0f, 1.0f, 1.0f), 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, Constructor_PolygonShape)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(squarePolygon);
    
    LOG_VAR_DEBUG(static_cast<int>(collider.type));
    LOG_VAR_DEBUG(collider.GetPolygon().size());
    
    EXPECT_EQ(collider.type, ColliderComponent::ShapeType::Polygon);
    EXPECT_EQ(collider.GetPolygon().size(), squarePolygon.size());
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, Constructor_CircleShape)
{
    LOG_FUNC_ENTER();
    ColliderComponent::CircleShape circle{{16.0f, 16.0f}, 20.0f};
    ColliderComponent collider(circle);
    
    LOG_VAR_DEBUG(static_cast<int>(collider.type));
    LOG_VAR_DEBUG(collider.GetCircle().center.x);
    LOG_VAR_DEBUG(collider.GetCircle().center.y);
    LOG_VAR_DEBUG(collider.GetCircle().radius);
    
    EXPECT_EQ(collider.type, ColliderComponent::ShapeType::Circle);
    EXPECT_VECTOR2_NEAR(collider.GetCircle().center, Vector2(16.0f, 16.0f), 1e-6f);
    EXPECT_FLOAT_NEAR(collider.GetCircle().radius, 20.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

// Shape type tests
TEST_F(ColliderComponentTest, GetType_Polygon)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(squarePolygon);
    
    auto type = collider.GetType();
    LOG_VAR_DEBUG(static_cast<int>(type));
    
    EXPECT_EQ(type, ColliderComponent::ShapeType::Polygon);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, GetType_Circle)
{
    LOG_FUNC_ENTER();
    ColliderComponent::CircleShape circle{{0.0f, 0.0f}, 15.0f};
    ColliderComponent collider(circle);
    
    auto type = collider.GetType();
    LOG_VAR_DEBUG(static_cast<int>(type));
    
    EXPECT_EQ(type, ColliderComponent::ShapeType::Circle);
    LOG_FUNC_EXIT();
}

// Polygon shape tests
TEST_F(ColliderComponentTest, GetPolygon_Valid)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(squarePolygon);
    
    const auto& polygon = collider.GetPolygon();
    
    LOG_VAR_DEBUG(polygon.size());
    LOG_VAR_DEBUG(polygon[0].x);
    LOG_VAR_DEBUG(polygon[0].y);
    
    EXPECT_EQ(polygon.size(), squarePolygon.size());
    EXPECT_VECTOR2_NEAR(polygon[0], squarePolygon[0], 1e-6f);
    EXPECT_VECTOR2_NEAR(polygon[2], squarePolygon[2], 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, GetPolygon_Empty)
{
    LOG_FUNC_ENTER();
    ColliderComponent::PolygonShape emptyPolygon;
    ColliderComponent collider(emptyPolygon);
    
    const auto& polygon = collider.GetPolygon();
    
    LOG_VAR_DEBUG(polygon.size());
    EXPECT_TRUE(polygon.empty());
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, GetPolygon_DifferentShapes)
{
    LOG_FUNC_ENTER();
    // Test with different polygon shapes
    std::vector<ColliderComponent::PolygonShape> testPolygons = {
        squarePolygon,
        trianglePolygon,
        rectanglePolygon,
        {{0.0f, 0.0f}}, // Single point
        {} // Empty
    };
    
    for (size_t i = 0; i < testPolygons.size(); ++i) {
        ColliderComponent collider(testPolygons[i]);
        const auto& polygon = collider.GetPolygon();
        
        LOG_VAR_DEBUG(i);
        LOG_VAR_DEBUG(polygon.size());
        EXPECT_EQ(polygon.size(), testPolygons[i].size());
    }
    LOG_FUNC_EXIT();
}

// Circle shape tests
TEST_F(ColliderComponentTest, GetCircle_Valid)
{
    LOG_FUNC_ENTER();
    ColliderComponent::CircleShape circle{{50.0f, 75.0f}, 25.0f};
    ColliderComponent collider(circle);
    
    const auto& retrievedCircle = collider.GetCircle();
    
    LOG_VAR_DEBUG(retrievedCircle.center.x);
    LOG_VAR_DEBUG(retrievedCircle.center.y);
    LOG_VAR_DEBUG(retrievedCircle.radius);
    
    EXPECT_VECTOR2_NEAR(retrievedCircle.center, Vector2(50.0f, 75.0f), 1e-6f);
    EXPECT_FLOAT_NEAR(retrievedCircle.radius, 25.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, GetCircle_DifferentParameters)
{
    LOG_FUNC_ENTER();
    std::vector<std::pair<Vector2, float>> circleParams = {
        {{0.0f, 0.0f}, 10.0f},
        {{100.0f, 200.0f}, 50.0f},
        {{-25.0f, -75.0f}, 5.0f},
        {{0.0f, 0.0f}, 0.0f}, // Zero radius
        {{1000.0f, 2000.0f}, 1000.0f} // Large radius
    };
    
    for (size_t i = 0; i < circleParams.size(); ++i) {
        ColliderComponent::CircleShape circle{circleParams[i].first, circleParams[i].second};
        ColliderComponent collider(circle);
        
        const auto& retrievedCircle = collider.GetCircle();
        
        LOG_VAR_DEBUG(i);
        LOG_VAR_DEBUG(retrievedCircle.center.x);
        LOG_VAR_DEBUG(retrievedCircle.center.y);
        LOG_VAR_DEBUG(retrievedCircle.radius);
        
        EXPECT_VECTOR2_NEAR(retrievedCircle.center, circleParams[i].first, 1e-6f);
        EXPECT_FLOAT_NEAR(retrievedCircle.radius, circleParams[i].second, 1e-6f);
    }
    LOG_FUNC_EXIT();
}

// AABB calculation tests
TEST_F(ColliderComponentTest, CalculateAABB_Polygon_Square)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(squarePolygon);
    Vector2 position(100.0f, 200.0f);
    Vector2 min, max;
    
    collider.CalculateAABB(position, min, max);
    
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

TEST_F(ColliderComponentTest, CalculateAABB_Polygon_Triangle)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(trianglePolygon);
    Vector2 position(50.0f, 100.0f);
    Vector2 min, max;
    
    collider.CalculateAABB(position, min, max);
    
    LOG_VAR_DEBUG(min.x);
    LOG_VAR_DEBUG(min.y);
    LOG_VAR_DEBUG(max.x);
    LOG_VAR_DEBUG(max.y);
    
    // Triangle from (0,0) to (32,0) to (16,32) offset by position
    EXPECT_FLOAT_NEAR(min.x, 50.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(min.y, 100.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.x, 82.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.y, 132.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, CalculateAABB_Polygon_Empty)
{
    LOG_FUNC_ENTER();
    ColliderComponent::PolygonShape emptyPolygon;
    ColliderComponent collider(emptyPolygon);
    Vector2 position(100.0f, 200.0f);
    Vector2 min, max;
    
    collider.CalculateAABB(position, min, max);
    
    LOG_VAR_DEBUG(min.x);
    LOG_VAR_DEBUG(min.y);
    LOG_VAR_DEBUG(max.x);
    LOG_VAR_DEBUG(max.y);
    
    // Empty polygon should return position as both min and max
    EXPECT_VECTOR2_NEAR(min, position, 1e-6f);
    EXPECT_VECTOR2_NEAR(max, position, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, CalculateAABB_Circle)
{
    LOG_FUNC_ENTER();
    ColliderComponent::CircleShape circle{{16.0f, 16.0f}, 20.0f};
    ColliderComponent collider(circle);
    Vector2 position(100.0f, 200.0f);
    Vector2 min, max;
    
    collider.CalculateAABB(position, min, max);
    
    LOG_VAR_DEBUG(min.x);
    LOG_VAR_DEBUG(min.y);
    LOG_VAR_DEBUG(max.x);
    LOG_VAR_DEBUG(max.y);
    
    Vector2 worldCenter = circle.center + position;
    EXPECT_FLOAT_NEAR(min.x, worldCenter.x - circle.radius, 1e-6f);
    EXPECT_FLOAT_NEAR(min.y, worldCenter.y - circle.radius, 1e-6f);
    EXPECT_FLOAT_NEAR(max.x, worldCenter.x + circle.radius, 1e-6f);
    EXPECT_FLOAT_NEAR(max.y, worldCenter.y + circle.radius, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, CalculateAABB_Circle_ZeroRadius)
{
    LOG_FUNC_ENTER();
    ColliderComponent::CircleShape circle{{16.0f, 16.0f}, 0.0f};
    ColliderComponent collider(circle);
    Vector2 position(100.0f, 200.0f);
    Vector2 min, max;
    
    collider.CalculateAABB(position, min, max);
    
    LOG_VAR_DEBUG(min.x);
    LOG_VAR_DEBUG(min.y);
    LOG_VAR_DEBUG(max.x);
    LOG_VAR_DEBUG(max.y);
    
    Vector2 worldCenter = circle.center + position;
    EXPECT_VECTOR2_NEAR(min, worldCenter, 1e-6f);
    EXPECT_VECTOR2_NEAR(max, worldCenter, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, CalculateAABB_DifferentPositions)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(squarePolygon);
    
    std::vector<Vector2> positions = {
        {0.0f, 0.0f},
        {100.0f, 200.0f},
        {-50.0f, -100.0f},
        {1000.0f, 2000.0f}
    };
    
    for (size_t i = 0; i < positions.size(); ++i) {
        Vector2 min, max;
        collider.CalculateAABB(positions[i], min, max);
        
        LOG_VAR_DEBUG(i);
        LOG_VAR_DEBUG(positions[i].x);
        LOG_VAR_DEBUG(positions[i].y);
        LOG_VAR_DEBUG(min.x);
        LOG_VAR_DEBUG(min.y);
        LOG_VAR_DEBUG(max.x);
        LOG_VAR_DEBUG(max.y);
        
        // Min should equal position, max should equal position + size
        EXPECT_VECTOR2_NEAR(min, positions[i], 1e-6f);
        EXPECT_VECTOR2_NEAR(max, positions[i] + Vector2(32.0f, 32.0f), 1e-6f);
    }
    LOG_FUNC_EXIT();
}

// Bounds alias method tests
TEST_F(ColliderComponentTest, GetBounds_AliasMethod)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(squarePolygon);
    Vector2 position(100.0f, 200.0f);
    Vector2 min1, max1;
    Vector2 min2, max2;
    
    // Test that GetBounds is an alias for CalculateAABB
    collider.CalculateAABB(position, min1, max1);
    collider.GetBounds(position, min2, max2);
    
    LOG_VAR_DEBUG(min1.x);
    LOG_VAR_DEBUG(min1.y);
    LOG_VAR_DEBUG(max1.x);
    LOG_VAR_DEBUG(max1.y);
    LOG_VAR_DEBUG(min2.x);
    LOG_VAR_DEBUG(min2.y);
    LOG_VAR_DEBUG(max2.x);
    LOG_VAR_DEBUG(max2.y);
    
    EXPECT_VECTOR2_NEAR(min1, min2, 1e-6f);
    EXPECT_VECTOR2_NEAR(max1, max2, 1e-6f);
    LOG_FUNC_EXIT();
}

// Color property tests
TEST_F(ColliderComponentTest, Color_SetAndGet)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(squarePolygon);
    
    Vector3 newColor(0.5f, 0.7f, 0.9f);
    collider.color = newColor;
    
    LOG_VAR_DEBUG(collider.color.x);
    LOG_VAR_DEBUG(collider.color.y);
    LOG_VAR_DEBUG(collider.color.z);
    
    EXPECT_VECTOR3_NEAR(collider.color, newColor, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, Color_DefaultValue)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider;
    
    LOG_VAR_DEBUG(collider.color.x);
    LOG_VAR_DEBUG(collider.color.y);
    LOG_VAR_DEBUG(collider.color.z);
    
    EXPECT_VECTOR3_NEAR(collider.color, Vector3(1.0f, 1.0f, 1.0f), 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, Color_DifferentValues)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider;
    
    std::vector<Vector3> testColors = {
        {1.0f, 0.0f, 0.0f}, // Red
        {0.0f, 1.0f, 0.0f}, // Green
        {0.0f, 0.0f, 1.0f}, // Blue
        {0.0f, 0.0f, 0.0f}, // Black
        {1.0f, 1.0f, 1.0f}, // White
        {0.5f, 0.5f, 0.5f}  // Gray
    };
    
    for (size_t i = 0; i < testColors.size(); ++i) {
        collider.color = testColors[i];
        
        LOG_VAR_DEBUG(i);
        LOG_VAR_DEBUG(collider.color.x);
        LOG_VAR_DEBUG(collider.color.y);
        LOG_VAR_DEBUG(collider.color.z);
        
        EXPECT_VECTOR3_NEAR(collider.color, testColors[i], 1e-6f);
    }
    LOG_FUNC_EXIT();
}

// Edge case tests
TEST_F(ColliderComponentTest, EdgeCase_VeryLargePolygon)
{
    LOG_FUNC_ENTER();
    // Create a polygon with very large coordinates
    ColliderComponent::PolygonShape largePolygon = {
        {0.0f, 0.0f},
        {1000000.0f, 0.0f},
        {1000000.0f, 1000000.0f},
        {0.0f, 1000000.0f}
    };
    
    ColliderComponent collider(largePolygon);
    Vector2 position(0.0f, 0.0f);
    Vector2 min, max;
    
    collider.CalculateAABB(position, min, max);
    
    LOG_VAR_DEBUG(min.x);
    LOG_VAR_DEBUG(min.y);
    LOG_VAR_DEBUG(max.x);
    LOG_VAR_DEBUG(max.y);
    
    EXPECT_FLOAT_NEAR(min.x, 0.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(min.y, 0.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.x, 1000000.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.y, 1000000.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, EdgeCase_VerySmallPolygon)
{
    LOG_FUNC_ENTER();
    // Create a polygon with very small coordinates
    ColliderComponent::PolygonShape tinyPolygon = {
        {0.0f, 0.0f},
        {0.0001f, 0.0f},
        {0.0001f, 0.0001f},
        {0.0f, 0.0001f}
    };
    
    ColliderComponent collider(tinyPolygon);
    Vector2 position(100.0f, 200.0f);
    Vector2 min, max;
    
    collider.CalculateAABB(position, min, max);
    
    LOG_VAR_DEBUG(min.x);
    LOG_VAR_DEBUG(min.y);
    LOG_VAR_DEBUG(max.x);
    LOG_VAR_DEBUG(max.y);
    
    EXPECT_FLOAT_NEAR(min.x, 100.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(min.y, 200.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.x, 100.0001f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.y, 200.0001f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, EdgeCase_SinglePointPolygon)
{
    LOG_FUNC_ENTER();
    ColliderComponent::PolygonShape pointPolygon = {{50.0f, 75.0f}};
    ColliderComponent collider(pointPolygon);
    Vector2 position(100.0f, 200.0f);
    Vector2 min, max;
    
    collider.CalculateAABB(position, min, max);
    
    LOG_VAR_DEBUG(min.x);
    LOG_VAR_DEBUG(min.y);
    LOG_VAR_DEBUG(max.x);
    LOG_VAR_DEBUG(max.y);
    
    Vector2 expectedPoint = pointPolygon[0] + position;
    EXPECT_VECTOR2_NEAR(min, expectedPoint, 1e-6f);
    EXPECT_VECTOR2_NEAR(max, expectedPoint, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, EdgeCase_NegativeCoordinates)
{
    LOG_FUNC_ENTER();
    ColliderComponent::PolygonShape negativePolygon = {
        {-32.0f, -32.0f},
        {0.0f, -32.0f},
        {0.0f, 0.0f},
        {-32.0f, 0.0f}
    };
    
    ColliderComponent collider(negativePolygon);
    Vector2 position(-100.0f, -200.0f);
    Vector2 min, max;
    
    collider.CalculateAABB(position, min, max);
    
    LOG_VAR_DEBUG(min.x);
    LOG_VAR_DEBUG(min.y);
    LOG_VAR_DEBUG(max.x);
    LOG_VAR_DEBUG(max.y);
    
    // Should handle negative coordinates correctly
    EXPECT_FLOAT_NEAR(min.x, -132.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(min.y, -232.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.x, -100.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.y, -200.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, EdgeCase_LargeCircle)
{
    LOG_FUNC_ENTER();
    ColliderComponent::CircleShape largeCircle{{0.0f, 0.0f}, 1000000.0f};
    ColliderComponent collider(largeCircle);
    Vector2 position(0.0f, 0.0f);
    Vector2 min, max;
    
    collider.CalculateAABB(position, min, max);
    
    LOG_VAR_DEBUG(min.x);
    LOG_VAR_DEBUG(min.y);
    LOG_VAR_DEBUG(max.x);
    LOG_VAR_DEBUG(max.y);
    
    EXPECT_FLOAT_NEAR(min.x, -1000000.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(min.y, -1000000.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.x, 1000000.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.y, 1000000.0f, 1e-6f);
    LOG_FUNC_EXIT();
}

// Performance tests
TEST_F(ColliderComponentTest, Performance_ManyAABBCalculations)
{
    LOG_FUNC_ENTER();
    NyonTest::PERF_TIMER("ColliderComponent_ManyAABBCalculations");
    
    ColliderComponent collider(squarePolygon);
    Vector2 min, max;
    
    const int calculationCount = 10000;
    Vector2 position(0.0f, 0.0f);
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < calculationCount; ++i) {
        position.x = static_cast<float>(i);
        position.y = static_cast<float>(i * 2);
        collider.CalculateAABB(position, min, max);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    LOG_VAR_DEBUG(duration.count());
    LOG_VAR_DEBUG(calculationCount);
    
    EXPECT_LT(duration.count(), 1000); // Should complete in less than 1 second
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, Performance_ComplexPolygon)
{
    LOG_FUNC_ENTER();
    NyonTest::PERF_TIMER("ColliderComponent_ComplexPolygon");
    
    // Create a complex polygon with many vertices
    ColliderComponent::PolygonShape complexPolygon;
    const int vertexCount = 100;
    for (int i = 0; i < vertexCount; ++i) {
        float angle = 2.0f * M_PI * i / vertexCount;
        complexPolygon.push_back({
            50.0f + 40.0f * std::cos(angle),
            50.0f + 40.0f * std::sin(angle)
        });
    }
    
    ColliderComponent collider(complexPolygon);
    Vector2 min, max;
    Vector2 position(100.0f, 200.0f);
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < 1000; ++i) {
        collider.CalculateAABB(position, min, max);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    LOG_VAR_DEBUG(duration.count());
    
    EXPECT_LT(duration.count(), 1000);
    LOG_FUNC_EXIT();
}

// Integration scenario tests
TEST_F(ColliderComponentTest, GamingScenario_PlayerCollider)
{
    LOG_FUNC_ENTER();
    // Simulate a player collider
    ColliderComponent::PolygonShape playerShape = {
        {0.0f, 0.0f},
        {32.0f, 0.0f},
        {32.0f, 64.0f},
        {0.0f, 64.0f}
    };
    
    ColliderComponent playerCollider(playerShape);
    playerCollider.color = {0.0f, 1.0f, 1.0f}; // Cyan for player
    
    Vector2 playerPosition(100.0f, 150.0f);
    Vector2 min, max;
    
    playerCollider.CalculateAABB(playerPosition, min, max);
    
    LOG_VAR_DEBUG(min.x);
    LOG_VAR_DEBUG(min.y);
    LOG_VAR_DEBUG(max.x);
    LOG_VAR_DEBUG(max.y);
    LOG_VAR_DEBUG(playerCollider.color.x);
    LOG_VAR_DEBUG(playerCollider.color.y);
    LOG_VAR_DEBUG(playerCollider.color.z);
    
    // Player bounds should be correct
    EXPECT_FLOAT_NEAR(min.x, 100.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(min.y, 150.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.x, 132.0f, 1e-6f);
    EXPECT_FLOAT_NEAR(max.y, 214.0f, 1e-6f);
    EXPECT_VECTOR3_NEAR(playerCollider.color, Vector3(0.0f, 1.0f, 1.0f), 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, GamingScenario_EnemyCollider)
{
    LOG_FUNC_ENTER();
    // Simulate an enemy collider (circle)
    ColliderComponent::CircleShape enemyShape{{16.0f, 16.0f}, 24.0f};
    ColliderComponent enemyCollider(enemyShape);
    enemyCollider.color = {1.0f, 0.0f, 0.0f}; // Red for enemy
    
    Vector2 enemyPosition(200.0f, 100.0f);
    Vector2 min, max;
    
    enemyCollider.CalculateAABB(enemyPosition, min, max);
    
    LOG_VAR_DEBUG(min.x);
    LOG_VAR_DEBUG(min.y);
    LOG_VAR_DEBUG(max.x);
    LOG_VAR_DEBUG(max.y);
    
    Vector2 worldCenter = enemyShape.center + enemyPosition;
    EXPECT_FLOAT_NEAR(min.x, worldCenter.x - enemyShape.radius, 1e-6f);
    EXPECT_FLOAT_NEAR(min.y, worldCenter.y - enemyShape.radius, 1e-6f);
    EXPECT_FLOAT_NEAR(max.x, worldCenter.x + enemyShape.radius, 1e-6f);
    EXPECT_FLOAT_NEAR(max.y, worldCenter.y + enemyShape.radius, 1e-6f);
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, GamingScenario_ProjectileCollider)
{
    LOG_FUNC_ENTER();
    // Simulate a small projectile collider
    ColliderComponent::CircleShape projectileShape{{4.0f, 4.0f}, 4.0f};
    ColliderComponent projectileCollider(projectileShape);
    projectileCollider.color = {1.0f, 1.0f, 0.0f}; // Yellow for projectile
    
    std::vector<Vector2> positions = {
        {100.0f, 200.0f},
        {150.0f, 250.0f},
        {300.0f, 100.0f}
    };
    
    for (size_t i = 0; i < positions.size(); ++i) {
        Vector2 min, max;
        projectileCollider.CalculateAABB(positions[i], min, max);
        
        LOG_VAR_DEBUG(i);
        LOG_VAR_DEBUG(min.x);
        LOG_VAR_DEBUG(min.y);
        LOG_VAR_DEBUG(max.x);
        LOG_VAR_DEBUG(max.y);
        
        Vector2 worldCenter = projectileShape.center + positions[i];
        EXPECT_FLOAT_NEAR(min.x, worldCenter.x - projectileShape.radius, 1e-6f);
        EXPECT_FLOAT_NEAR(min.y, worldCenter.y - projectileShape.radius, 1e-6f);
        EXPECT_FLOAT_NEAR(max.x, worldCenter.x + projectileShape.radius, 1e-6f);
        EXPECT_FLOAT_NEAR(max.y, worldCenter.y + projectileShape.radius, 1e-6f);
    }
    LOG_FUNC_EXIT();
}

TEST_F(ColliderComponentTest, GamingScenario_LevelGeometry)
{
    LOG_FUNC_ENTER();
    // Simulate level geometry colliders
    std::vector<ColliderComponent> levelColliders;
    
    // Floor
    ColliderComponent::PolygonShape floorShape = {
        {0.0f, 0.0f},
        {1280.0f, 0.0f},
        {1280.0f, 32.0f},
        {0.0f, 32.0f}
    };
    levelColliders.emplace_back(floorShape);
    levelColliders.back().color = {0.5f, 0.5f, 0.5f};
    
    // Wall
    ColliderComponent::PolygonShape wallShape = {
        {0.0f, 0.0f},
        {32.0f, 0.0f},
        {32.0f, 720.0f},
        {0.0f, 720.0f}
    };
    levelColliders.emplace_back(wallShape);
    levelColliders.back().color = {0.7f, 0.7f, 0.7f};
    
    // Platform
    ColliderComponent::PolygonShape platformShape = {
        {400.0f, 400.0f},
        {600.0f, 400.0f},
        {600.0f, 432.0f},
        {400.0f, 432.0f}
    };
    levelColliders.emplace_back(platformShape);
    levelColliders.back().color = {0.8f, 0.6f, 0.4f};
    
    // Test all level colliders
    Vector2 min, max;
    for (size_t i = 0; i < levelColliders.size(); ++i) {
        levelColliders[i].CalculateAABB({0.0f, 0.0f}, min, max);
        
        LOG_VAR_DEBUG(i);
        LOG_VAR_DEBUG(min.x);
        LOG_VAR_DEBUG(min.y);
        LOG_VAR_DEBUG(max.x);
        LOG_VAR_DEBUG(max.y);
        
        // Just verify they calculate without error
        EXPECT_TRUE(min.x <= max.x);
        EXPECT_TRUE(min.y <= max.y);
    }
    LOG_FUNC_EXIT();
}

// Backward compatibility tests
TEST_F(ColliderComponentTest, BackwardCompatibility_GetBounds)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(squarePolygon);
    Vector2 position(50.0f, 100.0f);
    Vector2 min1, max1;
    Vector2 min2, max2;
    
    // Test that both methods give same results
    collider.CalculateAABB(position, min1, max1);
    collider.GetBounds(position, min2, max2);
    
    LOG_VAR_DEBUG(min1.x);
    LOG_VAR_DEBUG(min1.y);
    LOG_VAR_DEBUG(max1.x);
    LOG_VAR_DEBUG(max1.y);
    LOG_VAR_DEBUG(min2.x);
    LOG_VAR_DEBUG(min2.y);
    LOG_VAR_DEBUG(max2.x);
    LOG_VAR_DEBUG(max2.y);
    
    EXPECT_VECTOR2_NEAR(min1, min2, 1e-6f);
    EXPECT_VECTOR2_NEAR(max1, max2, 1e-6f);
    LOG_FUNC_EXIT();
}