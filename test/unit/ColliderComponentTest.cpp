#include <gtest/gtest.h>
#include "TestHelpers.h"
#include "nyon/ecs/components/ColliderComponent.h"

using namespace Nyon::ECS;

/**
 * @brief Comprehensive unit tests for ColliderComponent.
 * 
 * Tests cover all core functionalities including:
 * - Shape construction and initialization
 * - Polygon properties (winding order, centroid, normals)
 * - Circle, capsule, and segment shapes
 * - Chain and composite shapes
 * - Filtering system
 * - Sensor system
 * - Material properties
 * - AABB calculations
 * - Area calculations
 * - Inertia calculations
 * - Mass calculations
 */

// ============================================================================
// CONSTRUCTION AND INITIALIZATION TESTS
// ============================================================================

TEST(ColliderComponentTest, DefaultConstructor)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider;
    
    EXPECT_EQ(collider.type, ColliderComponent::ShapeType::Polygon);
    EXPECT_FLOAT_NEAR(collider.color.x, 1.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(collider.color.y, 1.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(collider.color.z, 1.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(collider.density, 1.0f, 1e-5f);
    EXPECT_FALSE(collider.isSensor);
    EXPECT_TRUE(collider.enableContactEvents);
    EXPECT_TRUE(collider.enableSensorEvents);
    EXPECT_FALSE(collider.enableHitEvents);
    EXPECT_EQ(collider.proxyId, -1);
    EXPECT_FALSE(collider.forceUpdate);
    
    // Check default filter
    EXPECT_EQ(collider.filter.categoryBits, 0x0001);
    EXPECT_EQ(collider.filter.maskBits, 0xFFFF);
    EXPECT_EQ(collider.filter.groupIndex, 0);
    
    // Check default material
    EXPECT_FLOAT_NEAR(collider.material.friction, 0.2f, 1e-5f);
    EXPECT_FLOAT_NEAR(collider.material.restitution, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(collider.material.density, 1.0f, 1e-5f);
    EXPECT_EQ(collider.material.userData, 0);
    EXPECT_EQ(collider.material.name, "default");
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, ConstructorWithRadius)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(25.0f);
    
    EXPECT_EQ(collider.type, ColliderComponent::ShapeType::Circle);
    auto& circle = collider.GetCircle();
    EXPECT_FLOAT_NEAR(circle.radius, 25.0f, 1e-5f);
    EXPECT_VECTOR2_NEAR(circle.center, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, ConstructorWithPolygon)
{
    LOG_FUNC_ENTER();
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 0),
        Nyon::Math::Vector2(32, 0),
        Nyon::Math::Vector2(32, 32),
        Nyon::Math::Vector2(0, 32)
    };
    
    ColliderComponent::PolygonShape poly(vertices);
    ColliderComponent collider(poly);
    
    EXPECT_EQ(collider.type, ColliderComponent::ShapeType::Polygon);
    auto& polygon = collider.GetPolygon();
    EXPECT_EQ(polygon.vertices.size(), 4);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, ConstructorWithCircle)
{
    LOG_FUNC_ENTER();
    ColliderComponent::CircleShape circle{{10.0f, 20.0f}, 15.0f};
    ColliderComponent collider(circle);
    
    EXPECT_EQ(collider.type, ColliderComponent::ShapeType::Circle);
    auto& retrievedCircle = collider.GetCircle();
    EXPECT_VECTOR2_NEAR(retrievedCircle.center, Nyon::Math::Vector2(10.0f, 20.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(retrievedCircle.radius, 15.0f, 1e-5f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, ConstructorWithCapsule)
{
    LOG_FUNC_ENTER();
    ColliderComponent::CapsuleShape capsule{{0.0f, 0.0f}, {0.0f, 50.0f}, 10.0f};
    ColliderComponent collider(capsule);
    
    EXPECT_EQ(collider.type, ColliderComponent::ShapeType::Capsule);
    auto& retrievedCapsule = collider.GetCapsule();
    EXPECT_VECTOR2_NEAR(retrievedCapsule.center1, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_VECTOR2_NEAR(retrievedCapsule.center2, Nyon::Math::Vector2(0.0f, 50.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(retrievedCapsule.radius, 10.0f, 1e-5f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, ConstructorWithSegment)
{
    LOG_FUNC_ENTER();
    ColliderComponent::SegmentShape segment{{0.0f, 0.0f}, {100.0f, 50.0f}, 5.0f};
    ColliderComponent collider(segment);
    
    EXPECT_EQ(collider.type, ColliderComponent::ShapeType::Segment);
    auto& retrievedSegment = collider.GetSegment();
    EXPECT_VECTOR2_NEAR(retrievedSegment.point1, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    EXPECT_VECTOR2_NEAR(retrievedSegment.point2, Nyon::Math::Vector2(100.0f, 50.0f), 1e-5f);
    EXPECT_FLOAT_NEAR(retrievedSegment.radius, 5.0f, 1e-5f);
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// POLYGON SHAPE TESTS
// ============================================================================

TEST(ColliderComponentTest, PolygonWindingOrderClockwise)
{
    LOG_FUNC_ENTER();
    // Clockwise winding (should be reversed)
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 0),
        Nyon::Math::Vector2(0, 32),
        Nyon::Math::Vector2(32, 32),
        Nyon::Math::Vector2(32, 0)
    };
    
    ColliderComponent::PolygonShape polygon(vertices);
    
    // Should have been reversed to counter-clockwise
    EXPECT_TRUE(polygon.IsCounterClockwise());
    EXPECT_EQ(polygon.vertices.size(), 4);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, PolygonWindingOrderCounterClockwise)
{
    LOG_FUNC_ENTER();
    // Counter-clockwise winding (should stay as is)
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 0),
        Nyon::Math::Vector2(32, 0),
        Nyon::Math::Vector2(32, 32),
        Nyon::Math::Vector2(0, 32)
    };
    
    ColliderComponent::PolygonShape polygon(vertices);
    
    EXPECT_TRUE(polygon.IsCounterClockwise());
    EXPECT_EQ(polygon.vertices.size(), 4);
    // First vertex should still be (0,0)
    EXPECT_VECTOR2_NEAR(polygon.vertices[0], Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, PolygonCentroidCalculation)
{
    LOG_FUNC_ENTER();
    // Square centered at origin
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(-16, -16),
        Nyon::Math::Vector2(16, -16),
        Nyon::Math::Vector2(16, 16),
        Nyon::Math::Vector2(-16, 16)
    };
    
    ColliderComponent::PolygonShape polygon(vertices);
    
    // Centroid should be at origin for symmetric square
    EXPECT_VECTOR2_NEAR(polygon.centroid, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, PolygonCentroidOffCenter)
{
    LOG_FUNC_ENTER();
    // Rectangle offset from origin
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 0),
        Nyon::Math::Vector2(32, 0),
        Nyon::Math::Vector2(32, 16),
        Nyon::Math::Vector2(0, 16)
    };
    
    ColliderComponent::PolygonShape polygon(vertices);
    
    // Centroid should be at center of rectangle (16, 8)
    EXPECT_VECTOR2_NEAR(polygon.centroid, Nyon::Math::Vector2(16.0f, 8.0f), 1.0f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, PolygonNormalsCalculation)
{
    LOG_FUNC_ENTER();
    // Unit square
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 0),
        Nyon::Math::Vector2(1, 0),
        Nyon::Math::Vector2(1, 1),
        Nyon::Math::Vector2(0, 1)
    };
    
    ColliderComponent::PolygonShape polygon(vertices);
    
    // Should have 4 normals (one per edge)
    EXPECT_EQ(polygon.normals.size(), 4);
    
    // Normals should be normalized
    for (const auto& normal : polygon.normals) {
        float length = sqrt(normal.x * normal.x + normal.y * normal.y);
        EXPECT_FLOAT_NEAR(length, 1.0f, 1e-5f);
    }
    
    // First edge (0,0) to (1,0) should have normal pointing up (0, 1) for CCW polygon
    EXPECT_VECTOR2_NEAR(polygon.normals[0], Nyon::Math::Vector2(0.0f, 1.0f), 1e-5f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, PolygonDegenerateCase)
{
    LOG_FUNC_ENTER();
    // Less than 3 vertices
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 0),
        Nyon::Math::Vector2(1, 0)
    };
    
    ColliderComponent::PolygonShape polygon(vertices);
    
    EXPECT_EQ(polygon.vertices.size(), 2);
    // Line segment still has edge normals (2 edges for 2 points)
    EXPECT_EQ(polygon.normals.size(), 2);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, PolygonEmptyVertices)
{
    LOG_FUNC_ENTER();
    std::vector<Nyon::Math::Vector2> vertices;
    
    ColliderComponent::PolygonShape polygon(vertices);
    
    EXPECT_TRUE(polygon.vertices.empty());
    EXPECT_VECTOR2_NEAR(polygon.centroid, Nyon::Math::Vector2(0.0f, 0.0f), 1e-5f);
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// FILTERING SYSTEM TESTS
// ============================================================================

TEST(ColliderComponentTest, FilterDefaultValues)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider;
    
    EXPECT_EQ(collider.filter.categoryBits, 0x0001);
    EXPECT_EQ(collider.filter.maskBits, 0xFFFF);
    EXPECT_EQ(collider.filter.groupIndex, 0);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, FilterSamePositiveGroup)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider1, collider2;
    collider1.filter.groupIndex = 5;
    collider2.filter.groupIndex = 5;
    
    EXPECT_TRUE(collider1.filter.ShouldCollide(collider2.filter));
    EXPECT_TRUE(collider2.filter.ShouldCollide(collider1.filter));
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, FilterSameNegativeGroup)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider1, collider2;
    collider1.filter.groupIndex = -5;
    collider2.filter.groupIndex = -5;
    
    EXPECT_FALSE(collider1.filter.ShouldCollide(collider2.filter));
    EXPECT_FALSE(collider2.filter.ShouldCollide(collider1.filter));
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, FilterDifferentGroups)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider1, collider2;
    collider1.filter.groupIndex = 5;
    collider2.filter.groupIndex = 10;
    
    // Different groups fall back to category/mask bits
    EXPECT_TRUE(collider1.filter.ShouldCollide(collider2.filter));
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, FilterCategoryMaskCollision)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider1, collider2;
    
    // Collider1 is in category 2, collides with category 4
    collider1.filter.categoryBits = 0x0002;
    collider1.filter.maskBits = 0x0004;
    
    // Collider2 is in category 4, collides with category 2
    collider2.filter.categoryBits = 0x0004;
    collider2.filter.maskBits = 0x0002;
    
    EXPECT_TRUE(collider1.filter.ShouldCollide(collider2.filter));
    EXPECT_TRUE(collider2.filter.ShouldCollide(collider1.filter));
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, FilterCategoryMaskNoCollision)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider1, collider2;
    
    // Collider1 is in category 2, only collides with category 2
    collider1.filter.categoryBits = 0x0002;
    collider1.filter.maskBits = 0x0002;
    
    // Collider2 is in category 4, only collides with category 4
    collider2.filter.categoryBits = 0x0004;
    collider2.filter.maskBits = 0x0004;
    
    EXPECT_FALSE(collider1.filter.ShouldCollide(collider2.filter));
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, FilterOneWayCollision)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider1, collider2;
    
    // In this filtering system, BOTH colliders must agree to collide
    // This test demonstrates successful collision when both agree
    
    // Scenario: Player vs Enemy (mutual collision)
    collider1.filter.categoryBits = 0x0001;  // Player category
    collider1.filter.maskBits = 0xFFFF;      // Can hit everything
    
    collider2.filter.categoryBits = 0x0002;  // Enemy category
    collider2.filter.maskBits = 0xFFFF;      // Can hit everything
    
    // Both should want to collide
    bool c1WantsCollision = (collider1.filter.categoryBits & collider2.filter.maskBits) != 0;
    bool c2WantsCollision = (collider2.filter.categoryBits & collider1.filter.maskBits) != 0;
    
    EXPECT_TRUE(c1WantsCollision);  // Player can hit enemy
    EXPECT_TRUE(c2WantsCollision);  // Enemy can hit player
    
    // Result: YES collision because BOTH agree
    EXPECT_TRUE(collider1.filter.ShouldCollide(collider2.filter));
    EXPECT_TRUE(collider2.filter.ShouldCollide(collider1.filter));
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// SENSOR SYSTEM TESTS
// ============================================================================

TEST(ColliderComponentTest, SensorDefaultState)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider;
    
    EXPECT_FALSE(collider.isSensor);
    EXPECT_TRUE(collider.enableContactEvents);
    EXPECT_TRUE(collider.enableSensorEvents);
    EXPECT_FALSE(collider.enableHitEvents);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, SetSensor)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider;
    
    collider.SetSensor(true);
    EXPECT_TRUE(collider.IsSensor());
    
    collider.SetSensor(false);
    EXPECT_FALSE(collider.IsSensor());
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, EnableDisableEvents)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider;
    
    collider.enableContactEvents = false;
    EXPECT_FALSE(collider.enableContactEvents);
    
    collider.enableSensorEvents = false;
    EXPECT_FALSE(collider.enableSensorEvents);
    
    collider.enableHitEvents = true;
    EXPECT_TRUE(collider.enableHitEvents);
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// MATERIAL PROPERTIES TESTS
// ============================================================================

TEST(ColliderComponentTest, MaterialDefaultValues)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider;
    
    EXPECT_FLOAT_NEAR(collider.material.friction, 0.2f, 1e-5f);
    EXPECT_FLOAT_NEAR(collider.material.restitution, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(collider.material.density, 1.0f, 1e-5f);
    EXPECT_EQ(collider.material.userData, 0);
    EXPECT_EQ(collider.material.name, "default");
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, MaterialCustomValues)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider;
    
    collider.material.friction = 0.8f;
    collider.material.restitution = 0.5f;
    collider.material.density = 2.5f;
    collider.material.userData = 12345;
    collider.material.name = "rubber";
    
    EXPECT_FLOAT_NEAR(collider.material.friction, 0.8f, 1e-5f);
    EXPECT_FLOAT_NEAR(collider.material.restitution, 0.5f, 1e-5f);
    EXPECT_FLOAT_NEAR(collider.material.density, 2.5f, 1e-5f);
    EXPECT_EQ(collider.material.userData, 12345);
    EXPECT_EQ(collider.material.name, "rubber");
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// AABB CALCULATION TESTS
// ============================================================================

TEST(ColliderComponentTest, CircleAABB)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(10.0f);
    
    Nyon::Math::Vector2 min, max;
    collider.CalculateAABB(Nyon::Math::Vector2(50.0f, 50.0f), 0.0f, min, max);
    
    // Circle at (50,50) with radius 10 (plus speculative distance 0.1)
    EXPECT_FLOAT_NEAR(min.x, 39.9f, 0.2f);
    EXPECT_FLOAT_NEAR(min.y, 39.9f, 0.2f);
    EXPECT_FLOAT_NEAR(max.x, 60.1f, 0.2f);
    EXPECT_FLOAT_NEAR(max.y, 60.1f, 0.2f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, CircleAABBWithOffset)
{
    LOG_FUNC_ENTER();
    ColliderComponent::CircleShape circle{{20.0f, 30.0f}, 15.0f};
    ColliderComponent collider(circle);
    
    Nyon::Math::Vector2 min, max;
    collider.CalculateAABB(Nyon::Math::Vector2(100.0f, 100.0f), 0.0f, min, max);
    
    // Circle center at (120, 130) with radius 15 (plus speculative distance 0.1)
    EXPECT_FLOAT_NEAR(min.x, 104.9f, 0.2f);
    EXPECT_FLOAT_NEAR(min.y, 114.9f, 0.2f);
    EXPECT_FLOAT_NEAR(max.x, 135.1f, 0.2f);
    EXPECT_FLOAT_NEAR(max.y, 145.1f, 0.2f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, SquareAABBNoRotation)
{
    LOG_FUNC_ENTER();
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 0),
        Nyon::Math::Vector2(32, 0),
        Nyon::Math::Vector2(32, 32),
        Nyon::Math::Vector2(0, 32)
    };
    
    ColliderComponent collider(vertices);
    
    Nyon::Math::Vector2 min, max;
    collider.CalculateAABB(Nyon::Math::Vector2(10.0f, 10.0f), 0.0f, min, max);
    
    // Square from (10,10) to (42,42) (plus speculative distance 0.1 on each side)
    EXPECT_FLOAT_NEAR(min.x, 9.9f, 0.2f);
    EXPECT_FLOAT_NEAR(min.y, 9.9f, 0.2f);
    EXPECT_FLOAT_NEAR(max.x, 42.1f, 0.2f);
    EXPECT_FLOAT_NEAR(max.y, 42.1f, 0.2f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, CapsuleAABB)
{
    LOG_FUNC_ENTER();
    ColliderComponent::CapsuleShape capsule{{0.0f, 0.0f}, {0.0f, 50.0f}, 10.0f};
    ColliderComponent collider(capsule);
    
    Nyon::Math::Vector2 min, max;
    collider.CalculateAABB(Nyon::Math::Vector2(100.0f, 100.0f), 0.0f, min, max);
    
    // Vertical capsule from (100,100) to (100,150) with radius 10
    EXPECT_FLOAT_NEAR(min.x, 90.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(min.y, 90.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(max.x, 110.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(max.y, 160.0f, 1e-5f);
    
    LOG_FUNC_EXIT();
}

TEST(CollisionComponentTest, SegmentAABB)
{
    LOG_FUNC_ENTER();
    ColliderComponent::SegmentShape segment{{0.0f, 0.0f}, {100.0f, 50.0f}, 5.0f};
    ColliderComponent collider(segment);
    
    Nyon::Math::Vector2 min, max;
    collider.CalculateAABB(Nyon::Math::Vector2(10.0f, 10.0f), 0.0f, min, max);
    
    // Segment from (10,10) to (110,60) with radius 5
    EXPECT_FLOAT_NEAR(min.x, 5.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(min.y, 5.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(max.x, 115.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(max.y, 65.0f, 1e-5f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, RotatedSquareAABB)
{
    LOG_FUNC_ENTER();
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(-16, -16),
        Nyon::Math::Vector2(16, -16),
        Nyon::Math::Vector2(16, 16),
        Nyon::Math::Vector2(-16, 16)
    };
    
    ColliderComponent collider(vertices);
    
    Nyon::Math::Vector2 min, max;
    // Rotate 45 degrees (π/4)
    collider.CalculateAABB(Nyon::Math::Vector2(0.0f, 0.0f), 3.14159f / 4.0f, min, max);
    
    // Rotated square should have larger AABB
    float expectedExtent = 16.0f * sqrt(2.0f);  // Half diagonal
    EXPECT_FLOAT_NEAR(min.x, -expectedExtent, 1.0f);
    EXPECT_FLOAT_NEAR(min.y, -expectedExtent, 1.0f);
    EXPECT_FLOAT_NEAR(max.x, expectedExtent, 1.0f);
    EXPECT_FLOAT_NEAR(max.y, expectedExtent, 1.0f);
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// AREA CALCULATION TESTS
// ============================================================================

TEST(ColliderComponentTest, CircleArea)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(10.0f);
    
    float area = collider.CalculateArea();
    float expectedArea = 3.14159f * 10.0f * 10.0f;
    
    EXPECT_FLOAT_NEAR(area, expectedArea, 1.0f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, SquareArea)
{
    LOG_FUNC_ENTER();
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 0),
        Nyon::Math::Vector2(32, 0),
        Nyon::Math::Vector2(32, 32),
        Nyon::Math::Vector2(0, 32)
    };
    
    ColliderComponent collider(vertices);
    
    float area = collider.CalculateArea();
    float expectedArea = 32.0f * 32.0f;
    
    EXPECT_FLOAT_NEAR(area, expectedArea, 1.0f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, RectangleArea)
{
    LOG_FUNC_ENTER();
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 0),
        Nyon::Math::Vector2(50, 0),
        Nyon::Math::Vector2(50, 20),
        Nyon::Math::Vector2(0, 20)
    };
    
    ColliderComponent collider(vertices);
    
    float area = collider.CalculateArea();
    float expectedArea = 50.0f * 20.0f;
    
    EXPECT_FLOAT_NEAR(area, expectedArea, 1.0f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, TriangleArea)
{
    LOG_FUNC_ENTER();
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 0),
        Nyon::Math::Vector2(30, 0),
        Nyon::Math::Vector2(15, 20)
    };
    
    ColliderComponent collider(vertices);
    
    float area = collider.CalculateArea();
    float expectedArea = 0.5f * 30.0f * 20.0f;  // base * height / 2
    
    EXPECT_FLOAT_NEAR(area, expectedArea, 1.0f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, CapsuleArea)
{
    LOG_FUNC_ENTER();
    ColliderComponent::CapsuleShape capsule{{0.0f, 0.0f}, {0.0f, 50.0f}, 10.0f};
    ColliderComponent collider(capsule);
    
    float area = collider.CalculateArea();
    
    // Rectangle area + two semicircles (one full circle)
    float rectArea = 50.0f * 20.0f;  // length * diameter
    float circleArea = 3.14159f * 10.0f * 10.0f;
    float expectedArea = rectArea + circleArea;
    
    EXPECT_FLOAT_NEAR(area, expectedArea, 10.0f);
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// INERTIA CALCULATION TESTS
// ============================================================================

TEST(ColliderComponentTest, CircleInertiaForUnitDensity)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(10.0f);
    
    float inertia = collider.CalculateInertiaForUnitDensity();
    float expectedInertia = 0.5f * 10.0f * 10.0f;  // r²/2
    
    EXPECT_FLOAT_NEAR(inertia, expectedInertia, 1.0f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, SquareInertiaForUnitDensity)
{
    LOG_FUNC_ENTER();
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(-16, -16),
        Nyon::Math::Vector2(16, -16),
        Nyon::Math::Vector2(16, 16),
        Nyon::Math::Vector2(-16, 16)
    };
    
    ColliderComponent collider(vertices);
    
    float inertia = collider.CalculateInertiaForUnitDensity();
    
    // For a square with side a: I ≈ a⁴/6 for unit density
    float expectedInertia = (32.0f * 32.0f * 32.0f * 32.0f) / 6.0f;
    
    EXPECT_FLOAT_NEAR(inertia, expectedInertia, 1000.0f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, InertiaSmallVsLargeCircle)
{
    LOG_FUNC_ENTER();
    ColliderComponent smallCircle(5.0f);
    ColliderComponent largeCircle(20.0f);
    
    float smallInertia = smallCircle.CalculateInertiaForUnitDensity();
    float largeInertia = largeCircle.CalculateInertiaForUnitDensity();
    
    // Larger circle should have much larger inertia (scales with r²)
    EXPECT_GT(largeInertia, smallInertia);
    EXPECT_GT(largeInertia / smallInertia, 10.0f);  // Should be roughly 16x
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// MASS CALCULATION TESTS
// ============================================================================

TEST(ColliderComponentTest, MassFromDensityAndArea)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(10.0f);
    
    collider.material.density = 2.0f;
    float mass = collider.CalculateMass();
    
    float area = collider.CalculateArea();
    float expectedMass = area * 2.0f;
    
    EXPECT_FLOAT_NEAR(mass, expectedMass, 1.0f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, MassWithDensityOverride)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(10.0f);
    
    collider.material.density = 1.0f;
    float mass = collider.CalculateMass(5.0f);  // Override density
    
    float area = collider.CalculateArea();
    float expectedMass = area * 5.0f;
    
    EXPECT_FLOAT_NEAR(mass, expectedMass, 1.0f);
    EXPECT_FLOAT_NEAR(collider.material.density, 1.0f, 1e-5f);  // Original unchanged
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, MassZeroDensity)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(10.0f);
    
    collider.material.density = 0.0f;
    float mass = collider.CalculateMass();
    
    EXPECT_FLOAT_NEAR(mass, 0.0f, 1e-5f);
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// COMPREHENSIVE SCENARIO TESTS
// ============================================================================

TEST(ColliderComponentTest, PlayerCharacterCollider)
{
    LOG_FUNC_ENTER();
    
    // Typical player character setup
    ColliderComponent collider;
    collider.type = ColliderComponent::ShapeType::Capsule;
    collider.shape = ColliderComponent::CapsuleShape{{0.0f, 0.0f}, {0.0f, 60.0f}, 15.0f};
    
    // Player-specific settings
    collider.filter.categoryBits = 0x0001;  // Player category
    collider.filter.maskBits = 0xFFFF;       // Collides with everything
    collider.filter.groupIndex = 0;
    
    collider.material.friction = 0.3f;
    collider.material.restitution = 0.0f;    // No bounce
    collider.material.density = 1.0f;
    
    collider.enableContactEvents = true;
    collider.enableSensorEvents = false;
    collider.enableHitEvents = true;         // Detect hits
    
    EXPECT_EQ(collider.type, ColliderComponent::ShapeType::Capsule);
    EXPECT_EQ(collider.filter.categoryBits, 0x0001);
    EXPECT_TRUE(collider.enableHitEvents);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, SensorTriggerZone)
{
    LOG_FUNC_ENTER();
    
    // Sensor trigger zone
    ColliderComponent collider;
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 0),
        Nyon::Math::Vector2(100, 0),
        Nyon::Math::Vector2(100, 100),
        Nyon::Math::Vector2(0, 100)
    };
    collider.shape = ColliderComponent::PolygonShape(vertices);
    
    // Sensor configuration
    collider.isSensor = true;
    collider.enableContactEvents = false;
    collider.enableSensorEvents = true;
    collider.enableHitEvents = false;
    
    // Friendly fire prevention (same team doesn't collide)
    collider.filter.groupIndex = 10;  // Positive = must collide within team
    
    EXPECT_TRUE(collider.IsSensor());
    EXPECT_TRUE(collider.enableSensorEvents);
    EXPECT_TRUE(collider.filter.ShouldCollide(collider.filter));  // Same team collides
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, EnemyCollider)
{
    LOG_FUNC_ENTER();
    
    ColliderComponent enemy;
    enemy.shape = ColliderComponent::CircleShape{{0.0f, 0.0f}, 20.0f};
    
    // Enemy-specific filtering
    enemy.filter.categoryBits = 0x0002;  // Enemy category
    enemy.filter.maskBits = 0xFFFD;      // Doesn't collide with enemies (bit 2 off)
    enemy.filter.groupIndex = -5;        // Negative = never collide with same group
    
    enemy.material.friction = 0.5f;
    enemy.material.restitution = 0.3f;   // Slight bounce
    
    ColliderComponent player;
    player.filter.categoryBits = 0x0001;
    player.filter.maskBits = 0xFFFF;
    
    // Enemy collides with player
    EXPECT_TRUE(enemy.filter.ShouldCollide(player.filter));
    
    // Two enemies don't collide
    ColliderComponent enemy2;
    enemy2.filter = enemy.filter;
    EXPECT_FALSE(enemy.filter.ShouldCollide(enemy2.filter));
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, ProjectileCollider)
{
    LOG_FUNC_ENTER();
    
    ColliderComponent projectile;
    projectile.shape = ColliderComponent::CircleShape{{0.0f, 0.0f}, 5.0f};
    
    // Projectile settings
    projectile.filter.categoryBits = 0x0004;  // Projectile category
    projectile.filter.maskBits = 0xFFFB;      // Doesn't collide with other projectiles
    
    projectile.isSensor = false;
    projectile.enableContactEvents = true;
    projectile.enableHitEvents = true;        // Important for damage calculation
    
    projectile.material.density = 10.0f;      // Heavy projectile
    projectile.material.restitution = 0.7f;   // Bouncy
    
    EXPECT_TRUE(projectile.enableHitEvents);
    EXPECT_FLOAT_NEAR(projectile.material.density, 10.0f, 1e-5f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, TerrainBoundaryCollider)
{
    LOG_FUNC_ENTER();
    
    // Static terrain/boundary
    ColliderComponent terrain;
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 0),
        Nyon::Math::Vector2(1000, 0),
        Nyon::Math::Vector2(1000, 50),
        Nyon::Math::Vector2(0, 50)
    };
    terrain.shape = ColliderComponent::PolygonShape(vertices);
    
    // Static object settings
    terrain.filter.categoryBits = 0x0008;  // Terrain category
    terrain.filter.maskBits = 0xFFFF;
    
    terrain.material.friction = 0.9f;      // High friction
    terrain.material.restitution = 0.0f;   // No bounce
    
    EXPECT_FLOAT_NEAR(terrain.material.friction, 0.9f, 1e-5f);
    EXPECT_FLOAT_NEAR(terrain.material.restitution, 0.0f, 1e-5f);
    
    LOG_FUNC_EXIT();
}

// ============================================================================
// EDGE CASES AND BOUNDARY CONDITIONS
// ============================================================================

TEST(ColliderComponentTest, VerySmallCircle)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(0.001f);
    
    float area = collider.CalculateArea();
    EXPECT_GT(area, 0.0f);
    EXPECT_LT(area, 1.0f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, VeryLargeCircle)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider(10000.0f);
    
    float area = collider.CalculateArea();
    EXPECT_GT(area, 1000000.0f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, IrregularPolygon)
{
    LOG_FUNC_ENTER();
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 0),
        Nyon::Math::Vector2(50, 0),
        Nyon::Math::Vector2(60, 30),
        Nyon::Math::Vector2(30, 50),
        Nyon::Math::Vector2(-10, 30)
    };
    
    ColliderComponent collider(vertices);
    
    float area = collider.CalculateArea();
    EXPECT_GT(area, 0.0f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, ConcaveLikePolygon)
{
    LOG_FUNC_ENTER();
    // Star-like shape (non-convex)
    std::vector<Nyon::Math::Vector2> vertices = {
        Nyon::Math::Vector2(0, 50),
        Nyon::Math::Vector2(20, 20),
        Nyon::Math::Vector2(50, 0),
        Nyon::Math::Vector2(20, -20),
        Nyon::Math::Vector2(0, -50),
        Nyon::Math::Vector2(-20, -20),
        Nyon::Math::Vector2(-50, 0),
        Nyon::Math::Vector2(-20, 20)
    };
    
    ColliderComponent collider(vertices);
    
    // Should still calculate area even if concave
    float area = collider.CalculateArea();
    EXPECT_GT(area, 0.0f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, ZeroRadiusSegment)
{
    LOG_FUNC_ENTER();
    ColliderComponent::SegmentShape segment{{0.0f, 0.0f}, {100.0f, 0.0f}, 0.0f};
    ColliderComponent collider(segment);
    
    Nyon::Math::Vector2 min, max;
    collider.CalculateAABB(Nyon::Math::Vector2(0.0f, 0.0f), 0.0f, min, max);
    
    // Line has no thickness
    EXPECT_FLOAT_NEAR(min.y, 0.0f, 1e-5f);
    EXPECT_FLOAT_NEAR(max.y, 0.0f, 1e-5f);
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, NegativeFilterGroupIndex)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider1, collider2;
    
    collider1.filter.groupIndex = -100;
    collider2.filter.groupIndex = -100;
    
    // Same negative group = never collide
    EXPECT_FALSE(collider1.filter.ShouldCollide(collider2.filter));
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, AllCategoryBitsSet)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider1, collider2;
    
    collider1.filter.categoryBits = 0xFFFF;
    collider1.filter.maskBits = 0xFFFF;
    collider2.filter.categoryBits = 0xFFFF;
    collider2.filter.maskBits = 0xFFFF;
    
    // Should collide with everything
    EXPECT_TRUE(collider1.filter.ShouldCollide(collider2.filter));
    
    LOG_FUNC_EXIT();
}

TEST(ColliderComponentTest, NoCategoryBitsSet)
{
    LOG_FUNC_ENTER();
    ColliderComponent collider1, collider2;
    
    collider1.filter.categoryBits = 0x0000;
    collider1.filter.maskBits = 0x0000;
    collider2.filter.categoryBits = 0x0000;
    collider2.filter.maskBits = 0x0000;
    
    // Should not collide with anything
    EXPECT_FALSE(collider1.filter.ShouldCollide(collider2.filter));
    
    LOG_FUNC_EXIT();
}
