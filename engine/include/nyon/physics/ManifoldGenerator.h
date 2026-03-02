#pragma once

#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/math/Vector2.h"
#include <cstdint>
#include <vector>

namespace Nyon::Physics
{
    /**
     * @brief Contact point structure for collision information.
     * 
     * Represents a single contact point between two colliding shapes.
     */
    struct ContactPoint
    {
        Math::Vector2 position;         // World position of contact point
        Math::Vector2 normal;           // Contact normal (points from shape A to shape B)
        Math::Vector2 localPointA;      // Contact point in shape A's local coordinates
        Math::Vector2 localPointB;      // Contact point in shape B's local coordinates
        float separation;               // Separation distance (negative = penetration)
        float normalImpulse;            // Accumulated normal impulse
        float tangentImpulse;           // Accumulated tangent impulse
        float normalMass;               // Normal constraint mass
        float tangentMass;              // Tangent constraint mass
        uint32_t featureId;             // Feature identifier for persistence
        bool persisted;                 // Whether this point persisted from previous frame
        
        ContactPoint() : 
            separation(0.0f), normalImpulse(0.0f), tangentImpulse(0.0f),
            normalMass(0.0f), tangentMass(0.0f), featureId(0), persisted(false) {}
    };
    
    /**
     * @brief Contact manifold representing collision between two shapes.
     * 
     * Contains all contact points and geometric information for a collision pair.
     */
    struct ContactManifold
    {
        std::vector<ContactPoint> points;   // Contact points (0-2 typically)
        Math::Vector2 normal;               // Contact normal
        Math::Vector2 localNormal;          // Normal in local coordinates
        Math::Vector2 localPoint;           // Reference point in local coordinates
        float friction;                     // Combined friction coefficient
        float restitution;                  // Combined restitution coefficient
        float tangentSpeed;                 // Tangent speed for friction
        uint32_t entityIdA;                 // First entity ID
        uint32_t entityIdB;                 // Second entity ID
        uint32_t shapeIdA;                  // First shape ID
        uint32_t shapeIdB;                  // Second shape ID
        bool touching;                      // Whether shapes are touching
        uint64_t contactKey;                // Unique key for contact identification
        
        ContactManifold() : 
            friction(0.0f), restitution(0.0f), tangentSpeed(0.0f),
            entityIdA(0), entityIdB(0), shapeIdA(0), shapeIdB(0),
            touching(false), contactKey(0) {}
            
        // Generate unique contact key
        void GenerateKey()
        {
            // Simple hash combining entity and shape IDs
            contactKey = (static_cast<uint64_t>(entityIdA) << 32) | 
                        (static_cast<uint64_t>(entityIdB) << 16) |
                        (static_cast<uint64_t>(shapeIdA) << 8) | 
                        static_cast<uint64_t>(shapeIdB);
        }
    };
    
    /**
     * @brief Manifold generator for collision contact point generation.
     * 
     * Generates contact manifolds from collision detection results.
     * Supports various shape combinations and uses clipping algorithms
     * similar to Box2D's approach.
     */
    class ManifoldGenerator
    {
    public:
        // Generate manifold for circle-circle collision
        static ContactManifold GenerateCircleCircleManifold(
            const Math::Vector2& centerA, float radiusA,
            const Math::Vector2& centerB, float radiusB,
            uint32_t entityIdA, uint32_t entityIdB,
            uint32_t shapeIdA, uint32_t shapeIdB,
            float frictionA, float frictionB,
            float restitutionA, float restitutionB);
            
        // Generate manifold for circle-polygon collision
        static ContactManifold GenerateCirclePolygonManifold(
            const Math::Vector2& circleCenter, float circleRadius,
            const std::vector<Math::Vector2>& polygonVertices,
            const std::vector<Math::Vector2>& polygonNormals,
            float polygonRadius,
            uint32_t entityIdA, uint32_t entityIdB,
            uint32_t shapeIdA, uint32_t shapeIdB,
            float frictionA, float frictionB,
            float restitutionA, float restitutionB);
            
        // Generate manifold for polygon-polygon collision
        static ContactManifold GeneratePolygonPolygonManifold(
            const std::vector<Math::Vector2>& verticesA,
            const std::vector<Math::Vector2>& normalsA,
            float radiusA,
            const std::vector<Math::Vector2>& verticesB,
            const std::vector<Math::Vector2>& normalsB,
            float radiusB,
            uint32_t entityIdA, uint32_t entityIdB,
            uint32_t shapeIdA, uint32_t shapeIdB,
            float frictionA, float frictionB,
            float restitutionA, float restitutionB);
            
        // Generate manifold for capsule-shape collisions
        static ContactManifold GenerateCapsuleManifold(
            const Math::Vector2& capCenter1, const Math::Vector2& capCenter2, float capRadius,
            const Nyon::ECS::ColliderComponent* otherCollider,
            uint32_t entityIdA, uint32_t entityIdB,
            uint32_t shapeIdA, uint32_t shapeIdB,
            float frictionA, float frictionB,
            float restitutionA, float restitutionB);
            
        // Update existing manifold with new collision data
        static void UpdateManifold(ContactManifold& manifold,
                                 const Math::Vector2& newNormal,
                                 const std::vector<ContactPoint>& newPoints);
                                 
        // Warm start manifold by copying impulses from previous frame
        static void WarmStartManifold(ContactManifold& manifold,
                                    const ContactManifold& oldManifold);
                                    
        // Calculate combined material properties
        static float CalculateCombinedFriction(float frictionA, float frictionB);
        static float CalculateCombinedRestitution(float restitutionA, float restitutionB);
        
    private:
        // Helper methods for contact point generation
        static Math::Vector2 FindBestReferenceEdge(
            const std::vector<Math::Vector2>& vertices,
            const std::vector<Math::Vector2>& normals,
            const Math::Vector2& searchDirection);
            
        static std::vector<ContactPoint> ClipEdges(
            const Math::Vector2& v11, const Math::Vector2& v12,  // Reference edge
            const Math::Vector2& v21, const Math::Vector2& v22,  // Incident edge
            const Math::Vector2& normal,                         // Contact normal
            float totalRadius);
            
        static ContactPoint CreateContactPoint(
            const Math::Vector2& point, const Math::Vector2& normal,
            const Math::Vector2& localPointA, const Math::Vector2& localPointB,
            float separation, uint32_t featureId);
            
        // Constants
        static constexpr float SPECULATIVE_DISTANCE = 0.01f;  // Distance threshold for contact
        static constexpr float LINEAR_SLOP = 0.005f;          // Linear slop for position correction
        static constexpr float MIN_SEPARATION = -0.05f;       // Minimum separation for contact
    };
}
