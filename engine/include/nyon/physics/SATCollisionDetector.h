// SPDX-FileCopyrightText: 2026 Nyon Engine
// SPDX-License-Identifier: MIT

#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include <vector>
#include <array>
#include <cstdint>

namespace Nyon::Physics
{
    /**
     * @brief Contact point data structure
     */
    struct ContactPoint
    {
        Math::Vector2 position;     // World space contact point
        Math::Vector2 normal;       // Normal pointing from A to B
        float separation;           // Negative = penetration, positive = gap
        float normalImpulse;        // Accumulated normal impulse
        float tangentImpulse;       // Accumulated friction impulse
        float normalMass;           // Effective mass in normal direction
        float tangentMass;          // Effective mass in tangent direction
        uint32_t featureId;         // Feature identifier for persistence
        bool persisted;             // True if this point existed last frame
    };
    
    /**
     * @brief Contact manifold - collection of contact points between two shapes
     */
    struct ContactManifold
    {
        std::vector<ContactPoint> points;  // Up to 2 contact points in 2D
        Math::Vector2 normal;              // Unit normal from A to B
        Math::Vector2 localNormal;         // Normal in local space of A
        Math::Vector2 localPoint;          // Pivot point in local space of A
        uint32_t entityIdA;                // First entity
        uint32_t entityIdB;                // Second entity
        uint32_t shapeIdA;                 // First shape
        uint32_t shapeIdB;                 // Second shape
        bool touching = false;             // True if actually in contact
        
        void Clear()
        {
            points.clear();
            normal = {0.0f, 0.0f};
            localNormal = {0.0f, 0.0f};
            localPoint = {0.0f, 0.0f};
            touching = false;
        }
    };
    
    /**
     * @brief Separating Axis Theorem (SAT) collision detection system
     * 
     * Implements robust polygon collision detection using SAT with support for:
     * - Circle vs Circle
     * - Circle vs Polygon
     * - Polygon vs Polygon
     * - Capsule vs all shapes
     * 
     * Features:
     * - Continuous collision detection (CCD) for fast objects
     * - Contact persistence for stability
     * - Speculative contacts for anti-tunneling
     */
    class SATCollisionDetector
    {
    public:
        SATCollisionDetector() = default;
        ~SATCollisionDetector() = default;
        
        /**
         * @brief Detect collision between two circles
         * 
         * Fast analytical solution for circle-circle collision.
         * 
         * @param entityIdA First entity ID
         * @param entityIdB Second entity ID
         * @param shapeIdA First shape ID
         * @param shapeIdB Second shape ID
         * @param circleA First circle shape
         * @param circleB Second circle shape
         * @param transformA First transform
         * @param transformB Second transform
         * @param speculativeDistance Distance for speculative contacts
         * @return ContactManifold result
         */
        ContactManifold DetectCircleCircle(
            uint32_t entityIdA,
            uint32_t entityIdB,
            uint32_t shapeIdA,
            uint32_t shapeIdB,
            const ColliderComponent::CircleShape& circleA,
            const ColliderComponent::CircleShape& circleB,
            const TransformComponent& transformA,
            const TransformComponent& transformB,
            float speculativeDistance = 0.02f);
        
        /**
         * @brief Detect collision between circle and polygon using SAT
         * 
         * Tests circle against each polygon face and finds deepest penetration.
         * 
         * @param entityIdA Circle entity ID
         * @param entityIdB Polygon entity ID
         * @param shapeIdA Circle shape ID
         * @param shapeB Polygon shape ID
         * @param circle Circle shape
         * @param polygon Polygon shape
         * @param transformA Circle transform
         * @param transformB Polygon transform
         * @param speculativeDistance Distance for speculative contacts
         * @return ContactManifold result
         */
        ContactManifold DetectCirclePolygon(
            uint32_t entityIdA,
            uint32_t entityIdB,
            uint32_t shapeIdA,
            uint32_t shapeIdB,
            const ColliderComponent::CircleShape& circle,
            const ColliderComponent::PolygonShape& polygon,
            const TransformComponent& transformA,
            const TransformComponent& transformB,
            float speculativeDistance = 0.02f);
        
        /**
         * @brief Detect collision between two polygons using SAT
         * 
         * Comprehensive SAT test with edge-edge and vertex-face contacts.
         * Uses reference face method for clipping.
         * 
         * @param entityIdA First entity ID
         * @param entityIdB Second entity ID
         * @param shapeIdA First shape ID
         * @param shapeIdB Second shape ID
         * @param polygonA First polygon
         * @param polygonB Second polygon
         * @param transformA First transform
         * @param transformB Second transform
         * @param speculativeDistance Distance for speculative contacts
         * @return ContactManifold result
         */
        ContactManifold DetectPolygonPolygon(
            uint32_t entityIdA,
            uint32_t entityIdB,
            uint32_t shapeIdA,
            uint32_t shapeIdB,
            const ColliderComponent::PolygonShape& polygonA,
            const ColliderComponent::PolygonShape& polygonB,
            const TransformComponent& transformA,
            const TransformComponent& transformB,
            float speculativeDistance = 0.02f);
        
        /**
         * @brief Detect collision involving a capsule
         * 
         * Handles capsule vs circle, polygon, and capsule.
         * 
         * @param entityIdA Capsule entity ID
         * @param entityIdB Other entity ID
         * @param capsule Capsule shape
         * @param otherCollider Other collider shape
         * @param transformA Capsule transform
         * @param transformB Other transform
         * @param speculativeDistance Distance for speculative contacts
         * @return ContactManifold result
         */
        ContactManifold DetectCapsuleCollision(
            uint32_t entityIdA,
            uint32_t entityIdB,
            const ColliderComponent::CapsuleShape& capsule,
            const std::variant<
                ColliderComponent::PolygonShape,
                ColliderComponent::CircleShape,
                ColliderComponent::CapsuleShape>& otherCollider,
            const TransformComponent& transformA,
            const TransformComponent& transformB,
            float speculativeDistance = 0.02f);
        
        /**
         * @brief Compute contact manifold for rotated shapes
         * 
         * Helper function to transform shapes to world space.
         */
        static void ComputePolygonWorld(
            const ColliderComponent::PolygonShape& polygon,
            const TransformComponent& transform,
            std::vector<Math::Vector2>& outVertices,
            std::vector<Math::Vector2>& outNormals);
        
        /**
         * @brief Compute circle center in world space
         */
        static void ComputeCircleCenters(
            const ColliderComponent::CircleShape& circle,
            const TransformComponent& transform,
            Math::Vector2& outCenter);
        
        /**
         * @brief Compute capsule endpoints in world space
         */
        static void ComputeCapsuleEndpoints(
            const ColliderComponent::CapsuleShape& capsule,
            const TransformComponent& transform,
            Math::Vector2& outStart,
            Math::Vector2& outEnd);
        
    private:
        /**
         * @brief Find incident face on polygon B relative to face normal on A
         */
        static void FindIncidentFace(
            const std::vector<Math::Vector2>& vertsA,
            const std::vector<Math::Vector2>& normalsA,
            const std::vector<Math::Vector2>& vertsB,
            const std::vector<Math::Vector2>& normalsB,
            int referenceIndex,
            int& incidentIndex1,
            int& incidentIndex2);
        
        /**
         * @brief Clip polygon vertices against a plane
         */
        static std::vector<Math::Vector2> ClipVertices(
            const std::vector<Math::Vector2>& vertices,
            const Math::Vector2& normal,
            float offset);
        
        /**
         * @brief Sutherland-Hodgman clipping for contact generation
         */
        static void ClipSegmentToLine(
            std::vector<ContactPoint>& vOut,
            const std::vector<ContactPoint>& vIn,
            const Math::Vector2& normal,
            float offset,
            int clipEdgeIndex);
        
        /**
         * @class CCD
         * @brief Continuous Collision Detection utilities
         */
        class CCD
        {
        public:
            /**
             * @brief Time of Impact (TOI) for moving shapes
             * 
             * @param fraction Output: time of impact as fraction of time step [0,1]
             * @param point Output: contact point at TOI
             * @param normal Output: contact normal at TOI
             * @return true if impact occurred within time step
             */
            static bool ComputeTOI(
                const ContactManifold& manifold,
                const Math::Vector2& velocityA,
                const Math::Vector2& velocityB,
                float angularVelocityA,
                float angularVelocityB,
                float dt,
                float& fraction,
                Math::Vector2& point,
                Math::Vector2& normal);
            
            /**
             * @brief Conservative advancement for TOI
             */
            static float ConservativeAdvancement(
                const Math::Vector2& pointA,
                const Math::Vector2& pointB,
                const Math::Vector2& velocityA,
                const Math::Vector2& velocityB,
                float distance);
        };
    };
}
