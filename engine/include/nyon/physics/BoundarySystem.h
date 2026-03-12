// SPDX-FileCopyrightText: 2026 Nyon Engine
// SPDX-License-Identifier: MIT

#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/EntityManager.h"
#include <array>
#include <vector>
#include <cstdint>

namespace Nyon::Physics
{
    /**
     * @brief Screen/world boundary for collision detection
     * 
     * Defines rectangular boundaries that objects cannot pass through.
     * Used to prevent objects from leaving the playable area.
     */
    struct Boundary
    {
        Math::Vector2 minBounds;  // Bottom-left corner
        Math::Vector2 maxBounds;  // Top-right corner
        float thickness = 1.0f;   // Boundary thickness for collision
        bool enabled = true;
        
        Boundary() = default;
        Boundary(const Math::Vector2& min, const Math::Vector2& max)
            : minBounds(min), maxBounds(max) {}
        
        // Get the four boundary planes (normal points inward)
        struct Plane
        {
            Math::Vector2 normal;   // Points inward
            Math::Vector2 point;    // Point on the plane
            float distance;         // Distance from origin along normal
        };
        
        std::array<Plane, 4> GetPlanes() const
        {
            std::array<Plane, 4> planes;
            
            // Left wall (normal points right): x >= minBounds.x
            planes[0] = {
                .normal = {1.0f, 0.0f},
                .point = {minBounds.x, minBounds.y},
                .distance = minBounds.x
            };
            
            // Right wall (normal points left): x <= maxBounds.x
            planes[1] = {
                .normal = {-1.0f, 0.0f},
                .point = {maxBounds.x, minBounds.y},
                .distance = -maxBounds.x
            };
            
            // Bottom wall (normal points up): y >= minBounds.y
            planes[2] = {
                .normal = {0.0f, 1.0f},
                .point = {minBounds.x, minBounds.y},
                .distance = minBounds.y
            };
            
            // Top wall (normal points down): y <= maxBounds.y
            planes[3] = {
                .normal = {0.0f, -1.0f},
                .point = {minBounds.x, maxBounds.y},
                .distance = -maxBounds.y
            };
            
            return planes;
        }
        
        // Check if a point is inside the boundary
        bool ContainsPoint(const Math::Vector2& point) const
        {
            return point.x >= minBounds.x && point.x <= maxBounds.x &&
                   point.y >= minBounds.y && point.y <= maxBounds.y;
        }
        
        // Check if an AABB overlaps with the boundary
        bool OverlapsAABB(const Math::Vector2& aabbMin, const Math::Vector2& aabbMax) const
        {
            return !(aabbMax.x < minBounds.x || aabbMin.x > maxBounds.x ||
                     aabbMax.y < minBounds.y || aabbMin.y > maxBounds.y);
        }
    };
    
    /**
     * @brief Boundary collision result
     */
    struct BoundaryCollision
    {
        bool collided = false;
        Math::Vector2 contactPoint;
        Math::Vector2 normal;  // Points outward from boundary
        float penetration = 0.0f;
        int boundaryIndex = -1;  // Which wall was hit (0=left, 1=right, 2=bottom, 3=top)
    };
    
    /**
     * @brief Comprehensive boundary collision detection system
     * 
     * Handles collision detection between objects and world/screen boundaries.
     * Uses continuous collision detection to prevent tunneling.
     */
    class BoundarySystem
    {
    public:
        BoundarySystem() = default;
        ~BoundarySystem() = default;
        
        /**
         * @brief Set the world boundaries
         */
        void SetBoundaries(const Math::Vector2& minBounds, const Math::Vector2& maxBounds)
        {
            m_Boundary = Boundary{minBounds, maxBounds};
        }
        
        /**
         * @brief Enable or disable boundaries
         */
        void SetEnabled(bool enabled)
        {
            m_Boundary.enabled = enabled;
        }
        
        /**
         * @brief Check if boundaries are enabled
         */
        bool IsEnabled() const { return m_Boundary.enabled; }
        
        /**
         * @brief Get current boundaries
         */
        const Boundary& GetBoundaries() const { return m_Boundary; }
        
        /**
         * @brief Detect collision between a circle and boundaries
         * 
         * Uses continuous collision detection to prevent tunneling.
         * 
         * @param center Circle center position
         * @param radius Circle radius
         * @param velocity Circle velocity (for CCD)
         * @param dt Time step
         * @return BoundaryCollision result
         */
        BoundaryCollision DetectCircleCollision(
            const Math::Vector2& center,
            float radius,
            const Math::Vector2& velocity = Math::Vector2{0.0f, 0.0f},
            float dt = 0.0f) const;
        
        /**
         * @brief Detect collision between a polygon and boundaries
         * 
         * Uses SAT-based detection for accurate results.
         * 
         * @param vertices Polygon vertices in world space
         * @param velocity Polygon velocity (for CCD)
         * @param angularVelocity Polygon angular velocity (for CCD)
         * @param dt Time step
         * @return BoundaryCollision result
         */
        BoundaryCollision DetectPolygonCollision(
            const std::vector<Math::Vector2>& vertices,
            const Math::Vector2& velocity = Math::Vector2{0.0f, 0.0f},
            float angularVelocity = 0.0f,
            float dt = 0.0f) const;
        
        /**
         * @brief Detect collision between a capsule and boundaries
         * 
         * @param start Capsule start point (world space)
         * @param end Capsule end point (world space)
         * @param radius Capsule radius
         * @param velocity Capsule velocity (for CCD)
         * @param dt Time step
         * @return BoundaryCollision result
         */
        BoundaryCollision DetectCapsuleCollision(
            const Math::Vector2& start,
            const Math::Vector2& end,
            float radius,
            const Math::Vector2& velocity = Math::Vector2{0.0f, 0.0f},
            float dt = 0.0f) const;
        
        /**
         * @brief Resolve boundary collision by adjusting position
         * 
         * @param currentPosition Current position of the object
         * @param collision Collision result
         * @param linearSlop Position correction tolerance
         * @return Corrected position
         */
        static Math::Vector2 ResolvePosition(
            const Math::Vector2& currentPosition,
            const BoundaryCollision& collision,
            float linearSlop = 0.005f);
        
        /**
         * @brief Compute reflection velocity off boundary
         * 
         * @param incomingVelocity Incoming velocity
         * @param normal Boundary normal at contact
         * @param restitution Bounciness (0=no bounce, 1=full bounce)
         * @param friction Surface friction
         * @return Reflected velocity
         */
        static Math::Vector2 ComputeReflection(
            const Math::Vector2& incomingVelocity,
            const Math::Vector2& normal,
            float restitution = 0.0f,
            float friction = 0.1f);
        
    private:
        Boundary m_Boundary;
        
        /**
         * @brief Detect collision with a single boundary plane using CCD
         */
        BoundaryCollision DetectPlaneCCD(
            const Math::Vector2& point,
            float radius,
            const Math::Vector2& velocity,
            float dt,
            const Boundary::Plane& plane) const;
    };
}
