#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include <algorithm>
#include <cmath>

namespace Nyon::Physics
{
    /**
     * @brief Box2D-inspired Continuous Collision Detection implementation.
     * 
     * Implements swept collision detection to prevent tunneling through
     * thin obstacles and fast-moving objects.
     */
    class CollisionDetectionStrategy
    {
    public:
        enum class DetectionType
        {
            AABB_ONLY,          // Discrete collision detection only
            CCD_REQUIRED,       // Continuous collision detection required
            HYBRID              // Combination of both approaches
        };
        
        /**
         * @brief Determine collision detection strategy based on object characteristics.
         */
        static DetectionType DetermineStrategy(
            const ECS::PhysicsBodyComponent& bodyA,
            const ECS::PhysicsBodyComponent& bodyB,
            float deltaTime)
        {
            // Calculate movement distances for this frame
            float distanceA = bodyA.velocity.Length() * deltaTime;
            float distanceB = bodyB.velocity.Length() * deltaTime;
            float maxDistance = std::max(distanceA, distanceB);
            
            // Box2D-style thresholds
            const float VELOCITY_THRESHOLD = 100.0f;      // pixels/second
            const float SIZE_THRESHOLD = 20.0f;           // minimum object size
            const float DISTANCE_RATIO_THRESHOLD = 0.5f;  // distance vs size ratio
            
            // Check if either body is moving fast
            bool isFastMoving = (bodyA.velocity.Length() > VELOCITY_THRESHOLD) || 
                               (bodyB.velocity.Length() > VELOCITY_THRESHOLD);
            
            // Check if objects are small (more prone to tunneling)
            bool isSmall = (std::min(GetApproximateSize(bodyA), GetApproximateSize(bodyB)) < SIZE_THRESHOLD);
                            
            // Check if movement distance is large relative to object size
            float maxSize = std::max(GetApproximateSize(bodyA), GetApproximateSize(bodyB));
            bool highDistanceRatio = maxSize > 0.0f && (maxDistance / maxSize) > DISTANCE_RATIO_THRESHOLD;
            
            // Decision logic
            if (isFastMoving || isSmall || highDistanceRatio)
            {
                return DetectionType::CCD_REQUIRED;
            }
            
            return DetectionType::AABB_ONLY;
        }
        
        /**
         * @brief Get approximate size of a physics body based on collider geometry.
         * 
         * Uses the collider's AABB to estimate physical size, not mass.
         * Returns the smallest half-extent which is used for tunneling thresholds.
         */
        static float GetApproximateSize(const ECS::ColliderComponent& collider, const Math::Vector2& position)
        {
            Math::Vector2 min, max;
            collider.CalculateAABB(position, 0.0f, min, max);
            Math::Vector2 extents = (max - min) * 0.5f;
            return std::min(extents.x, extents.y); // smallest half-extent = tunnelling threshold
        }
        
        /**
         * @brief Legacy overload for backward compatibility - returns fixed default size.
         * @deprecated Use GetApproximateSize(collider, position) instead.
         */
        static float GetApproximateSize(const ECS::PhysicsBodyComponent& /*body*/)
        {
            return 50.0f; // Default size when collider info not available
        }
        
        /**
         * @brief Box2D-style swept AABB collision detection.
         * 
         * Calculates time of impact between two moving AABBs using slab method.
         * This provides consistent geometry between TOI calculation and resolution.
         */
        static float CalculateTimeOfImpact(
            const Math::Vector2& startPosA, const Math::Vector2& endPosA,
            const Math::Vector2& startPosB, const Math::Vector2& endPosB,
            float halfWidthA, float halfHeightA,
            float halfWidthB, float halfHeightB)
        {
            // Calculate relative motion
            Math::Vector2 relStart = startPosA - startPosB;
            Math::Vector2 relEnd = endPosA - endPosB;
            Math::Vector2 relVel = relEnd - relStart;
            
            // Combined half-extents
            float combinedHalfWidth = halfWidthA + halfWidthB;
            float combinedHalfHeight = halfHeightA + halfHeightB;
            
            // Already overlapping
            if (std::abs(relStart.x) < combinedHalfWidth && 
                std::abs(relStart.y) < combinedHalfHeight)
            {
                return 0.0f;
            }
            
            // Slab method for swept AABB
            float tEnter = -std::numeric_limits<float>::max();
            float tExit = std::numeric_limits<float>::max();
            
            // Check X-axis
            if (std::abs(relVel.x) > 1e-6f)
            {
                float t1 = (combinedHalfWidth - relStart.x) / relVel.x;
                float t2 = (-combinedHalfWidth - relStart.x) / relVel.x;
                
                if (t1 > t2) std::swap(t1, t2);
                
                tEnter = std::max(tEnter, t1);
                tExit = std::min(tExit, t2);
            }
            else
            {
                // No motion on X-axis, check overlap
                if (std::abs(relStart.x) >= combinedHalfWidth)
                {
                    return -1.0f; // No intersection
                }
            }
            
            // Check Y-axis
            if (std::abs(relVel.y) > 1e-6f)
            {
                float t1 = (combinedHalfHeight - relStart.y) / relVel.y;
                float t2 = (-combinedHalfHeight - relStart.y) / relVel.y;
                
                if (t1 > t2) std::swap(t1, t2);
                
                tEnter = std::max(tEnter, t1);
                tExit = std::min(tExit, t2);
            }
            else
            {
                // No motion on Y-axis, check overlap
                if (std::abs(relStart.y) >= combinedHalfHeight)
                {
                    return -1.0f; // No intersection
                }
            }
            
            // Valid intersection
            if (tEnter <= tExit && tEnter >= 0.0f && tEnter <= 1.0f)
            {
                return tEnter;
            }
            
            return -1.0f; // No intersection
        }
        
        /**
         * @brief Legacy sphere-based TOI for backward compatibility.
         * 
         * @deprecated Use the AABB-based version for consistency.
         */
        static float CalculateSphereTimeOfImpact(
            const Math::Vector2& startPosA, const Math::Vector2& endPosA,
            const Math::Vector2& startPosB, const Math::Vector2& endPosB,
            float radiusA, float radiusB)
        {
            // Relative motion
            Math::Vector2 relStart = startPosA - startPosB;
            Math::Vector2 relEnd = endPosA - endPosB;
            Math::Vector2 relVel = relEnd - relStart;
            
            float combinedRadius = radiusA + radiusB;
            
            // Already overlapping
            if (relStart.LengthSquared() <= combinedRadius * combinedRadius)
            {
                return 0.0f;
            }
            
            // Moving away from each other
            float relStartDotVel = Math::Vector2::Dot(relStart, relVel);
            if (relStartDotVel >= 0.0f)
            {
                return -1.0f;
            }
            
            // Solve quadratic equation: a*t² + b*t + c = 0
            float a = relVel.LengthSquared();
            float b = 2.0f * relStartDotVel;
            float c = relStart.LengthSquared() - combinedRadius * combinedRadius;
            
            float discriminant = b * b - 4.0f * a * c;
            
            if (discriminant < 0.0f)
            {
                return -1.0f; // No intersection
            }
            
            // First intersection time
            float t = (-b - std::sqrt(discriminant)) / (2.0f * a);
            
            // Clamp to valid range
            return std::clamp(t, 0.0f, 1.0f);
        }
        
        /**
         * @brief Box2D-style swept polygon collision detection.
         * 
         * Uses conservative advancement with distance queries.
         */
        static float CalculatePolygonTOI(
            const std::vector<Math::Vector2>& verticesA,
            const Math::Vector2& startPosA, const Math::Vector2& endPosA,
            const std::vector<Math::Vector2>& verticesB,
            const Math::Vector2& startPosB, const Math::Vector2& endPosB,
            float maxIterations = 20)
        {
            float t = 0.0f;
            float target = 1.0f;
            
            Math::Vector2 posA = startPosA;
            Math::Vector2 posB = startPosB;
            
            for (int iter = 0; iter < maxIterations; ++iter)
            {
                // Calculate current separation using SAT
                float separation = CalculatePolygonSeparation(verticesA, posA, verticesB, posB);
                
                if (separation <= 0.0f)
                {
                    // Collision detected
                    return t;
                }
                
                // Conservative advancement
                Math::Vector2 dPosA = endPosA - startPosA;
                Math::Vector2 dPosB = endPosB - startPosB;
                
                // Compute the separation axis (normalized line from posA to posB)
                Math::Vector2 separationDir = posB - posA;
                float sepLen = separationDir.Length();
                if (sepLen < 1e-6f) break; // Bodies at same position
                separationDir = separationDir * (1.0f / sepLen);
                
                // Approach speed is the component of relative velocity along the separation direction
                float approachSpeed = Math::Vector2::Dot(dPosB - dPosA, separationDir);
                if (approachSpeed <= 0.0f)
                {
                    break; // Bodies diverging or moving parallel
                }
                
                float delta = separation / approachSpeed;
                if (delta < 1e-6f)
                {
                    break; // Converged
                }
                
                t += delta;
                if (t >= target)
                {
                    return target;
                }
                
                // Interpolate positions
                float ratio = t / target;
                posA = startPosA + dPosA * ratio;
                posB = startPosB + dPosB * ratio;
            }
            
            return target;
        }
        
    private:
        /**
         * @brief Calculate minimum separation between two polygons using SAT.
         */
        static float CalculatePolygonSeparation(
            const std::vector<Math::Vector2>& verticesA, const Math::Vector2& posA,
            const std::vector<Math::Vector2>& verticesB, const Math::Vector2& posB)
        {
            float minSeparation = std::numeric_limits<float>::max();
            
            // Check normals of polygon A
            for (size_t i = 0; i < verticesA.size(); ++i)
            {
                size_t next = (i + 1) % verticesA.size();
                Math::Vector2 edge = verticesA[next] - verticesA[i];
                Math::Vector2 normal = {-edge.y, edge.x};
                normal = normal.Normalize();
                
                float separation = ProjectPolygon(verticesA, posA, normal) - 
                                 ProjectPolygon(verticesB, posB, -normal);
                minSeparation = std::min(minSeparation, separation);
            }
            
            // Check normals of polygon B
            for (size_t i = 0; i < verticesB.size(); ++i)
            {
                size_t next = (i + 1) % verticesB.size();
                Math::Vector2 edge = verticesB[next] - verticesB[i];
                Math::Vector2 normal = {-edge.y, edge.x};
                normal = normal.Normalize();
                
                float separation = ProjectPolygon(verticesB, posB, normal) - 
                                 ProjectPolygon(verticesA, posA, -normal);
                minSeparation = std::min(minSeparation, separation);
            }
            
            return minSeparation;
        }
        
        /**
         * @brief Project polygon onto axis.
         */
        static float ProjectPolygon(
            const std::vector<Math::Vector2>& vertices, 
            const Math::Vector2& position, 
            const Math::Vector2& axis)
        {
            float minProj = Math::Vector2::Dot(vertices[0] + position, axis);
            float maxProj = minProj;
            
            for (size_t i = 1; i < vertices.size(); ++i)
            {
                float proj = Math::Vector2::Dot(vertices[i] + position, axis);
                minProj = std::min(minProj, proj);
                maxProj = std::max(maxProj, proj);
            }
            
            return maxProj;
        }
    };
}