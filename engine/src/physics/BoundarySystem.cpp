// SPDX-FileCopyrightText: 2026 Nyon Engine
// SPDX-License-Identifier: MIT

#include "nyon/physics/BoundarySystem.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace Nyon::Physics
{
    BoundaryCollision BoundarySystem::DetectCircleCollision(
        const Math::Vector2& center,
        float radius,
        const Math::Vector2& velocity,
        float dt) const
    {
        BoundaryCollision result;
        
        if (!m_Boundary.enabled)
            return result;
        
        const auto planes = m_Boundary.GetPlanes();
        
        // For CCD, sweep the circle along its velocity vector
        Math::Vector2 sweptCenter = center;
        if (dt > 0.0f && velocity.LengthSquared() > 0.0f)
        {
            // Use continuous collision detection
            sweptCenter = center + velocity * dt;
        }
        
        // Test against each boundary plane
        for (int i = 0; i < 4; ++i)
        {
            const auto& plane = planes[i];
            
            // Distance from circle center to plane
            float distance = Math::Vector2::Dot(plane.normal, sweptCenter) + plane.distance;
            
            // Check for penetration
            if (distance < radius)
            {
                result.collided = true;
                result.contactPoint = sweptCenter - plane.normal * radius;
                result.normal = -plane.normal;  // Normal points outward
                result.penetration = radius - distance;
                result.boundaryIndex = i;
                
                // For CCD, compute time of impact
                if (dt > 0.0f && velocity.LengthSquared() > 0.0f)
                {
                    float approachVelocity = Math::Vector2::Dot(velocity, -plane.normal);
                    if (approachVelocity > 0.0f)
                    {
                        // Scale penetration by approach velocity for better CCD
                        result.penetration = std::max(result.penetration, 
                                                     approachVelocity * dt * 0.5f);
                    }
                }
                
                break;  // Report first collision
            }
        }
        
        return result;
    }
    
    BoundaryCollision BoundarySystem::DetectPolygonCollision(
        const std::vector<Math::Vector2>& vertices,
        const Math::Vector2& velocity,
        float angularVelocity,
        float dt) const
    {
        BoundaryCollision result;
        
        if (!m_Boundary.enabled || vertices.empty())
            return result;
        
        const auto planes = m_Boundary.GetPlanes();
        
        // Compute swept AABB for CCD
        Math::Vector2 minVertex = vertices[0];
        Math::Vector2 maxVertex = vertices[0];
        
        for (const auto& vertex : vertices)
        {
            minVertex.x = std::min(minVertex.x, vertex.x);
            minVertex.y = std::min(minVertex.y, vertex.y);
            maxVertex.x = std::max(maxVertex.x, vertex.x);
            maxVertex.y = std::max(maxVertex.y, vertex.y);
        }
        
        // Expand AABB for motion
        if (dt > 0.0f)
        {
            float motionRadius = velocity.Length() * dt;
            minVertex.x -= motionRadius;
            minVertex.y -= motionRadius;
            maxVertex.x += motionRadius;
            maxVertex.y += motionRadius;
        }
        
        // Early out if swept AABB doesn't overlap boundary
        if (!m_Boundary.OverlapsAABB(minVertex, maxVertex))
            return result;
        
        // Test each boundary plane using SAT
        for (int i = 0; i < 4; ++i)
        {
            const auto& plane = planes[i];
            
            // Find minimum distance from polygon vertices to plane
            float minDistance = std::numeric_limits<float>::infinity();
            Math::Vector2 closestVertex;
            
            for (const auto& vertex : vertices)
            {
                float distance = Math::Vector2::Dot(plane.normal, vertex) + plane.distance;
                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestVertex = vertex;
                }
            }
            
            // Check for penetration
            if (minDistance < 0.0f)
            {
                result.collided = true;
                result.contactPoint = closestVertex;
                result.normal = -plane.normal;  // Points outward
                result.penetration = -minDistance;
                result.boundaryIndex = i;
                
                // Apply CCD correction if moving
                if (dt > 0.0f && velocity.LengthSquared() > 0.0f)
                {
                    float approachVelocity = Math::Vector2::Dot(velocity, -plane.normal);
                    if (approachVelocity > 0.0f)
                    {
                        result.penetration = std::max(result.penetration,
                                                     approachVelocity * dt * 0.5f);
                    }
                }
                
                break;  // Report first collision
            }
        }
        
        return result;
    }
    
    BoundaryCollision BoundarySystem::DetectCapsuleCollision(
        const Math::Vector2& start,
        const Math::Vector2& end,
        float radius,
        const Math::Vector2& velocity,
        float dt) const
    {
        BoundaryCollision result;
        
        if (!m_Boundary.enabled)
            return result;
        
        const auto planes = m_Boundary.GetPlanes();
        
        // Test capsule as line segment with radius
        for (int i = 0; i < 4; ++i)
        {
            const auto& plane = planes[i];
            
            // Find closest point on capsule segment to plane
            float distStart = Math::Vector2::Dot(plane.normal, start) + plane.distance;
            float distEnd = Math::Vector2::Dot(plane.normal, end) + plane.distance;
            
            float minDistance = std::min(distStart, distEnd);
            Math::Vector2 closestPoint = (distStart < distEnd) ? start : end;
            
            // Add radius
            minDistance -= radius;
            
            if (minDistance < 0.0f)
            {
                result.collided = true;
                result.contactPoint = closestPoint - plane.normal * radius;
                result.normal = -plane.normal;
                result.penetration = -minDistance;
                result.boundaryIndex = i;
                
                // CCD correction
                if (dt > 0.0f && velocity.LengthSquared() > 0.0f)
                {
                    float approachVelocity = Math::Vector2::Dot(velocity, -plane.normal);
                    if (approachVelocity > 0.0f)
                    {
                        result.penetration = std::max(result.penetration,
                                                     approachVelocity * dt * 0.5f);
                    }
                }
                
                break;
            }
        }
        
        return result;
    }
    
    Math::Vector2 BoundarySystem::ResolvePosition(
        const Math::Vector2& currentPosition,
        const BoundaryCollision& collision,
        float linearSlop)
    {
        if (!collision.collided)
            return currentPosition;
        
        // Only correct if penetration is significant
        if (collision.penetration <= linearSlop)
            return currentPosition;
        
        // Push object out along collision normal
        float correctionMagnitude = collision.penetration - linearSlop;
        return currentPosition + collision.normal * correctionMagnitude;
    }
    
    Math::Vector2 BoundarySystem::ComputeReflection(
        const Math::Vector2& incomingVelocity,
        const Math::Vector2& normal,
        float restitution,
        float friction)
    {
        // Decompose velocity into normal and tangential components
        float normalVelocity = Math::Vector2::Dot(incomingVelocity, normal);
        
        // Only reflect if moving toward the boundary
        if (normalVelocity >= 0.0f)
            return incomingVelocity;
        
        Math::Vector2 reflectedVelocity = incomingVelocity;
        
        // Apply restitution to normal component
        reflectedVelocity = reflectedVelocity - normal * ((1.0f + restitution) * normalVelocity);
        
        // Apply friction to tangential component
        Math::Vector2 tangentVelocity = incomingVelocity - normal * normalVelocity;
        float tangentSpeed = tangentVelocity.Length();
        
        if (tangentSpeed > 1e-6f)
        {
            Math::Vector2 tangentDirection = tangentVelocity / tangentSpeed;
            float frictionImpulse = friction * std::abs(normalVelocity);
            float newTangentSpeed = std::max(0.0f, tangentSpeed - frictionImpulse);
            reflectedVelocity -= tangentDirection * (tangentSpeed - newTangentSpeed);
        }
        
        return reflectedVelocity;
    }
    
    BoundaryCollision BoundarySystem::DetectPlaneCCD(
        const Math::Vector2& point,
        float radius,
        const Math::Vector2& velocity,
        float dt,
        const Boundary::Plane& plane) const
    {
        BoundaryCollision result;
        
        // Distance at start of time step
        float distStart = Math::Vector2::Dot(plane.normal, point) + plane.distance;
        
        // Distance at end of time step
        Math::Vector2 sweptPoint = point + velocity * dt;
        float distEnd = Math::Vector2::Dot(plane.normal, sweptPoint) + plane.distance;
        
        // Check if crossed the plane
        if (distStart > radius && distEnd < radius)
        {
            // Linear interpolation to find time of impact
            float t = (distStart - radius) / (distStart - distEnd);
            
            result.collided = true;
            result.contactPoint = point + velocity * (t * dt) - plane.normal * radius;
            result.normal = -plane.normal;
            result.penetration = radius - distEnd;
            
            // Mark which side was hit
            if (plane.normal.x > 0.5f) result.boundaryIndex = 0;      // Left
            else if (plane.normal.x < -0.5f) result.boundaryIndex = 1; // Right
            else if (plane.normal.y > 0.5f) result.boundaryIndex = 2;  // Bottom
            else if (plane.normal.y < -0.5f) result.boundaryIndex = 3; // Top
        }
        
        return result;
    }
}
