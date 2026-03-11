// SPDX-FileCopyrightText: 2026 Nyon Engine
// SPDX-License-Identifier: MIT

#include "nyon/physics/ContinuousCollisionDetection.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace Nyon::Physics
{
    TOIResult ContinuousCollisionDetection::CircleCircleCCD(
        const Math::Vector2& centerA,
        float radiusA,
        const Math::Vector2& velocityA,
        const Math::Vector2& centerB,
        float radiusB,
        const Math::Vector2& velocityB,
        float dt)
    {
        TOIResult result;
        result.entityIdA = 0;
        result.entityIdB = 0;
        
        // Relative motion
        Math::Vector2 d = centerB - centerA;
        Math::Vector2 v = velocityB - velocityA;
        float r = radiusA + radiusB;
        
        // Quadratic equation: |d + v*t|^2 = r^2
        float a = Math::Vector2::Dot(v, v);
        float b = 2.0f * Math::Vector2::Dot(d, v);
        float c = Math::Vector2::Dot(d, d) - r * r;
        
        if (a < 1e-8f)
        {
            // No relative motion
            if (c <= 0.0f)
            {
                result.hit = true;
                result.fraction = 0.0f;
                result.point = centerA;
                result.normal = (d.Length() > 1e-6f) ? d / d.Length() : Math::Vector2{1.0f, 0.0f};
            }
            return result;
        }
        
        float t;
        if (SolveQuadratic(a, b, c, t))
        {
            if (t >= 0.0f && t <= dt)
            {
                result.hit = true;
                result.fraction = t / dt;
                result.point = centerA + velocityA * result.fraction * dt;
                Math::Vector2 contactNormal = d + v * result.fraction;
                result.normal = (contactNormal.Length() > 1e-6f) ? 
                               contactNormal / contactNormal.Length() : Math::Vector2{1.0f, 0.0f};
            }
        }
        
        return result;
    }
    
    TOIResult ContinuousCollisionDetection::CirclePolygonCCD(
        const Math::Vector2& circleCenter,
        float circleRadius,
        const Math::Vector2& circleVelocity,
        const std::vector<Math::Vector2>& polygonVertices,
        const Math::Vector2& polygonVelocity,
        float polygonAngularVelocity,
        float dt)
    {
        TOIResult result;
        
        // Use conservative advancement
        float maxMotion = (circleVelocity.Length() + polygonVelocity.Length() + 
                          std::abs(polygonAngularVelocity) * 2.0f) * dt;
        
        // Iteratively advance until collision or end of time step
        float t = 0.0f;
        Math::Vector2 currentCenter = circleCenter;
        
        const int maxIterations = 10;
        for (int iter = 0; iter < maxIterations; ++iter)
        {
            // Find closest point on polygon to circle
            Math::Vector2 closestPoint = ClosestPointOnPolygon(currentCenter, polygonVertices);
            Math::Vector2 delta = closestPoint - currentCenter;
            float distance = delta.Length();
            
            // Check for collision
            if (distance <= circleRadius)
            {
                result.hit = true;
                result.fraction = t / dt;
                result.point = currentCenter;
                result.normal = (distance > 1e-6f) ? delta / distance : Math::Vector2{1.0f, 0.0f};
                break;
            }
            
            // Conservative advancement
            float relativeSpeed = Math::Vector2::Dot(circleVelocity - polygonVelocity, delta / distance);
            if (relativeSpeed <= 0.0f)
                break;  // Moving apart
            
            float advance = ConservativeAdvancement(distance - circleRadius, relativeSpeed);
            t += advance * dt;
            
            if (t >= dt)
                break;  // No collision in this time step
            
            // Update circle position
            currentCenter = circleCenter + circleVelocity * t;
        }
        
        return result;
    }
    
    TOIResult ContinuousCollisionDetection::PolygonPolygonCCD(
        const std::vector<Math::Vector2>& vertsA,
        const Math::Vector2& velocityA,
        float angularVelA,
        const std::vector<Math::Vector2>& vertsB,
        const Math::Vector2& velocityB,
        float angularVelB,
        float dt)
    {
        TOIResult result;
        
        // Simplified: use bounding circle approximation for CCD
        Math::Vector2 centerA{0.0f, 0.0f}, centerB{0.0f, 0.0f};
        float radiusA = 0.0f, radiusB = 0.0f;
        
        for (const auto& v : vertsA)
        {
            centerA = centerA + v;
            radiusA = std::max(radiusA, v.Length());
        }
        centerA = centerA / static_cast<float>(vertsA.size());
        
        for (const auto& v : vertsB)
        {
            centerB = centerB + v;
            radiusB = std::max(radiusB, v.Length());
        }
        centerB = centerB / static_cast<float>(vertsB.size());
        
        // Use circle-circle CCD as approximation
        result = CircleCircleCCD(centerA, radiusA, velocityA, centerB, radiusB, velocityB, dt);
        
        return result;
    }
    
    TOIResult ContinuousCollisionDetection::CapsuleCCD(
        const Math::Vector2& capStart,
        const Math::Vector2& capEnd,
        float capRadius,
        const Math::Vector2& velocity,
        int otherType,
        const void* otherData,
        const Math::Vector2& otherVelocity,
        float dt)
    {
        TOIResult result;
        
        // Treat capsule as swept circle
        Math::Vector2 capCenter = (capStart + capEnd) * 0.5f;
        
        if (otherType == 0)  // Circle
        {
            const auto* circleData = static_cast<const struct { Math::Vector2 center; float radius; }*>(otherData);
            result = CircleCircleCCD(capCenter, capRadius, velocity, 
                                    circleData->center, circleData->radius, otherVelocity, dt);
        }
        else
        {
            // For polygons and other capsules, use conservative advancement
            float maxMotion = (velocity.Length() + otherVelocity.Length()) * dt;
            float distance = (capCenter - *static_cast<const Math::Vector2*>(otherData)).Length();
            
            if (distance <= capRadius + 1.0f)  // Approximate other size
            {
                float relativeSpeed = Math::Vector2::Dot(otherVelocity - velocity, 
                                                        (*static_cast<const Math::Vector2*>(otherData) - capCenter).Normalize());
                if (relativeSpeed > 0.0f)
                {
                    float fraction = ConservativeAdvancement(distance - capRadius - 1.0f, relativeSpeed);
                    if (fraction < 1.0f)
                    {
                        result.hit = true;
                        result.fraction = fraction;
                    }
                }
            }
        }
        
        return result;
    }
    
    float ContinuousCollisionDetection::ConservativeAdvancement(
        float distance,
        float relativeSpeed,
        float maxMotion)
    {
        if (relativeSpeed <= 1e-6f || distance <= 1e-6f)
            return 1.0f;
        
        float toi = distance / relativeSpeed;
        return std::min(toi, maxMotion);
    }
    
    bool ContinuousCollisionDetection::NeedsCCD(
        const Math::Vector2& velocity,
        float angularVelocity,
        float minExtent,
        float dt,
        float threshold)
    {
        if (minExtent <= 1e-6f)
            return false;
        
        float linearMotion = velocity.Length() * dt;
        float angularMotion = std::abs(angularVelocity) * dt * minExtent;
        float totalMotion = linearMotion + angularMotion;
        
        return totalMotion > threshold * minExtent;
    }
    
    bool ContinuousCollisionDetection::SolveQuadratic(float a, float b, float c, float& t)
    {
        float discriminant = b * b - 4.0f * a * c;
        if (discriminant < 0.0f)
            return false;
        
        float sqrtDisc = std::sqrt(discriminant);
        float t0 = (-b - sqrtDisc) / (2.0f * a);
        float t1 = (-b + sqrtDisc) / (2.0f * a);
        
        t = (t0 >= 0.0f) ? t0 : t1;
        return true;
    }
    
    Math::Vector2 ContinuousCollisionDetection::ClosestPointOnPolygon(
        const Math::Vector2& point,
        const std::vector<Math::Vector2>& vertices)
    {
        if (vertices.empty())
            return point;
        
        Math::Vector2 closest = vertices[0];
        float minDistSquared = (point - closest).LengthSquared();
        
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            Math::Vector2 v1 = vertices[i];
            Math::Vector2 v2 = vertices[(i + 1) % vertices.size()];
            
            Math::Vector2 edge = v2 - v1;
            Math::Vector2 toPoint = point - v1;
            
            float t = Math::Vector2::Dot(toPoint, edge) / Math::Vector2::Dot(edge, edge);
            t = std::clamp(t, 0.0f, 1.0f);
            
            Math::Vector2 proj = v1 + edge * t;
            float distSquared = (point - proj).LengthSquared();
            
            if (distSquared < minDistSquared)
            {
                minDistSquared = distSquared;
                closest = proj;
            }
        }
        
        return closest;
    }
}
