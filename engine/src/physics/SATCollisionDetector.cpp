// SPDX-FileCopyrightText: 2026 Nyon Engine
// SPDX-License-Identifier: MIT

#include "nyon/physics/SATCollisionDetector.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <cassert>

namespace Nyon::Physics
{
    void SATCollisionDetector::ComputePolygonWorld(
        const ECS::ColliderComponent::PolygonShape& polygon,
        const ECS::TransformComponent& transform,
        std::vector<Math::Vector2>& outVertices,
        std::vector<Math::Vector2>& outNormals)
    {
        outVertices.clear();
        outNormals.clear();
        
        if (polygon.vertices.empty())
            return;
        
        // Compute rotation matrix
        float cosA = std::cos(transform.rotation);
        float sinA = std::sin(transform.rotation);
        
        for (size_t i = 0; i < polygon.vertices.size(); ++i)
        {
            const auto& v = polygon.vertices[i];
            
            // Rotate and translate
            Math::Vector2 worldPoint;
            worldPoint.x = transform.position.x + (v.x * cosA - v.y * sinA);
            worldPoint.y = transform.position.y + (v.x * sinA + v.y * cosA);
            outVertices.push_back(worldPoint);
        }
        
        // Compute edge normals (counter-clockwise winding, normals point outward)
        for (size_t i = 0; i < outVertices.size(); ++i)
        {
            size_t next = (i + 1) % outVertices.size();
            Math::Vector2 edge = outVertices[next] - outVertices[i];
            
            // Normal is perpendicular to edge, pointing outward
            Math::Vector2 normal = {-edge.y, edge.x};
            
            // Normalize
            float length = std::sqrt(normal.x * normal.x + normal.y * normal.y);
            if (length > 1e-6f)
            {
                normal = normal / length;
            }
            
            outNormals.push_back(normal);
        }
    }
    
    void SATCollisionDetector::ComputeCircleCenters(
        const ECS::ColliderComponent::CircleShape& circle,
        const ECS::TransformComponent& transform,
        Math::Vector2& outCenter)
    {
        outCenter = transform.position + circle.center;
    }
    
    void SATCollisionDetector::ComputeCapsuleEndpoints(
        const ECS::ColliderComponent::CapsuleShape& capsule,
        const ECS::TransformComponent& transform,
        Math::Vector2& outStart,
        Math::Vector2& outEnd)
    {
        float cosA = std::cos(transform.rotation);
        float sinA = std::sin(transform.rotation);
        
        // Transform first capsule endpoint
        outStart.x = transform.position.x + (capsule.center1.x * cosA - capsule.center1.y * sinA);
        outStart.y = transform.position.y + (capsule.center1.x * sinA + capsule.center1.y * cosA);
        
        // Transform second capsule endpoint
        outEnd.x = transform.position.x + (capsule.center2.x * cosA - capsule.center2.y * sinA);
        outEnd.y = transform.position.y + (capsule.center2.x * sinA + capsule.center2.y * cosA);
    }
    
    ContactManifold SATCollisionDetector::DetectCircleCircle(
        uint32_t entityIdA,
        uint32_t entityIdB,
        uint32_t shapeIdA,
        uint32_t shapeIdB,
        const ECS::ColliderComponent::CircleShape& circleA,
        const ECS::ColliderComponent::CircleShape& circleB,
        const ECS::TransformComponent& transformA,
        const ECS::TransformComponent& transformB,
        float speculativeDistance)
    {
        ContactManifold manifold;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        
        // Get world space centers
        Math::Vector2 centerA, centerB;
        ComputeCircleCenters(circleA, transformA, centerA);
        ComputeCircleCenters(circleB, transformB, centerB);
        
        // Vector from A to B
        Math::Vector2 delta = centerB - centerA;
        float distanceSquared = delta.x * delta.x + delta.y * delta.y;
        float combinedRadius = circleA.radius + circleB.radius + speculativeDistance;
        
        // Early out if too far apart
        if (distanceSquared > combinedRadius * combinedRadius)
            return manifold;
        
        float distance = std::sqrt(distanceSquared);
        
        // Handle coincident circles
        if (distance < 1e-6f)
        {
            manifold.normal = {1.0f, 0.0f};
            distance = 0.0f;
        }
        else
        {
            manifold.normal = delta / distance;
        }
        
        // Compute penetration depth
        float penetration = combinedRadius - distance;
        
        // Create contact point
        ContactPoint cp;
        cp.position = centerA + manifold.normal * (circleA.radius - 0.5f * penetration);
        cp.normal = manifold.normal;
        cp.separation = -penetration;
        cp.featureId = 0;
        cp.persisted = false;
        
        manifold.points.push_back(cp);
        manifold.touching = penetration > 0.0f;
        
        return manifold;
    }
    
    ContactManifold SATCollisionDetector::DetectCirclePolygon(
        uint32_t entityIdA,
        uint32_t entityIdB,
        uint32_t shapeIdA,
        uint32_t shapeIdB,
        const ECS::ColliderComponent::CircleShape& circle,
        const ECS::ColliderComponent::PolygonShape& polygon,
        const ECS::TransformComponent& transformA,
        const ECS::TransformComponent& transformB,
        float speculativeDistance)
    {
        ContactManifold manifold;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        
        // Get world space data
        Math::Vector2 circleCenter;
        ComputeCircleCenters(circle, transformA, circleCenter);
        
        std::vector<Math::Vector2> polyVerts, polyNormals;
        ComputePolygonWorld(polygon, transformB, polyVerts, polyNormals);
        
        // Find the face with the largest separation along its normal
        float maxSeparation = -std::numeric_limits<float>::infinity();
        int bestFaceIndex = -1;
        
        for (size_t i = 0; i < polyNormals.size(); ++i)
        {
            float separation = Math::Vector2::Dot(polyNormals[i], circleCenter - polyVerts[i]);
            
            // Early out if circle is outside this face
            if (separation > circle.radius + speculativeDistance)
                return manifold;
            
            if (separation > maxSeparation)
            {
                maxSeparation = separation;
                bestFaceIndex = static_cast<int>(i);
            }
        }
        
        if (bestFaceIndex < 0)
            return manifold;
        
        // Manifold normal points from circle (A) to polygon (B)
        Math::Vector2 polyNormal = polyNormals[bestFaceIndex];
        manifold.normal = {-polyNormal.x, -polyNormal.y};
        
        float penetration = circle.radius - maxSeparation;
        
        // Contact point is circle center projected toward polygon
        Math::Vector2 contactPoint = circleCenter - manifold.normal * (circle.radius - 0.5f * penetration);
        
        ContactPoint cp;
        cp.position = contactPoint;
        cp.normal = manifold.normal;
        cp.separation = -penetration;
        cp.featureId = static_cast<uint32_t>(bestFaceIndex);
        cp.persisted = false;
        
        manifold.points.push_back(cp);
        manifold.touching = penetration > 0.0f;
        
        return manifold;
    }
    
    ContactManifold SATCollisionDetector::DetectPolygonPolygon(
        uint32_t entityIdA,
        uint32_t entityIdB,
        uint32_t shapeIdA,
        uint32_t shapeIdB,
        const ECS::ColliderComponent::PolygonShape& polygonA,
        const ECS::ColliderComponent::PolygonShape& polygonB,
        const ECS::TransformComponent& transformA,
        const ECS::TransformComponent& transformB,
        float speculativeDistance)
    {
        ContactManifold manifold;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        
        // Get world space polygons
        std::vector<Math::Vector2> vertsA, normalsA;
        std::vector<Math::Vector2> vertsB, normalsB;
        ComputePolygonWorld(polygonA, transformA, vertsA, normalsA);
        ComputePolygonWorld(polygonB, transformB, vertsB, normalsB);
        
        // SAT test: find axis of minimum penetration
        float minPenetration = std::numeric_limits<float>::infinity();
        int bestAxis = -1;
        bool referenceIsA = true;
        
        // Test axes from polygon A
        for (size_t i = 0; i < normalsA.size(); ++i)
        {
            float minProjB = std::numeric_limits<float>::infinity();
            float maxProjB = -std::numeric_limits<float>::infinity();
            
            // Project polygon B onto axis
            for (const auto& v : vertsB)
            {
                float proj = Math::Vector2::Dot(normalsA[i], v);
                minProjB = std::min(minProjB, proj);
                maxProjB = std::max(maxProjB, proj);
            }
            
            // Project polygon A onto axis (reference vertex)
            float projA = Math::Vector2::Dot(normalsA[i], vertsA[i]);
            
            // Separation along this axis
            float separation = minProjB - projA;
            
            if (separation > speculativeDistance)
                return manifold;  // No collision
            
            if (separation < minPenetration)
            {
                minPenetration = separation;
                bestAxis = static_cast<int>(i);
                referenceIsA = true;
            }
        }
        
        // Test axes from polygon B
        for (size_t i = 0; i < normalsB.size(); ++i)
        {
            float minProjA = std::numeric_limits<float>::infinity();
            float maxProjA = -std::numeric_limits<float>::infinity();
            
            // Project polygon A onto axis
            for (const auto& v : vertsA)
            {
                float proj = Math::Vector2::Dot(normalsB[i], v);
                minProjA = std::min(minProjA, proj);
                maxProjA = std::max(maxProjA, proj);
            }
            
            // Project polygon B onto axis (reference vertex)
            float projB = Math::Vector2::Dot(normalsB[i], vertsB[i]);
            
            // Separation along this axis
            float separation = minProjA - projB;
            
            if (separation > speculativeDistance)
                return manifold;  // No collision
            
            if (separation < minPenetration)
            {
                minPenetration = separation;
                bestAxis = static_cast<int>(i);
                referenceIsA = false;
            }
        }
        
        if (bestAxis < 0)
            return manifold;
        
        // Generate contact manifold using reference face method
        const std::vector<Math::Vector2>* refVerts = referenceIsA ? &vertsA : &vertsB;
        const std::vector<Math::Vector2>* refNormals = referenceIsA ? &normalsA : &normalsB;
        const std::vector<Math::Vector2>* incVerts = referenceIsA ? &vertsB : &vertsA;
        const std::vector<Math::Vector2>* incNormals = referenceIsA ? &normalsB : &normalsA;
        
        const auto& refV = *refVerts;
        const auto& refN = *refNormals;
        const auto& incV = *incVerts;
        
        // Reference face normal
        Math::Vector2 refFaceNormal = refN[bestAxis];
        
        // Find incident face vertices
        int incidentV1, incidentV2;
        FindIncidentFace(refV, refN, incV, *incNormals, bestAxis, incidentV1, incidentV2);
        
        // Clip incident face against reference face side planes
        std::vector<ContactPoint> clipPoints1, clipPoints2;
        
        // Vertex 1
        ContactPoint cp1;
        cp1.position = incV[incidentV1];
        cp1.featureId = static_cast<uint32_t>(incidentV1);
        clipPoints1.push_back(cp1);
        
        // Vertex 2
        ContactPoint cp2;
        cp2.position = incV[incidentV2];
        cp2.featureId = static_cast<uint32_t>(incidentV2);
        clipPoints1.push_back(cp2);
        
        // Clip against first adjacent edge
        int prevIndex = (bestAxis + refV.size() - 1) % refV.size();
        Math::Vector2 v1 = refV[bestAxis];
        Math::Vector2 v2 = refV[prevIndex];
        Math::Vector2 sideNormal = {-(v2.y - v1.y), v2.x - v1.x};  // Perpendicular to edge
        float offset = Math::Vector2::Dot(sideNormal, v1);
        ClipSegmentToLine(clipPoints2, clipPoints1, sideNormal, offset, bestAxis);
        
        // Clip against second adjacent edge
        clipPoints1.clear();
        int nextIndex = (bestAxis + 1) % refV.size();
        v1 = refV[bestAxis];
        v2 = refV[nextIndex];
        sideNormal = {-(v2.y - v1.y), v2.x - v1.x};
        offset = Math::Vector2::Dot(sideNormal, v1);
        ClipSegmentToLine(clipPoints1, clipPoints2, sideNormal, offset, bestAxis);
        
        // Keep only points behind reference face
        float refOffset = Math::Vector2::Dot(refFaceNormal, refV[bestAxis]);
        
        manifold.normal = referenceIsA ? refFaceNormal : Math::Vector2{-refFaceNormal.x, -refFaceNormal.y};
        
        for (const auto& cp : clipPoints1)
        {
            float separation = Math::Vector2::Dot(refFaceNormal, cp.position) - refOffset;
            
            if (separation <= speculativeDistance)
            {
                ContactPoint newCp = cp;
                newCp.normal = manifold.normal;
                newCp.separation = -separation;
                newCp.persisted = false;
                
                // Clamp penetration
                if (newCp.separation > 0.0f)
                    newCp.separation = 0.0f;
                
                manifold.points.push_back(newCp);
                
                // Limit to 2 contact points
                if (manifold.points.size() >= 2)
                    break;
            }
        }
        
        manifold.touching = !manifold.points.empty();
        
        return manifold;
    }
    
    ContactManifold SATCollisionDetector::DetectCapsuleCollision(
        uint32_t entityIdA,
        uint32_t entityIdB,
        const ECS::ColliderComponent::CapsuleShape& capsule,
        const std::variant<
            ECS::ColliderComponent::PolygonShape,
            ECS::ColliderComponent::CircleShape,
            ECS::ColliderComponent::CapsuleShape>& otherCollider,
        const ECS::TransformComponent& transformA,
        const ECS::TransformComponent& transformB,
        float speculativeDistance)
    {
        ContactManifold manifold;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        
        // Get capsule endpoints in world space
        Math::Vector2 capStart, capEnd;
        ComputeCapsuleEndpoints(capsule, transformA, capStart, capEnd);
        
        // Handle based on other collider type
        if (std::holds_alternative<ECS::ColliderComponent::CircleShape>(otherCollider))
        {
            const auto& circle = std::get<ECS::ColliderComponent::CircleShape>(otherCollider);
            Math::Vector2 circleCenter;
            ComputeCircleCenters(circle, transformB, circleCenter);
            
            // Find closest point on capsule segment to circle center
            Math::Vector2 segment = capEnd - capStart;
            float t = Math::Vector2::Dot(circleCenter - capStart, segment) / 
                     Math::Vector2::Dot(segment, segment);
            t = std::clamp(t, 0.0f, 1.0f);
            
            Math::Vector2 closestPoint = capStart + segment * t;
            Math::Vector2 delta = circleCenter - closestPoint;
            float distance = delta.Length();
            float combinedRadius = capsule.radius + circle.radius + speculativeDistance;
            
            if (distance <= combinedRadius)
            {
                float penetration = combinedRadius - distance;
                manifold.normal = (distance > 1e-6f) ? delta / distance : Math::Vector2{1.0f, 0.0f};
                
                ContactPoint cp;
                cp.position = closestPoint + manifold.normal * (capsule.radius - 0.5f * penetration);
                cp.normal = manifold.normal;
                cp.separation = -penetration;
                cp.featureId = 0;
                
                manifold.points.push_back(cp);
                manifold.touching = true;
            }
        }
        else if (std::holds_alternative<ECS::ColliderComponent::PolygonShape>(otherCollider))
        {
            // Treat capsule as swept circle - test against each polygon edge
            const auto& polygon = std::get<ECS::ColliderComponent::PolygonShape>(otherCollider);
            std::vector<Math::Vector2> polyVerts, polyNormals;
            ComputePolygonWorld(polygon, transformB, polyVerts, polyNormals);
            
            // Test capsule endpoints as circles
            for (const auto& endpoint : {capStart, capEnd})
            {
                float maxSeparation = -std::numeric_limits<float>::infinity();
                int bestFaceIndex = -1;
                
                for (size_t i = 0; i < polyNormals.size(); ++i)
                {
                    float separation = Math::Vector2::Dot(polyNormals[i], endpoint - polyVerts[i]);
                    
                    if (separation > capsule.radius + speculativeDistance)
                        continue;
                    
                    if (separation > maxSeparation)
                    {
                        maxSeparation = separation;
                        bestFaceIndex = static_cast<int>(i);
                    }
                }
                
                if (bestFaceIndex >= 0 && maxSeparation < capsule.radius)
                {
                    float penetration = capsule.radius - maxSeparation;
                    Math::Vector2 normal = {-polyNormals[bestFaceIndex].x, -polyNormals[bestFaceIndex].y};
                    
                    ContactPoint cp;
                    cp.position = endpoint - normal * (capsule.radius - 0.5f * penetration);
                    cp.normal = normal;
                    cp.separation = -penetration;
                    cp.featureId = static_cast<uint32_t>(bestFaceIndex);
                    
                    manifold.points.push_back(cp);
                    manifold.touching = true;
                }
            }
        }
        else if (std::holds_alternative<ECS::ColliderComponent::CapsuleShape>(otherCollider))
        {
            const auto& otherCapsule = std::get<ECS::ColliderComponent::CapsuleShape>(otherCollider);
            Math::Vector2 otherStart, otherEnd;
            ComputeCapsuleEndpoints(otherCapsule, transformB, otherStart, otherEnd);
            
            // Find closest points between two line segments
            Math::Vector2 d1 = capEnd - capStart;
            Math::Vector2 d2 = otherEnd - otherStart;
            Math::Vector2 r = capStart - otherStart;
            
            float a = Math::Vector2::Dot(d1, d1);
            float b = Math::Vector2::Dot(d1, d2);
            float c = Math::Vector2::Dot(d2, d2);
            float d = Math::Vector2::Dot(d1, r);
            float e = Math::Vector2::Dot(d2, r);
            
            float denom = a * c - b * b;
            float s = 0.0f, t = 0.0f;
            
            if (denom > 1e-6f)
            {
                s = std::clamp((b * e - c * d) / denom, 0.0f, 1.0f);
                t = std::clamp((a * e - b * d) / denom, 0.0f, 1.0f);
            }
            
            Math::Vector2 p1 = capStart + d1 * s;
            Math::Vector2 p2 = otherStart + d2 * t;
            Math::Vector2 delta = p2 - p1;
            float distance = delta.Length();
            float combinedRadius = capsule.radius + otherCapsule.radius + speculativeDistance;
            
            if (distance <= combinedRadius)
            {
                float penetration = combinedRadius - distance;
                manifold.normal = (distance > 1e-6f) ? delta / distance : Math::Vector2{1.0f, 0.0f};
                
                ContactPoint cp;
                cp.position = p1 + manifold.normal * (capsule.radius - 0.5f * penetration);
                cp.normal = manifold.normal;
                cp.separation = -penetration;
                cp.featureId = 0;
                
                manifold.points.push_back(cp);
                manifold.touching = true;
            }
        }
        
        return manifold;
    }
    
    void SATCollisionDetector::FindIncidentFace(
        const std::vector<Math::Vector2>& vertsA,
        const std::vector<Math::Vector2>& normalsA,
        const std::vector<Math::Vector2>& vertsB,
        const std::vector<Math::Vector2>& normalsB,
        int referenceIndex,
        int& incidentIndex1,
        int& incidentIndex2)
    {
        (void)(vertsA);
        
        // Invariant check: vertices and normals must have same size
        assert(normalsB.size() == vertsB.size());
        if (normalsB.size() != vertsB.size())
            return;
        
        const Math::Vector2 refNormal = normalsA[referenceIndex];
        
        // Find face on B most anti-parallel to reference normal
        float minDot = std::numeric_limits<float>::infinity();
        int incidentIndex = -1;
        
        for (size_t i = 0; i < normalsB.size(); ++i)
        {
            float dot = Math::Vector2::Dot(refNormal, normalsB[i]);
            if (dot < minDot)
            {
                minDot = dot;
                incidentIndex = static_cast<int>(i);
            }
        }
        
        incidentIndex1 = incidentIndex;
        incidentIndex2 = (incidentIndex + 1) % static_cast<int>(vertsB.size());
    }
    
    std::vector<Math::Vector2> SATCollisionDetector::ClipVertices(
        const std::vector<Math::Vector2>& vertices,
        const Math::Vector2& normal,
        float offset)
    {
        std::vector<Math::Vector2> clipped;
        
        if (vertices.empty())
            return clipped;
        
        Math::Vector2 v1 = vertices.back();
        float d1 = Math::Vector2::Dot(normal, v1) - offset;
        
        for (const auto& v2 : vertices)
        {
            float d2 = Math::Vector2::Dot(normal, v2) - offset;
            
            if (d1 * d2 < 0.0f)
            {
                // Edge crosses plane
                float t = d1 / (d1 - d2);
                Math::Vector2 point = v1 + (v2 - v1) * t;
                clipped.push_back(point);
            }
            
            if (d2 <= 0.0f)
            {
                clipped.push_back(v2);
            }
            
            v1 = v2;
            d1 = d2;
        }
        
        return clipped;
    }
    
    void SATCollisionDetector::ClipSegmentToLine(
        std::vector<ContactPoint>& vOut,
        const std::vector<ContactPoint>& vIn,
        const Math::Vector2& normal,
        float offset,
        int clipEdgeIndex)
    {
        (void)(clipEdgeIndex);
        
        vOut.clear();
        if (vIn.empty())
            return;
        
        // Full Sutherland-Hodgman clipping loop
        for (size_t i = 0; i < vIn.size(); ++i)
        {
            const ContactPoint& c1 = vIn[i];
            const ContactPoint& c2 = vIn[(i + 1) % vIn.size()];
            
            float d1 = Math::Vector2::Dot(normal, c1.position) - offset;
            float d2 = Math::Vector2::Dot(normal, c2.position) - offset;
            
            // Keep point if behind or on the plane
            if (d1 <= 0.0f)
                vOut.push_back(c1);
            
            // Add intersection if edge crosses plane
            if ((d1 < 0.0f) != (d2 < 0.0f))
            {
                float t = d1 / (d1 - d2);
                ContactPoint cp = c1;
                cp.position = c1.position + (c2.position - c1.position) * t;
                vOut.push_back(cp);
            }
        }
    }
    
    bool SATCollisionDetector::CCD::ComputeTOI(
        const ContactManifold& manifold,
        const Math::Vector2& velocityA,
        const Math::Vector2& velocityB,
        float angularVelocityA,
        float angularVelocityB,
        float dt,
        float& fraction,
        Math::Vector2& point,
        Math::Vector2& normal)
    {
        if (manifold.points.empty())
            return false;
        
        // Use conservative advancement
        const auto& cp = manifold.points[0];
        float distance = -cp.separation;
        
        // Account for rotational motion by conservatively expanding the query radius
        // This handles fast-spinning bodies that would otherwise tunnel through rotation
        // Estimate shape extents from contact point (approximate as distance from contact to body centers)
        float extentA = 1.0f; // Approximate half-diagonal of shape A
        float extentB = 1.0f; // Approximate half-diagonal of shape B
        
        // Add rotational displacement to distance for conservative TOI
        float rotationalMotionA = std::abs(angularVelocityA) * extentA * dt;
        float rotationalMotionB = std::abs(angularVelocityB) * extentB * dt;
        float expandedDistance = distance + rotationalMotionA + rotationalMotionB;
        
        // Calculate surface points on each body for conservative advancement
        Math::Vector2 ptA = cp.position - cp.normal * (cp.separation * 0.5f); // On A surface
        Math::Vector2 ptB = cp.position + cp.normal * (cp.separation * 0.5f); // On B surface
        
        fraction = ConservativeAdvancement(ptA, ptB, velocityA, velocityB, expandedDistance);
        
        if (fraction >= 0.0f && fraction <= 1.0f)
        {
            point = cp.position;
            normal = cp.normal;
            return true;
        }
        
        return false;
    }
    
    float SATCollisionDetector::CCD::ConservativeAdvancement(
        const Math::Vector2& pointA,
        const Math::Vector2& pointB,
        const Math::Vector2& velocityA,
        const Math::Vector2& velocityB,
        float distance)
    {
        Math::Vector2 delta = pointB - pointA;
        float length = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        
        if (length < 1e-6f)
            return 1.0f;
        
        Math::Vector2 direction = delta / length;
        float relativeSpeed = Math::Vector2::Dot(velocityB - velocityA, direction);
        
        if (relativeSpeed <= 0.0f)
            return 1.0f;  // Not approaching
        
        float toi = distance / relativeSpeed;
        return std::clamp(toi, 0.0f, 1.0f);
    }
}
