#include "nyon/physics/ManifoldGenerator.h"
#include <algorithm>
#include <cmath>

namespace Nyon::Physics
{
    ContactManifold ManifoldGenerator::GenerateCircleCircleManifold(
        const Math::Vector2& centerA, float radiusA,
        const Math::Vector2& centerB, float radiusB,
        uint32_t entityIdA, uint32_t entityIdB,
        uint32_t shapeIdA, uint32_t shapeIdB,
        float frictionA, float frictionB,
        float restitutionA, float restitutionB)
    {
        ContactManifold manifold;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        manifold.friction = CalculateCombinedFriction(frictionA, frictionB);
        manifold.restitution = CalculateCombinedRestitution(restitutionA, restitutionB);
        manifold.GenerateKey();
        
        // Calculate distance and normal
        Math::Vector2 diff = centerB - centerA;
        float distance = diff.Length();
        
        // Check for collision
        float totalRadius = radiusA + radiusB;
        float separation = distance - totalRadius;
        
        if (separation > SPECULATIVE_DISTANCE)
        {
            manifold.touching = false;
            return manifold;
        }
        
        // Generate contact point
        Math::Vector2 normal = (distance > 0.0001f) ? (diff / distance) : Math::Vector2{1.0f, 0.0f};
        Math::Vector2 contactPoint = centerA + normal * (radiusA - separation * 0.5f);
        
        ContactPoint point;
        point.position = contactPoint;
        point.normal = normal;
        point.localPointA = normal * radiusA;
        point.localPointB = normal * (-radiusB);
        point.separation = separation;
        point.featureId = 0;
        
        manifold.points.push_back(point);
        manifold.normal = normal;
        manifold.localNormal = normal;
        manifold.localPoint = point.localPointA;
        manifold.touching = true;
        
        return manifold;
    }
    
    ContactManifold ManifoldGenerator::GenerateCirclePolygonManifold(
        const Math::Vector2& circleCenter, float circleRadius,
        const std::vector<Math::Vector2>& polygonVertices,
        const std::vector<Math::Vector2>& polygonNormals,
        float polygonRadius,
        uint32_t entityIdA, uint32_t entityIdB,
        uint32_t shapeIdA, uint32_t shapeIdB,
        float frictionA, float frictionB,
        float restitutionA, float restitutionB)
    {
        ContactManifold manifold;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        manifold.friction = CalculateCombinedFriction(frictionA, frictionB);
        manifold.restitution = CalculateCombinedRestitution(restitutionA, restitutionB);
        manifold.GenerateKey();
        
        if (polygonVertices.empty())
        {
            manifold.touching = false;
            return manifold;
        }
        
        // Find the closest edge using SAT-like approach
        int bestIndex = 0;
        float maxSeparation = -std::numeric_limits<float>::max();
        
        for (size_t i = 0; i < polygonVertices.size(); ++i)
        {
            float separation = Math::Vector2::Dot(polygonNormals[i], circleCenter - polygonVertices[i]);
            if (separation > maxSeparation)
            {
                maxSeparation = separation;
                bestIndex = static_cast<int>(i);
            }
        }
        
        float totalRadius = circleRadius + polygonRadius;
        if (maxSeparation > totalRadius + SPECULATIVE_DISTANCE)
        {
            manifold.touching = false;
            return manifold;
        }
        
        // Generate contact point
        Math::Vector2 normal = polygonNormals[bestIndex];
        Math::Vector2 contactPoint = circleCenter - normal * (circleRadius - maxSeparation * 0.5f);
        
        ContactPoint point;
        point.position = contactPoint;
        point.normal = normal;
        point.localPointA = normal * circleRadius;
        point.localPointB = contactPoint - polygonVertices[bestIndex]; // Simplified
        point.separation = maxSeparation - totalRadius;
        point.featureId = bestIndex;
        
        manifold.points.push_back(point);
        manifold.normal = normal;
        manifold.localNormal = normal;
        manifold.localPoint = point.localPointA;
        manifold.touching = true;
        
        return manifold;
    }
    
    ContactManifold ManifoldGenerator::GeneratePolygonPolygonManifold(
        const std::vector<Math::Vector2>& verticesA,
        const std::vector<Math::Vector2>& normalsA,
        float radiusA,
        const std::vector<Math::Vector2>& verticesB,
        const std::vector<Math::Vector2>& normalsB,
        float radiusB,
        uint32_t entityIdA, uint32_t entityIdB,
        uint32_t shapeIdA, uint32_t shapeIdB,
        float frictionA, float frictionB,
        float restitutionA, float restitutionB)
    {
        ContactManifold manifold;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        manifold.friction = CalculateCombinedFriction(frictionA, frictionB);
        manifold.restitution = CalculateCombinedRestitution(restitutionA, restitutionB);
        manifold.GenerateKey();
        
        if (verticesA.empty() || verticesB.empty())
        {
            manifold.touching = false;
            return manifold;
        }
        
        // Simplified SAT implementation - find separating axis
        float minSeparation = std::numeric_limits<float>::max();
        Math::Vector2 bestNormal;
        int bestEdge = -1;
        
        // Check normals of polygon A
        for (size_t i = 0; i < normalsA.size(); ++i)
        {
            float separation = -std::numeric_limits<float>::max();
            for (const auto& vertex : verticesB)
            {
                float s = Math::Vector2::Dot(normalsA[i], vertex - verticesA[i]);
                separation = std::max(separation, s);
            }
            
            if (separation < minSeparation)
            {
                minSeparation = separation;
                bestNormal = normalsA[i];
                bestEdge = static_cast<int>(i);
            }
        }
        
        float totalRadius = radiusA + radiusB;
        if (minSeparation > totalRadius + SPECULATIVE_DISTANCE)
        {
            manifold.touching = false;
            return manifold;
        }
        
        // Generate simple contact point (reference implementation)
        Math::Vector2 contactPoint = (verticesA[0] + verticesB[0]) * 0.5f;
        
        ContactPoint point;
        point.position = contactPoint;
        point.normal = bestNormal;
        point.localPointA = contactPoint - verticesA[0];
        point.localPointB = contactPoint - verticesB[0];
        point.separation = minSeparation - totalRadius;
        point.featureId = bestEdge;
        
        manifold.points.push_back(point);
        manifold.normal = bestNormal;
        manifold.localNormal = bestNormal;
        manifold.localPoint = point.localPointA;
        manifold.touching = true;
        
        return manifold;
    }
    
    ContactManifold ManifoldGenerator::GenerateCapsuleManifold(
        const Math::Vector2& capCenter1, const Math::Vector2& capCenter2, float capRadius,
        const Nyon::ECS::ColliderComponent* otherCollider,
        uint32_t entityIdA, uint32_t entityIdB,
        uint32_t shapeIdA, uint32_t shapeIdB,
        float frictionA, float frictionB,
        float restitutionA, float restitutionB)
    {
        // Simplified capsule implementation - treat as circle for now
        Math::Vector2 capsuleCenter = (capCenter1 + capCenter2) * 0.5f;
        return GenerateCircleCircleManifold(
            capsuleCenter, capRadius,
            Math::Vector2{0, 0}, 1.0f, // Placeholder
            entityIdA, entityIdB, shapeIdA, shapeIdB,
            frictionA, frictionB, restitutionA, restitutionB);
    }
    
    void ManifoldGenerator::UpdateManifold(ContactManifold& manifold,
                                         const Math::Vector2& newNormal,
                                         const std::vector<ContactPoint>& newPoints)
    {
        manifold.normal = newNormal;
        manifold.points = newPoints;
        manifold.touching = !newPoints.empty();
    }
    
    void ManifoldGenerator::WarmStartManifold(ContactManifold& manifold,
                                            const ContactManifold& oldManifold)
    {
        // Copy impulses from old manifold to new manifold for warm starting
        for (auto& newPoint : manifold.points)
        {
            for (const auto& oldPoint : oldManifold.points)
            {
                if (newPoint.featureId == oldPoint.featureId)
                {
                    newPoint.normalImpulse = oldPoint.normalImpulse;
                    newPoint.tangentImpulse = oldPoint.tangentImpulse;
                    newPoint.persisted = true;
                    break;
                }
            }
        }
    }
    
    float ManifoldGenerator::CalculateCombinedFriction(float frictionA, float frictionB)
    {
        return std::sqrt(frictionA * frictionB);
    }
    
    float ManifoldGenerator::CalculateCombinedRestitution(float restitutionA, float restitutionB)
    {
        return std::max(restitutionA, restitutionB);
    }
    
    Math::Vector2 ManifoldGenerator::FindBestReferenceEdge(
        const std::vector<Math::Vector2>& vertices,
        const std::vector<Math::Vector2>& normals,
        const Math::Vector2& searchDirection)
    {
        // Find edge with normal most aligned with search direction
        float bestDot = -std::numeric_limits<float>::max();
        Math::Vector2 bestNormal;
        
        for (const auto& normal : normals)
        {
            float dot = Math::Vector2::Dot(normal, searchDirection);
            if (dot > bestDot)
            {
                bestDot = dot;
                bestNormal = normal;
            }
        }
        
        return bestNormal;
    }
    
    std::vector<ContactPoint> ManifoldGenerator::ClipEdges(
        const Math::Vector2& v11, const Math::Vector2& v12,
        const Math::Vector2& v21, const Math::Vector2& v22,
        const Math::Vector2& normal, float totalRadius)
    {
        // Simplified edge clipping - generates up to 2 contact points
        std::vector<ContactPoint> points;
        
        // This would implement the full clipping algorithm
        // For now, generate a simple contact point at midpoint
        Math::Vector2 midpoint = (v11 + v12 + v21 + v22) * 0.25f;
        
        ContactPoint point;
        point.position = midpoint;
        point.normal = normal;
        point.localPointA = midpoint - v11;
        point.localPointB = midpoint - v21;
        point.separation = 0.0f; // Would calculate actual separation
        point.featureId = 0;
        
        points.push_back(point);
        return points;
    }
    
    ContactPoint ManifoldGenerator::CreateContactPoint(
        const Math::Vector2& point, const Math::Vector2& normal,
        const Math::Vector2& localPointA, const Math::Vector2& localPointB,
        float separation, uint32_t featureId)
    {
        ContactPoint cp;
        cp.position = point;
        cp.normal = normal;
        cp.localPointA = localPointA;
        cp.localPointB = localPointB;
        cp.separation = separation;
        cp.featureId = featureId;
        return cp;
    }
}
