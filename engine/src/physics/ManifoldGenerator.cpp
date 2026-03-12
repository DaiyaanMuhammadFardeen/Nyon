#include "nyon/physics/ManifoldGenerator.h"
#include "nyon/physics/SATCollisionDetector.h"

#include <cmath>
#include <limits>
#include <algorithm>

using namespace Nyon::ECS;

namespace Nyon::Physics
{
    namespace
    {
        inline Math::Vector2 Rotate(const Math::Vector2& v, float angle)
        {
            float c = std::cos(angle);
            float s = std::sin(angle);
            return {v.x * c - v.y * s, v.x * s + v.y * c};
        }

        inline float Dot(const Math::Vector2& a, const Math::Vector2& b)
        {
            return a.x * b.x + a.y * b.y;
        }

        inline float CrossScalar(const Math::Vector2& a, const Math::Vector2& b)
        {
            return a.x * b.y - a.y * b.x;
        }

        inline Math::Vector2 Normalize(const Math::Vector2& v)
        {
            float lenSq = v.LengthSquared();
            if (lenSq < 1e-8f)
                return {0.0f, 0.0f};
            float invLen = 1.0f / std::sqrt(lenSq);
            return {v.x * invLen, v.y * invLen};
        }

        inline void ComputeCircleCenters(const ColliderComponent::CircleShape& circle,
                                         const TransformComponent& transform,
                                         Math::Vector2& outCenter)
        {
            outCenter = transform.position + circle.center;
        }

        inline void ComputePolygonWorld(const ColliderComponent::PolygonShape& poly,
                                        const TransformComponent& transform,
                                        std::vector<Math::Vector2>& outVertices,
                                        std::vector<Math::Vector2>& outNormals)
        {
            const size_t count = poly.vertices.size();
            outVertices.resize(count);
            outNormals.resize(poly.normals.size());

            for (size_t i = 0; i < count; ++i)
            {
                outVertices[i] = transform.position + Rotate(poly.vertices[i], transform.rotation);
            }

            for (size_t i = 0; i < poly.normals.size(); ++i)
            {
                outNormals[i] = Rotate(poly.normals[i], transform.rotation);
            }
        }

        // Project polygon vertices onto axis, returning min and max projection.
        inline void ProjectPolygon(const std::vector<Math::Vector2>& vertices,
                                   const Math::Vector2& axis,
                                   float& outMin,
                                   float& outMax)
        {
            outMin = outMax = Dot(vertices[0], axis);
            for (size_t i = 1; i < vertices.size(); ++i)
            {
                float p = Dot(vertices[i], axis);
                if (p < outMin) outMin = p;
                if (p > outMax) outMax = p;
            }
        }

        inline float ProjectPoint(const Math::Vector2& point, const Math::Vector2& axis)
        {
            return Dot(point, axis);
        }
    } // namespace

    ECS::ContactManifold ManifoldGenerator::GenerateManifold(uint32_t entityIdA,
                                                             uint32_t entityIdB,
                                                             uint32_t shapeIdA,
                                                             uint32_t shapeIdB,
                                                             const ColliderComponent& colliderA,
                                                             const ColliderComponent& colliderB,
                                                             const TransformComponent& transformA,
                                                             const TransformComponent& transformB)
    {
        ECS::ContactManifold manifold{};
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        manifold.touching = false;

        using ST = ColliderComponent::ShapeType;
        ST tA = colliderA.GetType();
        ST tB = colliderB.GetType();
        
        // Sort shape types to reduce duplicate collision functions
        bool swapped = false;
        if (static_cast<int>(tA) > static_cast<int>(tB))
        {
            std::swap(tA, tB);
            swapped = true;
        }
        
        // Check for Capsule and Segment shapes FIRST before circle/polygon pairs
        // This prevents capsule/circle from falling into CirclePolygon by mistake
        if (tA == ST::Capsule || tB == ST::Capsule)
        {
            return CapsuleCollision(entityIdA, entityIdB, shapeIdA, shapeIdB,
                                   colliderA, colliderB, transformA, transformB, manifold);
        }
        
        if (tA == ST::Segment || tB == ST::Segment)
        {
            return SegmentCollision(entityIdA, entityIdB, shapeIdA, shapeIdB,
                                   colliderA, colliderB, transformA, transformB, manifold);
        }
        
        // Dispatch to appropriate collision function based on shape type pair
        if (tA == ST::Circle && tB == ST::Circle)
        {
            return CircleCircle(entityIdA, entityIdB, shapeIdA, shapeIdB,
                               colliderA.GetCircle(), colliderB.GetCircle(),
                               transformA, transformB, manifold);
        }
        
        if (tA == ST::Circle && tB == ST::Polygon)
        {
            if (swapped)
            {
                return CirclePolygon(entityIdB, entityIdA, shapeIdB, shapeIdA,
                                    colliderB.GetCircle(), colliderA.GetPolygon(),
                                    transformB, transformA, true, manifold);
            }
            else
            {
                return CirclePolygon(entityIdA, entityIdB, shapeIdA, shapeIdB,
                                    colliderA.GetCircle(), colliderB.GetPolygon(),
                                    transformA, transformB, false, manifold);
            }
        }
        
        if (tA == ST::Polygon && tB == ST::Polygon)
        {
            return PolygonPolygon(entityIdA, entityIdB, shapeIdA, shapeIdB,
                                 colliderA.GetPolygon(), colliderB.GetPolygon(),
                                 transformA, transformB, manifold);
        }

        return manifold;
    }

    ECS::ContactManifold ManifoldGenerator::CircleCircle(uint32_t entityIdA,
                                                         uint32_t entityIdB,
                                                         uint32_t shapeIdA,
                                                         uint32_t shapeIdB,
                                                         const ColliderComponent::CircleShape& circleA,
                                                         const ColliderComponent::CircleShape& circleB,
                                                         const TransformComponent& transformA,
                                                         const TransformComponent& transformB,
                                                         ECS::ContactManifold& manifold)
    {
        manifold.touching = false;

        Math::Vector2 centerA, centerB;
        ComputeCircleCenters(circleA, transformA, centerA);
        ComputeCircleCenters(circleB, transformB, centerB);

        Math::Vector2 delta = centerB - centerA;
        float distSq = delta.LengthSquared();
        float radius = circleA.radius + circleB.radius;

        if (distSq > radius * radius)
        {
            return manifold;
        }

        float dist = std::sqrt(std::max(distSq, 1e-8f));
        Math::Vector2 normal = (dist > 1e-4f) ? Math::Vector2{delta.x / dist, delta.y / dist}
                                              : Math::Vector2{1.0f, 0.0f};

        float penetration = radius - dist;

        // Contact point is on the surface of circle A toward circle B
        ECS::ContactPoint cp{};
        cp.position = centerA + normal * circleA.radius;
        cp.normal = normal;
        cp.separation = -penetration;
        cp.normalImpulse = 0.0f;
        cp.tangentImpulse = 0.0f;
        cp.normalMass = 0.0f;
        cp.tangentMass = 0.0f;
        cp.featureId = 0;
        cp.persisted = false;

        manifold.points.push_back(cp);
        manifold.normal = normal;
        
        // Store local-space data for position correction
        Math::Vector2 invRotA = Rotate(normal, -transformA.rotation);
        manifold.localNormal = invRotA;
        Math::Vector2 localContact = Rotate(cp.position - transformA.position, -transformA.rotation);
        manifold.localPoint = localContact;
        
        manifold.touching = true;

        return manifold;
    }

    ECS::ContactManifold ManifoldGenerator::CirclePolygon(uint32_t entityIdA,
                                                          uint32_t entityIdB,
                                                          uint32_t shapeIdA,
                                                          uint32_t shapeIdB,
                                                          const ColliderComponent::CircleShape& circle,
                                                          const ColliderComponent::PolygonShape& polygon,
                                                          const TransformComponent& circleTransform,
                                                          const TransformComponent& polyTransform,
                                                          bool flipNormal,
                                                          ECS::ContactManifold& manifold)
    {
        manifold.touching = false;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;

        Math::Vector2 center;
        ComputeCircleCenters(circle, circleTransform, center);

        std::vector<Math::Vector2> verts;
        std::vector<Math::Vector2> normals;
        ComputePolygonWorld(polygon, polyTransform, verts, normals);

        // Find the face with the largest separation along its normal.
        float maxSeparation = -std::numeric_limits<float>::infinity();
        int bestIndex = -1;

        for (size_t i = 0; i < normals.size(); ++i)
        {
            float s = Dot(normals[i], center - verts[i]);
            if (s > circle.radius)
            {
                // No collision if circle center is outside this face by more than radius.
                return manifold;
            }
            if (s > maxSeparation)
            {
                maxSeparation = s;
                bestIndex = static_cast<int>(i);
            }
        }

        if (bestIndex < 0)
        {
            return manifold;
        }
        
        // Polygon face normals point outward from the polygon. In this helper the
        // circle is always entity A and the polygon is entity B, so the manifold
        // normal should point from A → B (circle → polygon). Flip the polygon
        // normal to enforce that convention.
        Math::Vector2 polyNormal = normals[bestIndex];
        Math::Vector2 normal = {-polyNormal.x, -polyNormal.y};
        
        float penetration = circle.radius - maxSeparation;
        
        // Contact point is the circle center projected back towards the polygon
        // along the opposite of the manifold normal.
        Math::Vector2 contactPoint = center - normal * (circle.radius - 0.5f * penetration);
        
        ECS::ContactPoint cp{};
        cp.position = contactPoint;
        cp.normal = normal;
        cp.separation = -penetration;
        cp.normalImpulse = 0.0f;
        cp.tangentImpulse = 0.0f;
        cp.normalMass = 0.0f;
        cp.tangentMass = 0.0f;
        cp.featureId = static_cast<uint32_t>(bestIndex);
        cp.persisted = false;

        manifold.points.push_back(cp);
        manifold.normal = normal;
        
        // Store local-space data for position correction
        Math::Vector2 invRotA = Rotate(normal, -circleTransform.rotation);
        manifold.localNormal = invRotA;
        Math::Vector2 localContact = Rotate(contactPoint - circleTransform.position, -circleTransform.rotation);
        manifold.localPoint = localContact;
        
        manifold.touching = true;

        return manifold;
    }

    ECS::ContactManifold ManifoldGenerator::PolygonPolygon(uint32_t entityIdA,
                                                           uint32_t entityIdB,
                                                           uint32_t shapeIdA,
                                                           uint32_t shapeIdB,
                                                           const ColliderComponent::PolygonShape& polyA,
                                                           const ColliderComponent::PolygonShape& polyB,
                                                           const TransformComponent& transformA,
                                                           const TransformComponent& transformB,
                                                           ECS::ContactManifold& manifold)
    {
        // Delegate to SATCollisionDetector for proper Sutherland-Hodgman clipping
        // This generates up to 2 contact points for stable stacking
        SATCollisionDetector detector;
        Physics::ContactManifold result = detector.DetectPolygonPolygon(
            entityIdA, entityIdB, shapeIdA, shapeIdB,
            polyA, polyB, transformA, transformB);
        return ConvertManifold(result);
    }
    
    ECS::ContactManifold ManifoldGenerator::CircleCapsule(uint32_t entityIdA,
                                                          uint32_t entityIdB,
                                                          uint32_t shapeIdA,
                                                          uint32_t shapeIdB,
                                                          const ColliderComponent& circleCollider,
                                                          const ColliderComponent& capsuleCollider,
                                                          const TransformComponent& transformA,
                                                          const TransformComponent& transformB,
                                                          ECS::ContactManifold& manifold)
    {
        // Circle vs Capsule: treat capsule as line segment with radius
        // Find closest point on capsule center line to circle center
        manifold.touching = false;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        
        // Get circle data
        const auto& circle = circleCollider.GetCircle();
        Math::Vector2 circleCenter = transformA.position + circle.center;
        
        // Get capsule endpoints in world space
        const auto& capsule = capsuleCollider.GetCapsule();
        float cosB = std::cos(transformB.rotation);
        float sinB = std::sin(transformB.rotation);
        Math::Vector2 capStart = {
            transformB.position.x + (capsule.center1.x * cosB - capsule.center1.y * sinB),
            transformB.position.y + (capsule.center1.x * sinB + capsule.center1.y * cosB)
        };
        Math::Vector2 capEnd = {
            transformB.position.x + (capsule.center2.x * cosB - capsule.center2.y * sinB),
            transformB.position.y + (capsule.center2.x * sinB + capsule.center2.y * cosB)
        };
        
        // Find closest point on capsule center line to circle center
        Math::Vector2 segDir = capEnd - capStart;
        float segLenSq = segDir.LengthSquared();
        float t = Dot(circleCenter - capStart, segDir) / segLenSq;
        t = std::clamp(t, 0.0f, 1.0f);
        Math::Vector2 closestPoint = capStart + segDir * t;
        
        // Distance from circle center to closest point
        Math::Vector2 delta = circleCenter - closestPoint;
        float dist = delta.Length();
        float combinedRadius = circle.radius + capsule.radius;
        
        if (dist > combinedRadius)
            return manifold;
        
        float penetration = combinedRadius - dist;
        Math::Vector2 normal = (dist > 1e-4f) ? Normalize(delta) : Math::Vector2{1.0f, 0.0f};
        
        // Contact point on circle surface
        Math::Vector2 contactPoint = circleCenter - normal * circle.radius;
        
        ECS::ContactPoint cp{};
        cp.position = contactPoint;
        cp.normal = normal;
        cp.separation = -penetration;
        cp.normalImpulse = 0.0f;
        cp.tangentImpulse = 0.0f;
        cp.normalMass = 0.0f;
        cp.tangentMass = 0.0f;
        cp.featureId = 0;
        cp.persisted = false;
        
        manifold.points.push_back(cp);
        manifold.normal = normal;
        
        // Store local-space data
        Math::Vector2 invRotA = Rotate(normal, -transformA.rotation);
        manifold.localNormal = invRotA;
        Math::Vector2 localContact = Rotate(contactPoint - transformA.position, -transformA.rotation);
        manifold.localPoint = localContact;
        
        manifold.touching = true;
        
        return manifold;
    }
    
    ECS::ContactManifold ManifoldGenerator::PolygonCapsule(uint32_t entityIdA,
                                                           uint32_t entityIdB,
                                                           uint32_t shapeIdA,
                                                           uint32_t shapeIdB,
                                                           const ColliderComponent& polyCollider,
                                                           const ColliderComponent& capsuleCollider,
                                                           const TransformComponent& transformA,
                                                           const TransformComponent& transformB,
                                                           ECS::ContactManifold& manifold)
    {
        // Polygon vs Capsule: test polygon edges against capsule line
        manifold.touching = false;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        
        // For now, approximate capsule as circle at its center
        // TODO: Implement full polygon-capsule SAT test
        const auto& capsule = capsuleCollider.GetCapsule();
        Math::Vector2 capCenter = transformB.position + (capsule.center1 + capsule.center2) * 0.5f;
        
        // Create temporary circle for approximation
        ColliderComponent::CircleShape tempCircle{{0.0f, 0.0f}, capsule.radius};
        TransformComponent tempTransform;
        tempTransform.position = capCenter;
        tempTransform.rotation = 0.0f;
        
        return CirclePolygon(entityIdB, entityIdA, shapeIdB, shapeIdA,
                            tempCircle, polyCollider.GetPolygon(),
                            tempTransform, transformA, true, manifold);
    }
    
    ECS::ContactManifold ManifoldGenerator::CapsuleCapsule(uint32_t entityIdA,
                                                           uint32_t entityIdB,
                                                           uint32_t shapeIdA,
                                                           uint32_t shapeIdB,
                                                           const ColliderComponent& colliderA,
                                                           const ColliderComponent& colliderB,
                                                           const TransformComponent& transformA,
                                                           const TransformComponent& transformB,
                                                           ECS::ContactManifold& manifold)
    {
        // Capsule vs Capsule: find closest points on both center lines
        manifold.touching = false;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        
        // Get capsule endpoints in world space for both capsules
        const auto& capA = colliderA.GetCapsule();
        const auto& capB = colliderB.GetCapsule();
        
        // Transform capsule A endpoints
        float cosA = std::cos(transformA.rotation);
        float sinA = std::sin(transformA.rotation);
        Math::Vector2 a1 = {
            transformA.position.x + (capA.center1.x * cosA - capA.center1.y * sinA),
            transformA.position.y + (capA.center1.x * sinA + capA.center1.y * cosA)
        };
        Math::Vector2 a2 = {
            transformA.position.x + (capA.center2.x * cosA - capA.center2.y * sinA),
            transformA.position.y + (capA.center2.x * sinA + capA.center2.y * cosA)
        };
        
        // Transform capsule B endpoints
        float cosB = std::cos(transformB.rotation);
        float sinB = std::sin(transformB.rotation);
        Math::Vector2 b1 = {
            transformB.position.x + (capB.center1.x * cosB - capB.center1.y * sinB),
            transformB.position.y + (capB.center1.x * sinB + capB.center1.y * cosB)
        };
        Math::Vector2 b2 = {
            transformB.position.x + (capB.center2.x * cosB - capB.center2.y * sinB),
            transformB.position.y + (capB.center2.x * sinB + capB.center2.y * cosB)
        };
        
        // Find closest points on two line segments
        Math::Vector2 d1 = a2 - a1; // Direction of segment A
        Math::Vector2 d2 = b2 - b1; // Direction of segment B
        Math::Vector2 r = a1 - b1;
        
        float a = Dot(d1, d1);
        float b = Dot(d1, d2);
        float c = Dot(d2, d2);
        float d = Dot(d1, r);
        float e = Dot(d2, r);
        
        float denom = a * c - b * b;
        float s, t;
        
        if (denom < 1e-8f)
        {
            // Segments are parallel
            s = 0.0f;
            t = 0.0f;
        }
        else
        {
            s = std::clamp((b * e - c * d) / denom, 0.0f, 1.0f);
            t = std::clamp((a * e - b * d) / denom, 0.0f, 1.0f);
        }
        
        Math::Vector2 closestA = a1 + d1 * s;
        Math::Vector2 closestB = b1 + d2 * t;
        
        Math::Vector2 delta = closestB - closestA;
        float dist = delta.Length();
        float combinedRadius = capA.radius + capB.radius;
        
        if (dist > combinedRadius)
            return manifold;
        
        float penetration = combinedRadius - dist;
        Math::Vector2 normal = (dist > 1e-4f) ? Normalize(delta) : Math::Vector2{1.0f, 0.0f};
        
        // Contact point at midpoint
        Math::Vector2 contactPoint = closestA + normal * capA.radius;
        
        ECS::ContactPoint cp{};
        cp.position = contactPoint;
        cp.normal = normal;
        cp.separation = -penetration;
        cp.normalImpulse = 0.0f;
        cp.tangentImpulse = 0.0f;
        cp.normalMass = 0.0f;
        cp.tangentMass = 0.0f;
        cp.featureId = 0;
        cp.persisted = false;
        
        manifold.points.push_back(cp);
        manifold.normal = normal;
        
        // Store local-space data
        Math::Vector2 invRotA = Rotate(normal, -transformA.rotation);
        manifold.localNormal = invRotA;
        Math::Vector2 localContact = Rotate(contactPoint - transformA.position, -transformA.rotation);
        manifold.localPoint = localContact;
        
        manifold.touching = true;
        
        return manifold;
    }
    
    ECS::ContactManifold ManifoldGenerator::CapsuleCollision(uint32_t entityIdA,
                                                             uint32_t entityIdB,
                                                             uint32_t shapeIdA,
                                                             uint32_t shapeIdB,
                                                             const ColliderComponent& colliderA,
                                                             const ColliderComponent& colliderB,
                                                             const TransformComponent& transformA,
                                                             const TransformComponent& transformB,
                                                             ECS::ContactManifold& manifold)
    {
        // Delegate to SATCollisionDetector for proper capsule collision detection
        SATCollisionDetector detector;
        
        const auto& capsuleA = colliderA.GetCapsule();
        
        // Build variant for other shape - must match SATCollisionDetector signature
        std::variant<ColliderComponent::PolygonShape,
                     ColliderComponent::CircleShape,
                     ColliderComponent::CapsuleShape> otherShape;
        
        using ST = ColliderComponent::ShapeType;
        ST tB = colliderB.GetType();
        
        if (tB == ST::Polygon)
            otherShape = colliderB.GetPolygon();
        else if (tB == ST::Circle)
            otherShape = colliderB.GetCircle();
        else if (tB == ST::Capsule)
            otherShape = colliderB.GetCapsule();
        else
        {
            // For Segment shapes, we'll handle them separately or skip
            // For now, return empty manifold
            return manifold;
        }
        
        Physics::ContactManifold result = detector.DetectCapsuleCollision(
            entityIdA, entityIdB,
            capsuleA, otherShape, transformA, transformB);
        return ConvertManifold(result);
    }
    
    ECS::ContactManifold ManifoldGenerator::SegmentCollision(uint32_t entityIdA,
                                                             uint32_t entityIdB,
                                                             uint32_t shapeIdA,
                                                             uint32_t shapeIdB,
                                                             const ColliderComponent& colliderA,
                                                             const ColliderComponent& colliderB,
                                                             const TransformComponent& transformA,
                                                             const TransformComponent& transformB,
                                                             ECS::ContactManifold& manifold)
    {
        // Segments are zero-thickness edges (or thin if radius > 0)
        // Segment collisions involve:
        // 1. Point-line distance tests for endpoints
        // 2. Line-line intersection for crossing segments
        // 3. Special handling for endpoint contacts
        
        manifold.touching = false;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        
        // TODO: Implement full segment collision detection
        // Segments typically collide with circles and polygons
        // Segment-segment collision is rare and can be handled as degenerate polygon
        
        return manifold;
    }
    
    ECS::ContactManifold ManifoldGenerator::ConvertManifold(const Physics::ContactManifold& physicsManifold)
    {
        ECS::ContactManifold ecsManifold;
        ecsManifold.entityIdA = physicsManifold.entityIdA;
        ecsManifold.entityIdB = physicsManifold.entityIdB;
        ecsManifold.shapeIdA = physicsManifold.shapeIdA;
        ecsManifold.shapeIdB = physicsManifold.shapeIdB;
        ecsManifold.normal = physicsManifold.normal;
        ecsManifold.localNormal = physicsManifold.localNormal;
        ecsManifold.localPoint = physicsManifold.localPoint;
        ecsManifold.touching = physicsManifold.touching;
        
        // Convert contact points
        for (const auto& physicsPoint : physicsManifold.points)
        {
            ECS::ContactPoint ecsPoint;
            ecsPoint.position = physicsPoint.position;
            ecsPoint.normal = physicsPoint.normal;
            ecsPoint.separation = physicsPoint.separation;
            ecsPoint.normalImpulse = physicsPoint.normalImpulse;
            ecsPoint.tangentImpulse = physicsPoint.tangentImpulse;
            ecsPoint.normalMass = physicsPoint.normalMass;
            ecsPoint.tangentMass = physicsPoint.tangentMass;
            ecsPoint.featureId = physicsPoint.featureId;
            ecsPoint.persisted = physicsPoint.persisted;
            
            ecsManifold.points.push_back(ecsPoint);
        }
        
        return ecsManifold;
    }
}

