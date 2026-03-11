#include "nyon/physics/ManifoldGenerator.h"

#include <cmath>
#include <limits>
#include <algorithm>
#include <fstream>
#include <chrono>

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

    ContactManifold ManifoldGenerator::GenerateManifold(uint32_t entityIdA,
                                                        uint32_t entityIdB,
                                                        uint32_t shapeIdA,
                                                        uint32_t shapeIdB,
                                                        const ColliderComponent& colliderA,
                                                        const ColliderComponent& colliderB,
                                                        const TransformComponent& transformA,
                                                        const TransformComponent& transformB)
    {
        ContactManifold manifold{};
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        manifold.touching = false;
        manifold.tangentSpeed = 0.0f;

        // Compute combined material properties using Box2D-style mixing
        // Friction uses geometric mean: sqrt(frictionA * frictionB)
        // Restitution uses maximum: max(restitutionA, restitutionB)
        manifold.friction = std::sqrt(colliderA.material.friction * colliderB.material.friction);
        manifold.restitution = std::max(colliderA.material.restitution, colliderB.material.restitution);

        if (colliderA.GetType() == ColliderComponent::ShapeType::Circle &&
            colliderB.GetType() == ColliderComponent::ShapeType::Circle)
        {
            return CircleCircle(entityIdA, entityIdB, shapeIdA, shapeIdB,
                                colliderA.GetCircle(), colliderB.GetCircle(),
                                transformA, transformB, manifold);
        }

        if (colliderA.GetType() == ColliderComponent::ShapeType::Circle &&
            colliderB.GetType() == ColliderComponent::ShapeType::Polygon)
        {
            return CirclePolygon(entityIdA, entityIdB, shapeIdA, shapeIdB,
                                 colliderA.GetCircle(), colliderB.GetPolygon(),
                                 transformA, transformB, false, manifold);
        }

        if (colliderA.GetType() == ColliderComponent::ShapeType::Polygon &&
            colliderB.GetType() == ColliderComponent::ShapeType::Circle)
        {
            // Flip normal so it always points from A to B.
            return CirclePolygon(entityIdB, entityIdA, shapeIdB, shapeIdA,
                                 colliderB.GetCircle(), colliderA.GetPolygon(),
                                 transformB, transformA, true, manifold);
        }

        if (colliderA.GetType() == ColliderComponent::ShapeType::Polygon &&
            colliderB.GetType() == ColliderComponent::ShapeType::Polygon)
        {
            return PolygonPolygon(entityIdA, entityIdB, shapeIdA, shapeIdB,
                                  colliderA.GetPolygon(), colliderB.GetPolygon(),
                                  transformA, transformB, manifold);
        }

        // Capsule collisions
        if (colliderA.GetType() == ColliderComponent::ShapeType::Capsule ||
            colliderB.GetType() == ColliderComponent::ShapeType::Capsule)
        {
            return CapsuleCollision(entityIdA, entityIdB, shapeIdA, shapeIdB,
                                   colliderA, colliderB, transformA, transformB, manifold);
        }

        // Segment collisions
        if (colliderA.GetType() == ColliderComponent::ShapeType::Segment ||
            colliderB.GetType() == ColliderComponent::ShapeType::Segment)
        {
            return SegmentCollision(entityIdA, entityIdB, shapeIdA, shapeIdB,
                                   colliderA, colliderB, transformA, transformB, manifold);
        }

        return manifold;
    }

    ContactManifold ManifoldGenerator::CircleCircle(uint32_t entityIdA,
                                                    uint32_t entityIdB,
                                                    uint32_t shapeIdA,
                                                    uint32_t shapeIdB,
                                                    const ColliderComponent::CircleShape& circleA,
                                                    const ColliderComponent::CircleShape& circleB,
                                                    const TransformComponent& transformA,
                                                    const TransformComponent& transformB,
                                                    ContactManifold& manifold)
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

        ContactPoint cp{};
        cp.position = centerA + normal * (circleA.radius - 0.5f * penetration);
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
        manifold.localNormal = {0.0f, 0.0f};
        manifold.localPoint = {0.0f, 0.0f};
        manifold.touching = true;

        return manifold;
    }

    ContactManifold ManifoldGenerator::CirclePolygon(uint32_t entityIdA,
                                                     uint32_t entityIdB,
                                                     uint32_t shapeIdA,
                                                     uint32_t shapeIdB,
                                                     const ColliderComponent::CircleShape& circle,
                                                     const ColliderComponent::PolygonShape& polygon,
                                                     const TransformComponent& circleTransform,
                                                     const TransformComponent& polyTransform,
                                                     bool flipNormal,
                                                     ContactManifold& manifold)
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
        
        ContactPoint cp{};
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
        manifold.localNormal = {0.0f, 0.0f};
        manifold.localPoint = {0.0f, 0.0f};
        manifold.touching = true;

        return manifold;
    }

    ContactManifold ManifoldGenerator::PolygonPolygon(uint32_t entityIdA,
                                                      uint32_t entityIdB,
                                                      uint32_t shapeIdA,
                                                      uint32_t shapeIdB,
                                                      const ColliderComponent::PolygonShape& polyA,
                                                      const ColliderComponent::PolygonShape& polyB,
                                                      const TransformComponent& transformA,
                                                      const TransformComponent& transformB,
                                                      ContactManifold& manifold)
    {
        manifold.touching = false;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;

        std::vector<Math::Vector2> vertsA, vertsB;
        std::vector<Math::Vector2> normalsA, normalsB;
        ComputePolygonWorld(polyA, transformA, vertsA, normalsA);
        ComputePolygonWorld(polyB, transformB, vertsB, normalsB);

        float minOverlap = std::numeric_limits<float>::infinity();
        Math::Vector2 bestAxis = {0.0f, 0.0f};

        auto testAxes = [&](const std::vector<Math::Vector2>& vertices1,
                            const std::vector<Math::Vector2>& vertices2,
                            const std::vector<Math::Vector2>& axes,
                            bool flip) -> bool
        {
            for (const auto& axisRaw : axes)
            {
                Math::Vector2 axis = Normalize(axisRaw);
                if (axis.LengthSquared() < 1e-6f)
                    continue;

                float min1, max1, min2, max2;
                ProjectPolygon(vertices1, axis, min1, max1);
                ProjectPolygon(vertices2, axis, min2, max2);

                float overlap = std::min(max1, max2) - std::max(min1, min2);
                if (overlap <= 0.0f)
                {
                    // Separating axis found – no collision.
                    return false;
                }

                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    bestAxis = flip ? Math::Vector2{-axis.x, -axis.y} : axis;
                }
            }
            return true;
        };

        if (!testAxes(vertsA, vertsB, normalsA, false))
            return manifold;
        if (!testAxes(vertsB, vertsA, normalsB, true))
            return manifold;

        // Collision – build a simple manifold with a single contact point at the midpoints of support features.
        // Find support points by projecting vertices onto the best separating axis.
        float bestProjA = -std::numeric_limits<float>::infinity();
        Math::Vector2 supportA;
        for (const auto& v : vertsA)
        {
            float p = Dot(v, bestAxis);
            if (p > bestProjA)
            {
                bestProjA = p;
                supportA = v;
            }
        }

        float bestProjB = std::numeric_limits<float>::infinity();
        Math::Vector2 supportB;
        for (const auto& v : vertsB)
        {
            float p = Dot(v, bestAxis);
            if (p < bestProjB)
            {
                bestProjB = p;
                supportB = v;
            }
        }

        Math::Vector2 contactPoint = (supportA + supportB) * 0.5f;

        ContactPoint cp{};
        cp.position = contactPoint;
        cp.normal = bestAxis;
        cp.separation = -minOverlap;
        cp.normalImpulse = 0.0f;
        cp.tangentImpulse = 0.0f;
        cp.normalMass = 0.0f;
        cp.tangentMass = 0.0f;
        cp.featureId = 0;
        cp.persisted = false;

        manifold.points.push_back(cp);
        manifold.normal = bestAxis;
        manifold.localNormal = {0.0f, 0.0f};
        manifold.localPoint = {0.0f, 0.0f};
        manifold.touching = true;

        return manifold;
    }
    
    ContactManifold ManifoldGenerator::CapsuleCollision(uint32_t entityIdA,
                                                        uint32_t entityIdB,
                                                        uint32_t shapeIdA,
                                                        uint32_t shapeIdB,
                                                        const ColliderComponent& colliderA,
                                                        const ColliderComponent& colliderB,
                                                        const TransformComponent& transformA,
                                                        const TransformComponent& transformB,
                                                        ContactManifold& manifold)
    {
        // Capsules are approximated as line segments with radius
        // For now, delegate to circle or segment collision based on other shape
        // Full capsule-capsule collision would involve:
        // 1. Finding closest points on both capsule center lines
        // 2. Treating as sphere-sphere collision with those centers
        // 3. Using combined radii for collision test
        
        manifold.touching = false;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        
        // TODO: Implement full capsule collision detection
        // This requires finding the closest approach between two line segments
        // and treating it as a circle-circle collision at that point
        
        return manifold;
    }
    
    ContactManifold ManifoldGenerator::SegmentCollision(uint32_t entityIdA,
                                                        uint32_t entityIdB,
                                                        uint32_t shapeIdA,
                                                        uint32_t shapeIdB,
                                                        const ColliderComponent& colliderA,
                                                        const ColliderComponent& colliderB,
                                                        const TransformComponent& transformA,
                                                        const TransformComponent& transformB,
                                                        ContactManifold& manifold)
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
}

