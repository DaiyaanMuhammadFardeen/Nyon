#include "nyon/physics/ManifoldGenerator.h"

#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>

// Collision detection debug logging (only when needed)
#ifdef _DEBUG
#define COLLISION_DEBUG_LOG(x) std::cerr << "[COLLISION] " << x << std::endl
#else
#define COLLISION_DEBUG_LOG(x)
#endif

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
            if (lenSq < 1e-8f) return {1.0f, 0.0f};
            float inv = 1.0f / std::sqrt(lenSq);
            return {v.x * inv, v.y * inv};
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
        
        // Find incident face on polygon that is most anti-parallel to reference normal
        inline int FindIncidentFace(const std::vector<Math::Vector2>& normals,
                                   const Math::Vector2& referenceNormal)
        {
            int incidentFace = 0;
            float maxDot = -std::numeric_limits<float>::infinity();
            
            for (size_t i = 0; i < normals.size(); ++i)
            {
                float dot = Dot(normals[i], referenceNormal);
                if (dot > maxDot)
                {
                    maxDot = dot;
                    incidentFace = static_cast<int>(i);
                }
            }
            
            return incidentFace;
        }
        
        // Clip segment to line using Sutherland-Hodgman algorithm
        inline void ClipSegmentToLine(std::vector<ECS::ContactPoint>& outContacts,
                                      const std::vector<Math::Vector2>& refVerts,
                                      const std::vector<Math::Vector2>& incVerts,
                                      int refFaceIndex,
                                      int incFaceIndex,
                                      const Math::Vector2& normal,
                                      float /*separation*/)
        {
            COLLISION_DEBUG_LOG("    [Clip] Ref face " << refFaceIndex << " | Inc face " << incFaceIndex);

            size_t nRef = refVerts.size();
            Math::Vector2 v1 = refVerts[refFaceIndex];
            Math::Vector2 v2 = refVerts[(refFaceIndex + 1) % nRef];

            size_t nInc = incVerts.size();
            Math::Vector2 iv1 = incVerts[incFaceIndex];
            Math::Vector2 iv2 = incVerts[(incFaceIndex + 1) % nInc];

            // --- Side 1 clip (perpendicular to reference edge) ---
            Math::Vector2 side1 = v2 - v1;
            side1 = {-side1.y, side1.x};               // inward perpendicular
            float offset1 = Dot(side1, v1);

            float d1 = Dot(side1, iv1) - offset1;
            float d2 = Dot(side1, iv2) - offset1;

            std::vector<Math::Vector2> clip;
            if (d1 <= 0.0f) clip.push_back(iv1);
            if (d1 * d2 < 0.0f && std::abs(d2 - d1) > 1e-6f)
            {
                float t = d1 / (d1 - d2);
                clip.push_back(iv1 + (iv2 - iv1) * t);
            }
            if (d2 <= 0.0f) clip.push_back(iv2);

            // --- Side 2 clip (opposite side of reference edge) ---
            if (clip.empty())
            {
                COLLISION_DEBUG_LOG("      Clipped away by side planes");
                return;
            }

            std::vector<Math::Vector2> finalClip;
            Math::Vector2 side2 = v1 - v2;
            side2 = {-side2.y, side2.x};
            float offset2 = Dot(side2, v2);

            if (clip.size() == 1)
            {
                float d = Dot(side2, clip[0]) - offset2;
                if (d <= 0.0f) finalClip.push_back(clip[0]);
            }
            else // 2 points = clipped edge
            {
                float dA = Dot(side2, clip[0]) - offset2;
                float dB = Dot(side2, clip[1]) - offset2;

                if (dA <= 0.0f) finalClip.push_back(clip[0]);
                if (dA * dB < 0.0f && std::abs(dB - dA) > 1e-6f)
                {
                    float t = dA / (dA - dB);
                    finalClip.push_back(clip[0] + (clip[1] - clip[0]) * t);
                }
                if (dB <= 0.0f) finalClip.push_back(clip[1]);
            }

            // --- Final clip against reference face plane ---
            float refOffset = Dot(normal, v1);
            for (const auto& pt : finalClip)
            {
                float sep = Dot(normal, pt) - refOffset;
                if (sep <= 0.0f)                     // penetration or touching
                {
                    ECS::ContactPoint cp{};
                    cp.position     = pt;
                    cp.normal       = normal;
                    cp.separation   = sep;
                    cp.normalImpulse = 0.0f;
                    cp.tangentImpulse = 0.0f;
                    cp.normalMass   = 0.0f;
                    cp.tangentMass  = 0.0f;
                    cp.persisted    = false;
                    outContacts.push_back(cp);

                    COLLISION_DEBUG_LOG("        Contact @ (" << pt.x << "," << pt.y << ") sep=" << sep);
                }
            }
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

        COLLISION_DEBUG_LOG("GenerateManifold: entityA=" << entityIdA << " entityB=" << entityIdB);

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
            COLLISION_DEBUG_LOG("  -> Circle-Circle collision");
            return CircleCircle(entityIdA, entityIdB, shapeIdA, shapeIdB,
                               colliderA.GetCircle(), colliderB.GetCircle(),
                               transformA, transformB, manifold);
        }
        
        if (tA == ST::Circle && tB == ST::Polygon)
        {
            COLLISION_DEBUG_LOG("  -> Circle-Polygon collision");
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
            COLLISION_DEBUG_LOG("  -> Polygon-Polygon collision");
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
        
        // If flipNormal is true, the entities were swapped during dispatch.
        // We need to flip the normal back to match the original entity ordering.
        if (flipNormal)
        {
            normal = -normal;
        }
        
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
        
        // If flipNormal is true, we also need to flip local normal and contact point normals
        if (flipNormal)
        {
            manifold.localNormal = -manifold.localNormal;
            for (auto& cp : manifold.points)
            {
                cp.normal = -cp.normal;
            }
        }
        
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
        // Implement SAT-based polygon collision detection directly
        manifold.touching = false;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        
        COLLISION_DEBUG_LOG("  [PolygonPolygon] Testing collision");
        
        // Get world-space vertices and normals for both polygons
        std::vector<Math::Vector2> vertsA, normalsA;
        std::vector<Math::Vector2> vertsB, normalsB;
        ComputePolygonWorld(polyA, transformA, vertsA, normalsA);
        ComputePolygonWorld(polyB, transformB, vertsB, normalsB);
        
        float minOverlap = std::numeric_limits<float>::infinity();
        Math::Vector2 separatingAxis;
        int referenceFace = -1;
        
        // Test axes from polygon A
        for (size_t i = 0; i < normalsA.size(); ++i)
        {
            Math::Vector2 axis = normalsA[i];
            float minA, maxA, minB, maxB;
            ProjectPolygon(vertsA, axis, minA, maxA);
            ProjectPolygon(vertsB, axis, minB, maxB);
            
            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < 0) return manifold; // Separating axis found
            
            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                separatingAxis = axis;
                referenceFace = static_cast<int>(i);
            }
        }
        
        // Test axes from polygon B
        for (size_t i = 0; i < normalsB.size(); ++i)
        {
            Math::Vector2 axis = normalsB[i];
            float minA, maxA, minB, maxB;
            ProjectPolygon(vertsA, axis, minA, maxA);
            ProjectPolygon(vertsB, axis, minB, maxB);
            
            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < 0) return manifold; // Separating axis found
            
            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                separatingAxis = axis;
                referenceFace = static_cast<int>(i) + static_cast<int>(normalsA.size());  // Fixed: use normalsA.size() instead of vertsA.size()
            }
        }
        
        // Collision detected - generate contact manifold
        COLLISION_DEBUG_LOG("  [PolygonPolygon] Collision detected! Min overlap=" << minOverlap);
        manifold.normal = separatingAxis;
        manifold.points.clear();
        
        // Find incident face on polygon B relative to reference face on A
        if (referenceFace < static_cast<int>(normalsA.size()))  // Fixed: use normalsA.size() for correct bounds check
        {
            // Reference face is on polygon A, find incident face on B
            int incidentFace = FindIncidentFace(normalsB, -separatingAxis);
            COLLISION_DEBUG_LOG("    Reference face on A (index=" << referenceFace << "), incident face on B (index=" << incidentFace << ")");
            ClipSegmentToLine(manifold.points, vertsA, vertsB, referenceFace, incidentFace, separatingAxis, minOverlap);
        }
        else
        {
            // Reference face is on polygon B, find incident face on A
            int incidentFace = FindIncidentFace(normalsA, -separatingAxis);  // Fixed: was +separatingAxis
            int refFace = referenceFace - static_cast<int>(normalsA.size());  // Fixed: use normalsA.size() for correct decode
            COLLISION_DEBUG_LOG("    Reference face on B (index=" << refFace << "), incident face on A (index=" << incidentFace << ")");
            ClipSegmentToLine(manifold.points, vertsB, vertsA, refFace, incidentFace, -separatingAxis, minOverlap);
            // Flip normal direction
            manifold.normal = -separatingAxis;
        }
        
        // Canonicalize normal to point from entity A toward entity B
        // This ensures consistent behavior regardless of which polygon is the reference
        Math::Vector2 d = transformB.position - transformA.position;
        if (Math::Vector2::Dot(d, manifold.normal) < 0.0f) {
            manifold.normal = -manifold.normal;
            // Also flip each contact point's normal and separation to match
            // the new normal direction (separation is defined along the normal).
            for (auto& cp : manifold.points) {
                cp.normal = -cp.normal;
                cp.separation = -cp.separation;
            }
        }
        
        // Store local-space data
        if (!manifold.points.empty())
        {
            Math::Vector2 invRotA = Rotate(manifold.normal, -transformA.rotation);
            manifold.localNormal = invRotA;
            manifold.localPoint = Rotate(manifold.points[0].position - transformA.position, -transformA.rotation);
            manifold.touching = true;
            
            COLLISION_DEBUG_LOG("  [PolygonPolygon] Generated " << manifold.points.size() << " contact point(s):");
            for (size_t i = 0; i < manifold.points.size(); ++i) {
                const auto& pt = manifold.points[i];
                COLLISION_DEBUG_LOG("    Contact[" << i << "] pos=(" << pt.position.x << "," << pt.position.y 
                                  << ") sep=" << pt.separation << " normalImpulse=" << pt.normalImpulse);
            }
        }
        else
        {
            COLLISION_DEBUG_LOG("  [PolygonPolygon] No contact points generated (clipping failed). Using fallback contact.");

            // Fallback: generate a single contact point at the midpoint between bodies.
            // This ensures the solver has something to resolve even if clipping fails.
            ECS::ContactPoint cp{};
            cp.position = (transformA.position + transformB.position) * 0.5f;
            cp.normal = manifold.normal;
            cp.separation = -minOverlap;
            cp.normalImpulse = 0.0f;
            cp.tangentImpulse = 0.0f;
            cp.normalMass = 0.0f;
            cp.tangentMass = 0.0f;
            cp.featureId = 0;
            cp.persisted = false;
            manifold.points.push_back(cp);
            manifold.touching = true;
        }
        
        return manifold;
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
        // Polygon vs Capsule: test polygon vertices against capsule line segment
        manifold.touching = false;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;
        
        const auto& capsule = capsuleCollider.GetCapsule();
        const auto& polygon = polyCollider.GetPolygon();
        
        if (polygon.vertices.empty())
            return manifold;
        
        // Transform capsule endpoints to world space with rotation
        float cosB = std::cos(transformB.rotation);
        float sinB = std::sin(transformB.rotation);
        Math::Vector2 c1 = {
            transformB.position.x + (capsule.center1.x * cosB - capsule.center1.y * sinB),
            transformB.position.y + (capsule.center1.x * sinB + capsule.center1.y * cosB)
        };
        Math::Vector2 c2 = {
            transformB.position.x + (capsule.center2.x * cosB - capsule.center2.y * sinB),
            transformB.position.y + (capsule.center2.x * sinB + capsule.center2.y * cosB)
        };
        
        // Transform polygon vertices to world space
        float cosA = std::cos(transformA.rotation);
        float sinA = std::sin(transformA.rotation);
        std::vector<Math::Vector2> worldVerts;
        worldVerts.reserve(polygon.vertices.size());
        for (const auto& v : polygon.vertices)
        {
            Math::Vector2 rotated{
                v.x * cosA - v.y * sinA,
                v.x * sinA + v.y * cosA
            };
            worldVerts.push_back(transformA.position + rotated);
        }
        
        // Find the closest distance between polygon vertices and capsule segment
        float minSeparation = -std::numeric_limits<float>::infinity();
        Math::Vector2 contactPoint{0.0f, 0.0f};
        Math::Vector2 normal{1.0f, 0.0f};
        int closestVertexIndex = -1;
        
        // Test each polygon vertex against the capsule line segment [c1, c2]
        for (size_t i = 0; i < worldVerts.size(); ++i)
        {
            const Math::Vector2& p = worldVerts[i];
            
            // Find closest point on capsule segment to this vertex
            Math::Vector2 segDir = c2 - c1;
            float segLenSq = Dot(segDir, segDir);
            
            Math::Vector2 closestOnSeg;
            if (segLenSq < 1e-8f)
            {
                // Capsule is degenerate (point)
                closestOnSeg = c1;
            }
            else
            {
                // Project vertex onto line, clamp to segment
                float t = Dot(p - c1, segDir) / segLenSq;
                t = std::clamp(t, 0.0f, 1.0f);
                closestOnSeg = c1 + segDir * t;
            }
            
            // Distance from vertex to capsule surface
            Math::Vector2 delta = p - closestOnSeg;
            float dist = delta.Length();
            
            // Check if vertex is inside capsule
            float separation = dist - capsule.radius;
            if (separation < minSeparation || closestVertexIndex == -1)
            {
                minSeparation = separation;
                closestVertexIndex = static_cast<int>(i);
                
                // Normal points from capsule to polygon
                if (dist > 1e-6f)
                {
                    normal = Normalize(delta);
                }
                else
                {
                    // Vertex is at capsule center - use perpendicular to segment
                    normal = Normalize(Math::Vector2{-segDir.y, segDir.x});
                }
                
                // Contact point on capsule surface
                contactPoint = closestOnSeg + normal * capsule.radius;
            }
        }
        
        // Also test capsule endpoints against polygon edges (for deep penetration cases)
        Math::Vector2 capPoints[2] = {c1, c2};
        for (int capIdx = 0; capIdx < 2; ++capIdx)
        {
            const Math::Vector2& capPt = capPoints[capIdx];
            
            // Test against each polygon edge
            for (size_t i = 0; i < worldVerts.size(); ++i)
            {
                size_t next = (i + 1) % worldVerts.size();
                Math::Vector2 edgeStart = worldVerts[i];
                Math::Vector2 edgeEnd = worldVerts[next];
                Math::Vector2 edgeDir = edgeEnd - edgeStart;
                float edgeLenSq = Dot(edgeDir, edgeDir);
                
                if (edgeLenSq < 1e-8f)
                    continue; // Degenerate edge
                
                // Project capsule endpoint onto edge
                float t = Dot(capPt - edgeStart, edgeDir) / edgeLenSq;
                t = std::clamp(t, 0.0f, 1.0f);
                Math::Vector2 closestOnEdge = edgeStart + edgeDir * t;
                
                // Distance from capsule endpoint to polygon edge
                Math::Vector2 delta = capPt - closestOnEdge;
                float dist = delta.Length();
                float separation = dist - capsule.radius;
                
                // Check if this is a deeper penetration
                if (separation < minSeparation)
                {
                    minSeparation = separation;
                    closestVertexIndex = -1; // Mark as edge contact
                    
                    if (dist > 1e-6f)
                    {
                        normal = Normalize(delta);
                    }
                    else
                    {
                        // Use polygon edge normal
                        Math::Vector2 edgeNormal{-edgeDir.y, edgeDir.x};
                        normal = Normalize(edgeNormal);
                        // Ensure normal points toward capsule
                        if (Dot(normal, delta) < 0.0f)
                            normal = -normal;
                    }
                    
                    contactPoint = closestOnEdge;
                }
            }
        }
        
        // Check if we have a collision
        if (minSeparation > 0.0f)
            return manifold; // No collision
        
        float penetration = -minSeparation;
        
        // Create contact manifold
        ECS::ContactPoint cp{};
        cp.position = contactPoint;
        cp.normal = normal;
        cp.separation = -penetration;
        cp.normalImpulse = 0.0f;
        cp.tangentImpulse = 0.0f;
        cp.normalMass = 0.0f;
        cp.tangentMass = 0.0f;
        cp.featureId = static_cast<uint32_t>(closestVertexIndex >= 0 ? closestVertexIndex : 0xFF);
        cp.persisted = false;
        
        manifold.points.push_back(cp);
        manifold.normal = normal;
        
        // Store local-space data (relative to polygon body A)
        Math::Vector2 invRot = Rotate(normal, -transformA.rotation);
        manifold.localNormal = invRot;
        Math::Vector2 localContact = Rotate(contactPoint - transformA.position, -transformA.rotation);
        manifold.localPoint = localContact;
        
        manifold.touching = true;
        
        return manifold;
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
        // Dispatch capsule collision to appropriate handler based on other shape
        using ST = ColliderComponent::ShapeType;
        ST tA = colliderA.GetType();
        ST tB = colliderB.GetType();
        
        // Handle circle vs capsule
        if ((tA == ST::Capsule && tB == ST::Circle) || (tA == ST::Circle && tB == ST::Capsule))
        {
            const ColliderComponent* circleColl = (tA == ST::Circle) ? &colliderA : &colliderB;
            const ColliderComponent* capColl = (tA == ST::Capsule) ? &colliderA : &colliderB;
            const TransformComponent* circleTrans = (tA == ST::Circle) ? &transformA : &transformB;
            const TransformComponent* capTrans = (tA == ST::Capsule) ? &transformA : &transformB;
            uint32_t circleEnt = (tA == ST::Circle) ? entityIdA : entityIdB;
            uint32_t capEnt = (tA == ST::Capsule) ? entityIdA : entityIdB;
            uint32_t circleShape = (tA == ST::Circle) ? shapeIdA : shapeIdB;
            uint32_t capShape = (tA == ST::Capsule) ? shapeIdA : shapeIdB;
            
            return CircleCapsule(circleEnt, capEnt, circleShape, capShape,
                               *circleColl, *capColl, *circleTrans, *capTrans, manifold);
        }
        
        // Handle polygon vs capsule
        if ((tA == ST::Capsule && tB == ST::Polygon) || (tA == ST::Polygon && tB == ST::Capsule))
        {
            const ColliderComponent* polyColl = (tA == ST::Polygon) ? &colliderA : &colliderB;
            const ColliderComponent* capColl = (tA == ST::Capsule) ? &colliderA : &colliderB;
            const TransformComponent* polyTrans = (tA == ST::Polygon) ? &transformA : &transformB;
            const TransformComponent* capTrans = (tA == ST::Capsule) ? &transformA : &transformB;
            uint32_t polyEnt = (tA == ST::Polygon) ? entityIdA : entityIdB;
            uint32_t capEnt = (tA == ST::Capsule) ? entityIdA : entityIdB;
            uint32_t polyShape = (tA == ST::Polygon) ? shapeIdA : shapeIdB;
            uint32_t capShape = (tA == ST::Capsule) ? shapeIdA : shapeIdB;
            
            return PolygonCapsule(polyEnt, capEnt, polyShape, capShape,
                                *polyColl, *capColl, *polyTrans, *capTrans, manifold);
        }
        
        // Handle capsule vs capsule
        if (tA == ST::Capsule && tB == ST::Capsule)
        {
            return CapsuleCapsule(entityIdA, entityIdB, shapeIdA, shapeIdB,
                                 colliderA, colliderB, transformA, transformB, manifold);
        }
        
        // Unsupported shape combination
        return manifold;
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
}

