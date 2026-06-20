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
        
        // Clip incident edge segment against reference face boundaries.
        // Keeps the portion of the incident edge that lies within the reference face's
        // side planes (edge-direction extent) and is penetrating the reference face plane.
        // featureId encodes: (refFaceIndex << 16) | incFaceIndex | (clipIdx << 24)
        // so each clip vertex gets a unique, persistent identifier for warm starting.
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

            // Reference edge direction (used as side plane normals).
            // Side plane 1 at v1 keeps points at or ahead of v1 along edgeDir.
            // Side plane 2 at v2 keeps points at or behind v2 along edgeDir.
            Math::Vector2 edgeDir = v2 - v1;
            float edgeLen = edgeDir.Length();
            if (edgeLen < 1e-8f) return;
            edgeDir = edgeDir * (1.0f / edgeLen);

            float offset1 = Dot(edgeDir, v1);
            float offset2 = Dot(-edgeDir, v2);

            // Clip the two incident vertices against both side planes
            std::vector<Math::Vector2> clippingIn;
            clippingIn.push_back(iv1);
            clippingIn.push_back(iv2);

            for (int pass = 0; pass < 2; ++pass)
            {
                Math::Vector2 clipNormal = (pass == 0) ? edgeDir : -edgeDir;
                float clipOffset = (pass == 0) ? offset1 : offset2;

                std::vector<Math::Vector2> clippingOut;
                for (size_t i = 0; i < clippingIn.size(); ++i)
                {
                    size_t j = (i + 1) % clippingIn.size();
                    const auto& p1 = clippingIn[i];
                    const auto& p2 = clippingIn[j];
                    float d1 = Dot(clipNormal, p1) - clipOffset;
                    float d2 = Dot(clipNormal, p2) - clipOffset;

                    if (d1 >= 0.0f)
                        clippingOut.push_back(p1);
                    if (d1 * d2 < 0.0f && std::abs(d2 - d1) > 1e-6f)
                    {
                        float t = d1 / (d1 - d2);
                        clippingOut.push_back(p1 + (p2 - p1) * t);
                    }
                }
                clippingIn = std::move(clippingOut);
            }

            // --- Final clip against reference face plane ---
            float refOffset = Dot(normal, v1);
            for (size_t ci = 0; ci < clippingIn.size(); ++ci)
            {
                const auto& pt = clippingIn[ci];
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
                    cp.featureId    = (static_cast<uint32_t>(refFaceIndex) << 16)
                                    | (static_cast<uint32_t>(incFaceIndex) & 0xFFFF)
                                    | (static_cast<uint32_t>(ci) << 24);
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
                auto result = CirclePolygon(entityIdB, entityIdA, shapeIdB, shapeIdA,
                                           colliderB.GetCircle(), colliderA.GetPolygon(),
                                           transformB, transformA, manifold);
                // When swapped, the result has entityIdA=circle (original B) and entityIdB=polygon (original A),
                // with normal pointing circle→polygon. The manifold must use the original entity order
                // (polygon, circle) with normal pointing polygon→circle.
                std::swap(result.entityIdA, result.entityIdB);
                result.normal = -result.normal;
                result.localNormal = -result.localNormal;
                for (auto& cp : result.points) {
                    cp.normal = -cp.normal;
                }
                return result;
            }
            else
            {
                return CirclePolygon(entityIdA, entityIdB, shapeIdA, shapeIdB,
                                    colliderA.GetCircle(), colliderB.GetPolygon(),
                                    transformA, transformB, manifold);
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
                                                           ECS::ContactManifold& manifold)
    {
        manifold.touching = false;
        // Entity IDs are set by GenerateManifold, do not overwrite them here.

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
        Math::Vector2 normal = -polyNormal;
        
        float penetration = circle.radius - maxSeparation;
        
        // Contact point is the circle center projected along the manifold normal
        // toward the polygon (from A→B convention: normal points from circle toward polygon).
        Math::Vector2 contactPoint = center + normal * (circle.radius - 0.5f * penetration);
        
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

            COLLISION_DEBUG_LOG("    SAT axis=A[" << i << "] (" << axis.x << "," << axis.y << ") overlap=" << overlap << " minA=" << minA << " maxA=" << maxA << " minB=" << minB << " maxB=" << maxB);
            
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
            
            COLLISION_DEBUG_LOG("    SAT axis=B[" << i << "] (" << axis.x << "," << axis.y << ") overlap=" << overlap << " minA=" << minA << " maxA=" << maxA << " minB=" << minB << " maxB=" << maxB);

            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                separatingAxis = axis;
                referenceFace = static_cast<int>(i) + static_cast<int>(normalsA.size());
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
            int incidentFace = FindIncidentFace(normalsA, -separatingAxis);
            int refFace = referenceFace - static_cast<int>(normalsA.size());
            COLLISION_DEBUG_LOG("    Reference face on B (index=" << refFace << "), incident face on A (index=" << incidentFace << ")");
            // Pass separatingAxis (B's outward normal) to ClipSegmentToLine so it keeps
            // points inside B (penetrating) rather than outside B.
            ClipSegmentToLine(manifold.points, vertsB, vertsA, refFace, incidentFace, separatingAxis, minOverlap);
            // ClipSegmentToLine sets cp.normal = separatingAxis (B's outward).
            // Convention requires normal from A to B = -separatingAxis.
            for (auto& cp : manifold.points) {
                cp.normal = -separatingAxis;
                cp.separation = -cp.separation;
            }
            manifold.normal = -separatingAxis;
        }
        
        // Canonicalize normal to point from entity A toward entity B
        // This ensures consistent behavior regardless of which polygon is the reference.
        // We use the shape centroids instead of raw transform positions to ensure
        // correct orientation even for highly asymmetric polygons or significant offsets.
        Math::Vector2 centroidA = transformA.position + Rotate(polyA.centroid, transformA.rotation);
        Math::Vector2 centroidB = transformB.position + Rotate(polyB.centroid, transformB.rotation);
        Math::Vector2 d = centroidB - centroidA;
        
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

            // Fallback: generate a single contact point at the midpoint between shape centroids.
            // This is more accurate than using raw body positions for offset polygons.
            ECS::ContactPoint cp{};
            cp.position = (centroidA + centroidB) * 0.5f;
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
                    
                    // Normal points from polygon (A) to capsule (B), i.e., opposite to delta.
                    if (dist > 1e-6f)
                    {
                        normal = -Normalize(delta);
                    }
                    else
                    {
                        // Vertex is at capsule center - use perpendicular to segment
                        normal = -Normalize(Math::Vector2{-segDir.y, segDir.x});
                    }
                    
                    // Contact point on polygon vertex (the colliding point on body A)
                    contactPoint = p;
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
                        // Capsule endpoint is exactly on polygon edge.
                        // Use polygon edge outward normal.
                        Math::Vector2 edgeNormal{-edgeDir.y, edgeDir.x};
                        normal = Normalize(edgeNormal);
                        // Determine outward direction using polygon centroid (which is always inside).
                        Math::Vector2 centroid = transformA.position + Rotate(polygon.centroid, transformA.rotation);
                        // If the edge normal points toward the centroid, flip to point outward (A→B).
                        if (Dot(normal, centroid - closestOnEdge) > 0.0f)
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
            // Compute unconstrained closest-point parameters
            s = (b * e - c * d) / denom;
            t = (a * e - b * d) / denom;
            
            // Clamp with recomputation to handle parameter coupling.
            // When one parameter is clamped, we must recompute the other
            // given the fixed value to find the correct closest point on the boundary.
            if (s < 0.0f) {
                s = 0.0f;
                t = e / c;
            } else if (s > 1.0f) {
                s = 1.0f;
                t = (e + b) / c;
            }
            t = std::clamp(t, 0.0f, 1.0f);
            
            if (t < 0.0f) {
                t = 0.0f;
                s = -d / a;
            } else if (t > 1.0f) {
                t = 1.0f;
                s = (b - d) / a;
            }
            s = std::clamp(s, 0.0f, 1.0f);
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
        manifold.touching = false;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        manifold.shapeIdA = shapeIdA;
        manifold.shapeIdB = shapeIdB;

        using ST = ColliderComponent::ShapeType;
        ST tA = colliderA.GetType();
        ST tB = colliderB.GetType();

        // Identify which collider is the segment
        const ColliderComponent* segColl = (tA == ST::Segment) ? &colliderA : &colliderB;
        const ColliderComponent* otherColl = (tA == ST::Segment) ? &colliderB : &colliderA;
        const TransformComponent* segTrans = (tA == ST::Segment) ? &transformA : &transformB;
        const TransformComponent* otherTrans = (tA == ST::Segment) ? &transformB : &transformA;
        uint32_t segEnt = (tA == ST::Segment) ? entityIdA : entityIdB;
        uint32_t otherEnt = (tA == ST::Segment) ? entityIdB : entityIdA;
        uint32_t segShape = (tA == ST::Segment) ? shapeIdA : shapeIdB;
        uint32_t otherShape = (tA == ST::Segment) ? shapeIdB : shapeIdA;

        // Build a temporary CapsuleShape from the segment (same endpoints, same radius)
        ColliderComponent::CapsuleShape tempCap;
        tempCap.center1 = segColl->GetSegment().point1;
        tempCap.center2 = segColl->GetSegment().point2;
        tempCap.radius = segColl->GetSegment().radius;

        // Build a temporary ColliderComponent wrapping the capsule
        ColliderComponent tempCapColl(tempCap);
        tempCapColl.material = segColl->material;

        ST otherType = otherColl->GetType();

        // Segment vs Circle
        if (otherType == ST::Circle)
        {
            return CapsuleCollision(segEnt, otherEnt, segShape, otherShape,
                                   tempCapColl, *otherColl, *segTrans, *otherTrans, manifold);
        }

        // Segment vs Polygon
        if (otherType == ST::Polygon)
        {
            return CapsuleCollision(segEnt, otherEnt, segShape, otherShape,
                                   tempCapColl, *otherColl, *segTrans, *otherTrans, manifold);
        }

        // Segment vs Capsule — delegate to CapsuleCapsule
        if (otherType == ST::Capsule)
        {
            return CapsuleCapsule(segEnt, otherEnt, segShape, otherShape,
                                 tempCapColl, *otherColl, *segTrans, *otherTrans, manifold);
        }

        // Segment vs Segment — treat both as capsules
        if (otherType == ST::Segment)
        {
            ColliderComponent::CapsuleShape tempCapB;
            tempCapB.center1 = otherColl->GetSegment().point1;
            tempCapB.center2 = otherColl->GetSegment().point2;
            tempCapB.radius = otherColl->GetSegment().radius;
            ColliderComponent tempCapCollB(tempCapB);
            tempCapCollB.material = otherColl->material;

            return CapsuleCapsule(segEnt, otherEnt, segShape, otherShape,
                                 tempCapColl, tempCapCollB, *segTrans, *otherTrans, manifold);
        }

        return manifold;
    }
}

