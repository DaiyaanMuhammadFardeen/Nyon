#include "nyon/utils/CollisionPhysics.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace Nyon::Utils
{
    // Helper function to get the normal of an edge (perpendicular vector)
    Math::Vector2 CollisionPhysics::GetEdgeNormal(const Math::Vector2& edge)
    {
        return Math::Vector2(-edge.y, edge.x);
    }

    // Helper function to calculate dot product
    float CollisionPhysics::DotProduct(const Math::Vector2& a, const Math::Vector2& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    // Project a polygon onto an axis and return min/max values
    std::pair<float, float> CollisionPhysics::ProjectPolygonOntoAxis(const Polygon& polygon,
                                                                     const Math::Vector2& pos,
                                                                     const Math::Vector2& axis)
    {
        if (polygon.empty()) {
            return {std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()};
        }

        float minProj = DotProduct(polygon[0] + pos, axis);
        float maxProj = minProj;

        for (size_t i = 1; i < polygon.size(); ++i) {
            float projection = DotProduct(polygon[i] + pos, axis);
            if (projection < minProj) {
                minProj = projection;
            }
            if (projection > maxProj) {
                maxProj = projection;
            }
        }

        return {minProj, maxProj};
    }

    // Calculate the center of a polygon
    Math::Vector2 CollisionPhysics::CalculatePolygonCenter(const Polygon& polygon, const Math::Vector2& pos)
    {
        if (polygon.empty()) {
            return pos;
        }

        Math::Vector2 sum(0, 0);
        for (const auto& vertex : polygon) {
            sum = sum + vertex + pos;
        }
        return sum / static_cast<float>(polygon.size());
    }

    // Check if two projected intervals overlap
    bool CollisionPhysics::CheckOverlap(float min1, float max1, float min2, float max2)
    {
        const float epsilon = 0.0001f;
        return !(max1 < min2 - epsilon || max2 < min1 - epsilon);
    }

    // Calculate overlap distance between two intervals
    float CollisionPhysics::CalculateIntervalOverlap(float min1, float max1, float min2, float max2)
    {
        return std::min(max1, max2) - std::max(min1, min2);
    }

    // Broad-phase collision check before SAT
    bool CollisionPhysics::CheckAABBCollision(const Math::Vector2& pos1, const Math::Vector2& size1,
                                            const Math::Vector2& pos2, const Math::Vector2& size2)
    {
        float x1 = pos1.x;
        float y1 = pos1.y;
        float w1 = size1.x;
        float h1 = size1.y;

        float x2 = pos2.x;
        float y2 = pos2.y;
        float w2 = size2.x;
        float h2 = size2.y;

        bool collisionX = x1 < x2 + w2 && x1 + w1 > x2;
        bool collisionY = y1 < y2 + h2 && y1 + h1 > y2;

        return collisionX && collisionY;
    }

    // Calculate AABB for a polygon
    void CollisionPhysics::CalculateAABB(const Polygon& polygon, const Math::Vector2& pos,
                                        Math::Vector2& outMin, Math::Vector2& outMax)
    {
        if (polygon.empty()) {
            outMin = pos;
            outMax = pos;
            return;
        }

        outMin = polygon[0] + pos;
        outMax = outMin;

        for (const auto& vertex : polygon) {
            Math::Vector2 worldVertex = vertex + pos;
            if (worldVertex.x < outMin.x) outMin.x = worldVertex.x;
            if (worldVertex.y < outMin.y) outMin.y = worldVertex.y;
            if (worldVertex.x > outMax.x) outMax.x = worldVertex.x;
            if (worldVertex.y > outMax.y) outMax.y = worldVertex.y;
        }
    }

    // Create swept AABB from start and end positions
    void CollisionPhysics::CalculateSweptAABB(const Polygon& polygon, const Math::Vector2& startPos,
                                             const Math::Vector2& endPos,
                                             Math::Vector2& outMin, Math::Vector2& outMax)
    {
        Math::Vector2 min1, max1, min2, max2;
        CalculateAABB(polygon, startPos, min1, max1);
        CalculateAABB(polygon, endPos, min2, max2);

        // Combine both AABBs to create swept volume
        outMin.x = std::min(min1.x, min2.x);
        outMin.y = std::min(min1.y, min2.y);
        outMax.x = std::max(max1.x, max2.x);
        outMax.y = std::max(max1.y, max2.y);
    }

    // Ray-AABB intersection test (for early rejection in CCD)
    bool CollisionPhysics::RayAABBIntersection(const Math::Vector2& rayOrigin,
                                              const Math::Vector2& rayDirection,
                                              const Math::Vector2& aabbMin,
                                              const Math::Vector2& aabbMax,
                                              float& tMin, float& tMax)
    {
        const float epsilon = 1e-6f;
        tMin = 0.0f;
        tMax = 1.0f;

        // Check X axis
        if (std::abs(rayDirection.x) > epsilon) {
            float tx1 = (aabbMin.x - rayOrigin.x) / rayDirection.x;
            float tx2 = (aabbMax.x - rayOrigin.x) / rayDirection.x;

            tMin = std::max(tMin, std::min(tx1, tx2));
            tMax = std::min(tMax, std::max(tx1, tx2));
        } else {
            // Ray parallel to X axis
            if (rayOrigin.x < aabbMin.x || rayOrigin.x > aabbMax.x) {
                return false;
            }
        }

        // Check Y axis
        if (std::abs(rayDirection.y) > epsilon) {
            float ty1 = (aabbMin.y - rayOrigin.y) / rayDirection.y;
            float ty2 = (aabbMax.y - rayOrigin.y) / rayDirection.y;

            tMin = std::max(tMin, std::min(ty1, ty2));
            tMax = std::min(tMax, std::max(ty1, ty2));
        } else {
            // Ray parallel to Y axis
            if (rayOrigin.y < aabbMin.y || rayOrigin.y > aabbMax.y) {
                return false;
            }
        }

        return tMax >= tMin && tMin <= 1.0f && tMax >= 0.0f;
    }

    // Main SAT-based collision detection function (discrete)
    CollisionPhysics::CollisionResult CollisionPhysics::CheckPolygonCollision(const Polygon& poly1, const Math::Vector2& pos1,
                                                                           const Polygon& poly2, const Math::Vector2& pos2)
    {
        if (poly1.empty() || poly2.empty()) {
            return CollisionResult(false, Math::Vector2(0.0f, 0.0f), 0.0f);
        }

        // Broad-phase AABB check
        if (poly1.size() >= 2 && poly2.size() >= 2) {
            Math::Vector2 min1, max1, min2, max2;
            CalculateAABB(poly1, pos1, min1, max1);
            CalculateAABB(poly2, pos2, min2, max2);

            Math::Vector2 size1 = max1 - min1;
            Math::Vector2 size2 = max2 - min2;

            if (!CheckAABBCollision(min1, size1, min2, size2)) {
                return CollisionResult(false, Math::Vector2(0.0f, 0.0f), 0.0f);
            }
        }

        // Get all potential separating axes from both polygons
        std::vector<Math::Vector2> axes;
        const float epsilon = 1e-6f;
        const float parallelEpsilon = 0.999f;

        // Get axes from poly1 edges
        for (size_t i = 0; i < poly1.size(); ++i) {
            size_t nextIdx = (i + 1) % poly1.size();
            Math::Vector2 edge = poly1[nextIdx] - poly1[i];
            Math::Vector2 normal = GetEdgeNormal(edge);
            float length = std::sqrt(normal.x * normal.x + normal.y * normal.y);
            if (length > epsilon) {
                normal.x /= length;
                normal.y /= length;

                bool isUnique = true;
                for (const auto& existingAxis : axes) {
                    float dotProduct = std::abs(DotProduct(normal, existingAxis));
                    if (dotProduct > parallelEpsilon) {
                        isUnique = false;
                        break;
                    }
                }

                if (isUnique) {
                    axes.push_back(normal);
                }
            }
        }

        // Get axes from poly2 edges
        for (size_t i = 0; i < poly2.size(); ++i) {
            size_t nextIdx = (i + 1) % poly2.size();
            Math::Vector2 edge = poly2[nextIdx] - poly2[i];
            Math::Vector2 normal = GetEdgeNormal(edge);
            float length = std::sqrt(normal.x * normal.x + normal.y * normal.y);
            if (length > epsilon) {
                normal.x /= length;
                normal.y /= length;

                bool isUnique = true;
                for (const auto& existingAxis : axes) {
                    float dotProduct = std::abs(DotProduct(normal, existingAxis));
                    if (dotProduct > parallelEpsilon) {
                        isUnique = false;
                        break;
                    }
                }

                if (isUnique) {
                    axes.push_back(normal);
                }
            }
        }

        // Track the axis with the minimum overlap (MTV)
        float minOverlap = std::numeric_limits<float>::max();
        Math::Vector2 minAxis = Math::Vector2(0.0f, 0.0f);

        // Check for separation along each axis
        for (const auto& axis : axes) {
            auto proj1 = ProjectPolygonOntoAxis(poly1, pos1, axis);
            auto proj2 = ProjectPolygonOntoAxis(poly2, pos2, axis);

            if (!CheckOverlap(proj1.first, proj1.second, proj2.first, proj2.second)) {
                return CollisionResult(false, Math::Vector2(0.0f, 0.0f), 0.0f);
            }

            float overlap = CalculateIntervalOverlap(proj1.first, proj1.second, proj2.first, proj2.second);

            const float epsilon_overlap = 1e-4f;
            if (overlap < epsilon_overlap) {
                continue;
            }

            if (overlap < minOverlap) {
                minOverlap = overlap;
                minAxis = axis;
            }
        }

        // Determine correct direction of MTV
        Math::Vector2 center1 = CalculatePolygonCenter(poly1, pos1);
        Math::Vector2 center2 = CalculatePolygonCenter(poly2, pos2);
        Math::Vector2 directionVec = center2 - center1;

        if (DotProduct(directionVec, minAxis) < 0.0f) {
            minAxis = minAxis * -1.0f;
        }

        return CollisionResult(true, minAxis, minOverlap);
    }

    // ============================================================================
    // CONTINUOUS COLLISION DETECTION (CCD) IMPLEMENTATION
    // ============================================================================

    // Conservative advancement using binary search
    // This finds the Time of Impact (TOI) between two moving polygons
    CollisionPhysics::CCDResult CollisionPhysics::ContinuousCollisionCheck(
        const Polygon& poly1, const Math::Vector2& startPos1, const Math::Vector2& endPos1,
        const Polygon& poly2, const Math::Vector2& startPos2, const Math::Vector2& endPos2,
        int maxIterations)
    {
        const float epsilon = 0.001f; // Tolerance for TOI precision
        const float safetyMargin = 0.02f; // Small margin to prevent tunneling

        // Early exit: check if objects are colliding at start
        CollisionResult startCollision = CheckPolygonCollision(poly1, startPos1, poly2, startPos2);
        if (startCollision.collided) {
            return CCDResult(true, 0.0f, startPos1, startCollision);
        }

        // Early exit: check if objects don't collide at end
        CollisionResult endCollision = CheckPolygonCollision(poly1, endPos1, poly2, endPos2);
        if (!endCollision.collided) {
            // Additional swept AABB check to see if paths could have crossed
            Math::Vector2 swept1Min, swept1Max, swept2Min, swept2Max;
            CalculateSweptAABB(poly1, startPos1, endPos1, swept1Min, swept1Max);
            CalculateSweptAABB(poly2, startPos2, endPos2, swept2Min, swept2Max);

            Math::Vector2 swept1Size = swept1Max - swept1Min;
            Math::Vector2 swept2Size = swept2Max - swept2Min;

            if (!CheckAABBCollision(swept1Min, swept1Size, swept2Min, swept2Size)) {
                return CCDResult(false, 1.0f, endPos1, CollisionResult(false, Math::Vector2(0.0f, 0.0f), 0.0f));
            }
        }

        // Binary search for Time of Impact
        float tMin = 0.0f;
        float tMax = 1.0f;
        float toi = 1.0f;
        Math::Vector2 impactPos = endPos1;
        CollisionResult impactCollision(false, Math::Vector2(0.0f, 0.0f), 0.0f);

        for (int iteration = 0; iteration < maxIterations; ++iteration) {
            float tMid = (tMin + tMax) * 0.5f;

            // Interpolate positions at time t
            Math::Vector2 pos1 = startPos1 + (endPos1 - startPos1) * tMid;
            Math::Vector2 pos2 = startPos2 + (endPos2 - startPos2) * tMid;

            CollisionResult collision = CheckPolygonCollision(poly1, pos1, poly2, pos2);

            if (collision.collided) {
                // Collision found, search earlier in time
                tMax = tMid;
                toi = tMid;
                impactPos = pos1;
                impactCollision = collision;

                // If we're close enough, stop
                if (tMax - tMin < epsilon) {
                    break;
                }
            } else {
                // No collision, search later in time
                tMin = tMid;

                if (tMax - tMin < epsilon) {
                    break;
                }
            }
        }

        // If we found a collision, back up slightly to ensure safe position
        if (impactCollision.collided && toi > 0.0f) {
            toi = std::max(0.0f, toi - safetyMargin);
            impactPos = startPos1 + (endPos1 - startPos1) * toi;
        }

        return CCDResult(impactCollision.collided, toi, impactPos, impactCollision);
    }

    // Simplified CCD for moving vs static objects (common in platformers)
    CollisionPhysics::CCDResult CollisionPhysics::ContinuousCollisionCheckMovingVsStatic(
        const Polygon& movingPoly, const Math::Vector2& startPos, const Math::Vector2& endPos,
        const Polygon& staticPoly, const Math::Vector2& staticPos,
        int maxIterations)
    {
        return ContinuousCollisionCheck(movingPoly, startPos, endPos,
                                       staticPoly, staticPos, staticPos,
                                       maxIterations);
    }

    // Cast a ray and return the first collision point and normal
    CollisionPhysics::RaycastResult CollisionPhysics::RaycastPolygon(
        const Math::Vector2& rayStart, const Math::Vector2& rayEnd,
        const Polygon& polygon, const Math::Vector2& polyPos)
    {
        if (polygon.empty()) {
            return RaycastResult(false, Math::Vector2(0.0f, 0.0f), Math::Vector2(0.0f, 0.0f), 1.0f);
        }

        Math::Vector2 rayDir = rayEnd - rayStart;
        float rayLength = std::sqrt(rayDir.x * rayDir.x + rayDir.y * rayDir.y);

        if (rayLength < 1e-6f) {
            return RaycastResult(false, Math::Vector2(0.0f, 0.0f), Math::Vector2(0.0f, 0.0f), 1.0f);
        }

        rayDir.x /= rayLength;
        rayDir.y /= rayLength;

        float closestT = std::numeric_limits<float>::max();
        Math::Vector2 closestPoint(0.0f, 0.0f);
        Math::Vector2 closestNormal(0.0f, 0.0f);
        bool hit = false;

        // Check intersection with each edge of the polygon
        for (size_t i = 0; i < polygon.size(); ++i) {
            size_t nextIdx = (i + 1) % polygon.size();

            Math::Vector2 edgeStart = polygon[i] + polyPos;
            Math::Vector2 edgeEnd = polygon[nextIdx] + polyPos;
            Math::Vector2 edgeDir = edgeEnd - edgeStart;

            // Ray-line segment intersection using parametric form
            // Ray: P = rayStart + t * rayDir
            // Edge: Q = edgeStart + s * edgeDir
            // Solve: rayStart + t * rayDir = edgeStart + s * edgeDir

            float cross = rayDir.x * edgeDir.y - rayDir.y * edgeDir.x;

            if (std::abs(cross) < 1e-6f) {
                continue; // Parallel or collinear
            }

            Math::Vector2 diff = edgeStart - rayStart;
            float t = (diff.x * edgeDir.y - diff.y * edgeDir.x) / cross;
            float s = (diff.x * rayDir.y - diff.y * rayDir.x) / cross;

            // Check if intersection is within both line segments
            if (t >= 0.0f && t <= rayLength && s >= 0.0f && s <= 1.0f) {
                if (t < closestT) {
                    closestT = t;
                    closestPoint = rayStart + rayDir * t;

                    // Calculate edge normal
                    Math::Vector2 edgeNormal = GetEdgeNormal(edgeDir);
                    float normalLength = std::sqrt(edgeNormal.x * edgeNormal.x + edgeNormal.y * edgeNormal.y);
                    if (normalLength > 1e-6f) {
                        edgeNormal.x /= normalLength;
                        edgeNormal.y /= normalLength;
                    }

                    // Make sure normal points toward the ray origin
                    if (DotProduct(edgeNormal, rayDir) > 0.0f) {
                        edgeNormal = edgeNormal * -1.0f;
                    }

                    closestNormal = edgeNormal;
                    hit = true;
                }
            }
        }

        float hitT = (rayLength > 0.0f) ? (closestT / rayLength) : 1.0f;
        return RaycastResult(hit, closestPoint, closestNormal, hitT);
    }

    // ============================================================================
    // COLLISION RESOLUTION
    // ============================================================================

    void CollisionPhysics::ResolveCollision(Physics::Body& body1, Physics::Body& body2, const CollisionResult& result)
    {
        if (!result.collided) {
            return;
        }

        if (body1.isStatic && body2.isStatic) {
            return;
        }

        float mass1 = body1.isStatic ? 0.0f : body1.mass;
        float mass2 = body2.isStatic ? 0.0f : body2.mass;
        float totalMass = mass1 + mass2;

        if (totalMass <= 0.0f) {
            return;
        }

        float percent1 = (totalMass > 0.0f) ? mass2 / totalMass : 0.5f;
        float percent2 = (totalMass > 0.0f) ? mass1 / totalMass : 0.5f;

        // Penetration correction with slop
        const float penetrationSlop = 0.01f;
        float correctedPenetration = std::max(result.overlapAmount - penetrationSlop, 0.0f);

        // Move bodies apart using MTV
        if (!body1.isStatic) {
            body1.position = body1.position - result.overlapAxis * correctedPenetration * percent1;
        }

        if (!body2.isStatic) {
            body2.position = body2.position + result.overlapAxis * correctedPenetration * percent2;
        }

        // Velocity correction
        Math::Vector2 relativeVelocity = body2.velocity - body1.velocity;
        float velocityAlongNormal = DotProduct(relativeVelocity, result.overlapAxis);

        if (velocityAlongNormal > 0.0f) {
            return;
        }

        // Calculate restitution (can be made configurable per body)
        float restitution = 0.0f; // Perfectly inelastic
        float j = -(1.0f + restitution) * velocityAlongNormal;
        j /= (body1.isStatic ? 0.0f : 1.0f / body1.mass) + (body2.isStatic ? 0.0f : 1.0f / body2.mass);

        Math::Vector2 impulse = result.overlapAxis * j;

        if (!body1.isStatic) {
            body1.velocity = body1.velocity - impulse * (1.0f / body1.mass);
        }

        if (!body2.isStatic) {
            body2.velocity = body2.velocity + impulse * (1.0f / body2.mass);
        }
    }

    // Resolve collision using CCD result
    void CollisionPhysics::ResolveCCDCollision(Physics::Body& body, const CCDResult& ccdResult, float deltaTime)
    {
        if (!ccdResult.collided || body.isStatic) {
            return;
        }

        // Move body to impact position
        body.position = ccdResult.impactPosition;

        // Project velocity along collision normal to remove component moving into the surface
        const CollisionResult& collision = ccdResult.collision;
        float velocityAlongNormal = DotProduct(body.velocity, collision.overlapAxis);

        if (velocityAlongNormal < 0.0f) {
            // Remove velocity component moving into the surface
            body.velocity = body.velocity - collision.overlapAxis * velocityAlongNormal;

            // Optional: Add a small bounce
            // float restitution = 0.0f;
            // body.velocity = body.velocity + collision.overlapAxis * (velocityAlongNormal * restitution);
        }
    }

    bool CollisionPhysics::IsBodyGrounded(const Physics::Body& body, const CollisionResult& collisionResult, float threshold)
    {
        if (!collisionResult.collided) {
            return false;
        }

        // Body is grounded if collision normal points upward (y-component is negative in y-down system)
        return (collisionResult.overlapAxis.y < 0.0f && std::abs(collisionResult.overlapAxis.y) > threshold);
    }
}