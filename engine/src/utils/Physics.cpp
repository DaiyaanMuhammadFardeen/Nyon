#include "nyon/utils/Physics.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace Nyon::Utils
{
    const float Physics::Gravity = 980.0f; // pixels/s^2
    
    void Physics::UpdateBody(Body& body, float deltaTime, bool isGrounded)
    {
        if (!body.isStatic)
        {
            // Sub-stepping to prevent tunneling with large delta times
            const float maxStep = 1.0f / 60.0f;  // Maximum time step (60 FPS equivalent)
            float remainingTime = deltaTime;
            
            while (remainingTime > maxStep) {
                InternalUpdateBody(body, maxStep, isGrounded);
                remainingTime -= maxStep;
            }
            // Update with remaining time (could be 0 if deltaTime < maxStep)
            InternalUpdateBody(body, remainingTime, isGrounded);
        }
    }
    
    void Physics::InternalUpdateBody(Body& body, float deltaTime, bool isGrounded)
    {
        if (deltaTime > 0.0f)
        {
            if (body.isStatic) {
                // Static bodies have zero velocity and acceleration
                body.velocity.x = 0.0f;
                body.velocity.y = 0.0f;
                body.acceleration.x = 0.0f;
                body.acceleration.y = 0.0f;
                return; // Static bodies don't need further updates
            }
            
            // Apply other accelerations to velocity
            body.velocity.x += body.acceleration.x * deltaTime;
            body.velocity.y += body.acceleration.y * deltaTime;
            
            // Apply gravity as acceleration
            body.acceleration.y += Gravity;
            
            // Apply all accumulated acceleration to velocity
            body.velocity.x += body.acceleration.x * deltaTime;
            body.velocity.y += body.acceleration.y * deltaTime;
            
            // Handle grounded state after velocity update
            if (isGrounded) {
                // Clamp vertical velocity if moving downward into the ground
                if (body.velocity.y > 0.0f) {
                    body.velocity.y = 0.0f;
                }
            }
            
            // Apply friction if grounded
            if (isGrounded) {
                body.velocity.x *= (1.0f - std::min(body.friction * deltaTime, 1.0f));
            }
            // Apply drag (air resistance)
            body.velocity *= (1.0f - std::min(body.drag * deltaTime, 1.0f));
            
            // Apply velocity to position (Symplectic Euler)
            body.position.x += body.velocity.x * deltaTime;
            body.position.y += body.velocity.y * deltaTime;
            
            // Clamp velocity to prevent extreme speeds
            const float maxVel = body.maxSpeed;
            if (body.velocity.x > maxVel) body.velocity.x = maxVel;
            if (body.velocity.x < -maxVel) body.velocity.x = -maxVel;
            if (body.velocity.y > maxVel) body.velocity.y = maxVel;
            if (body.velocity.y < -maxVel) body.velocity.y = -maxVel;
            
            // CRITICAL: Reset acceleration for the next frame
            body.acceleration.x = 0.0f;
            body.acceleration.y = 0.0f;
        }
    }
    
    bool Physics::CheckCollision(const Body& body1, const Math::Vector2& size1, 
                                const Body& body2, const Math::Vector2& size2)
    {
        float x1 = body1.position.x;
        float y1 = body1.position.y;
        float w1 = size1.x;
        float h1 = size1.y;
        
        float x2 = body2.position.x;
        float y2 = body2.position.y;
        float w2 = size2.x;
        float h2 = size2.y;
        
        // Standard AABB collision detection
        bool collisionX = x1 < x2 + w2 && x1 + w1 > x2;
        bool collisionY = y1 < y2 + h2 && y1 + h1 > y2;
        
        return collisionX && collisionY;
    }
    
    // Helper function to get the normal of an edge (perpendicular vector)
    Math::Vector2 Physics::GetEdgeNormal(const Math::Vector2& edge)
    {
        // Calculate perpendicular vector (normal)
        // Using the formula: (-edge.y, edge.x) or (edge.y, -edge.x)
        // Both are valid normals, just pointing in opposite directions
        return Math::Vector2(-edge.y, edge.x);
    }
    
    // Helper function to calculate dot product
    float Physics::DotProduct(const Math::Vector2& a, const Math::Vector2& b)
    {
        return a.x * b.x + a.y * b.y;
    }
    
    // Project a polygon onto an axis and return min/max values
    std::pair<float, float> Physics::ProjectPolygonOntoAxis(const Polygon& polygon, 
                                                           const Math::Vector2& pos,
                                                           const Math::Vector2& axis)
    {
        if (polygon.empty()) {
            // Return invalid interval for empty polygon to ensure no overlap
            return {std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()};
        }
        
        // Project the first vertex to initialize min and max
        float minProj = DotProduct(polygon[0] + pos, axis);
        float maxProj = minProj;
        
        // Project the rest of the vertices and update min/max
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
    Math::Vector2 Physics::CalculatePolygonCenter(const Polygon& polygon, const Math::Vector2& pos)
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
    bool Physics::CheckOverlap(float min1, float max1, float min2, float max2)
    {
        const float epsilon = 0.0001f; // Small tolerance for floating-point precision
        return !(max1 < min2 - epsilon || max2 < min1 - epsilon);
    }
    
    // Broad-phase collision check before SAT
    bool Physics::CheckAABBCollision(const Math::Vector2& pos1, const Math::Vector2& size1,
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
        
        // Standard AABB collision detection
        bool collisionX = x1 < x2 + w2 && x1 + w1 > x2;
        bool collisionY = y1 < y2 + h2 && y1 + h1 > y2;
        
        return collisionX && collisionY;
    }
    
    // Main SAT-based collision detection function
    Physics::CollisionResult Physics::CheckPolygonCollision(const Polygon& poly1, const Math::Vector2& pos1,
                                                           const Polygon& poly2, const Math::Vector2& pos2)
    {
        // Check if either polygon is empty
        if (poly1.empty() || poly2.empty()) {
            return CollisionResult(false, Math::Vector2(0.0f, 0.0f), 0.0f);
        }
        
        // Broad-phase check using AABBs to avoid expensive SAT if objects are far apart
        // Calculate bounding boxes for both polygons
        if (poly1.size() >= 2 && poly2.size() >= 2) {
            // Calculate AABB for poly1
            Math::Vector2 min1 = poly1[0] + pos1;
            Math::Vector2 max1 = min1;
            for (const auto& vertex : poly1) {
                Math::Vector2 worldVertex = vertex + pos1;
                if (worldVertex.x < min1.x) min1.x = worldVertex.x;
                if (worldVertex.y < min1.y) min1.y = worldVertex.y;
                if (worldVertex.x > max1.x) max1.x = worldVertex.x;
                if (worldVertex.y > max1.y) max1.y = worldVertex.y;
            }
            Math::Vector2 size1 = max1 - min1;
            
            // Calculate AABB for poly2
            Math::Vector2 min2 = poly2[0] + pos2;
            Math::Vector2 max2 = min2;
            for (const auto& vertex : poly2) {
                Math::Vector2 worldVertex = vertex + pos2;
                if (worldVertex.x < min2.x) min2.x = worldVertex.x;
                if (worldVertex.y < min2.y) min2.y = worldVertex.y;
                if (worldVertex.x > max2.x) max2.x = worldVertex.x;
                if (worldVertex.y > max2.y) max2.y = worldVertex.y;
            }
            Math::Vector2 size2 = max2 - min2;
            
            // Early exit if AABBs don't collide
            if (!CheckAABBCollision(min1, size1, min2, size2)) {
                return CollisionResult(false, Math::Vector2(0.0f, 0.0f), 0.0f);
            }
        }
        
        // Get all potential separating axes from both polygons
        std::vector<Math::Vector2> axes;
        const float epsilon = 1e-6f;  // Small epsilon to prevent division by tiny numbers
        const float parallelEpsilon = 0.999f; // For detecting parallel axes
        
        // Get axes from poly1 edges
        for (size_t i = 0; i < poly1.size(); ++i) {
            size_t nextIdx = (i + 1) % poly1.size();
            Math::Vector2 edge = poly1[nextIdx] - poly1[i];
            Math::Vector2 normal = GetEdgeNormal(edge);
            // Normalize the normal vector with epsilon check
            float length = std::sqrt(normal.x * normal.x + normal.y * normal.y);
            if (length > epsilon) {
                normal.x /= length;
                normal.y /= length;
                
                // Check if this axis is parallel to any existing axis (avoid duplicates)
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
            } else {
                continue;  // Skip near-zero length normals
            }
        }
        
        // Get axes from poly2 edges
        for (size_t i = 0; i < poly2.size(); ++i) {
            size_t nextIdx = (i + 1) % poly2.size();
            Math::Vector2 edge = poly2[nextIdx] - poly2[i];
            Math::Vector2 normal = GetEdgeNormal(edge);
            // Normalize the normal vector with epsilon check
            float length = std::sqrt(normal.x * normal.x + normal.y * normal.y);
            if (length > epsilon) {
                normal.x /= length;
                normal.y /= length;
                
                // Check if this axis is parallel to any existing axis (avoid duplicates)
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
            } else {
                continue;  // Skip near-zero length normals
            }
        }
        
        // Track the axis with the minimum overlap (MTV)
        float minOverlap = std::numeric_limits<float>::max();
        Math::Vector2 minAxis = Math::Vector2(0.0f, 0.0f);
        bool foundSeparation = false;
        
        // Check for separation along each axis
        for (const auto& axis : axes) {
            auto proj1 = ProjectPolygonOntoAxis(poly1, pos1, axis);
            auto proj2 = ProjectPolygonOntoAxis(poly2, pos2, axis);
            
            // If projections don't overlap, we've found a separating axis
            if (!CheckOverlap(proj1.first, proj1.second, proj2.first, proj2.second)) {
                return CollisionResult(false, Math::Vector2(0.0f, 0.0f), 0.0f); // No collision
            }
            
            // Calculate the overlap amount using the correct formula
            float overlap = std::min(proj1.second, proj2.second) - std::max(proj1.first, proj2.first);
            
            // Only consider this axis if the overlap is significant (not just floating point error)
            const float epsilon_overlap = 1e-4f;
            if (overlap < epsilon_overlap) {
                continue;  // Skip axes with negligible overlap
            }
            
            // Update minimum translation vector if this is the smallest overlap
            if (overlap < minOverlap) {
                minOverlap = overlap;
                minAxis = axis;
            }
        }
        
        // If we've checked all axes and found no separation, the polygons are colliding
        
        // Determine correct direction of MTV based on relative positions of polygons
        Math::Vector2 center1 = CalculatePolygonCenter(poly1, pos1);
        Math::Vector2 center2 = CalculatePolygonCenter(poly2, pos2);
        
        // Calculate direction from center1 to center2
        Math::Vector2 directionVec = center2 - center1;
        
        // If the dot product of the direction and the axis is negative,
        // flip the axis so it points from poly1 to poly2
        if (DotProduct(directionVec, minAxis) < 0.0f) {
            minAxis = minAxis * -1.0f;
        }
        
        // Return the MTV (axis with minimum overlap)
        return CollisionResult(true, minAxis, minOverlap);
    }
    
    void Physics::ResolveCollision(Body& body1, Body& body2, const CollisionResult& result)
    {
        if (!result.collided) {
            return; // No collision to resolve
        }
        
        // Only resolve if at least one body is not static
        if (body1.isStatic && body2.isStatic) {
            return; // Both bodies are static, no resolution needed
        }
        
        // Calculate total mass of movable bodies
        float mass1 = body1.isStatic ? 0.0f : body1.mass;
        float mass2 = body2.isStatic ? 0.0f : body2.mass;
        float totalMass = mass1 + mass2;
        
        if (totalMass <= 0.0f) {
            return; // Both bodies are effectively static
        }
        
        // Calculate how much to move each body based on their masses
        // Heavier objects move less, lighter objects move more
        float percent1 = (totalMass > 0.0f) ? mass2 / totalMass : 0.5f;
        float percent2 = (totalMass > 0.0f) ? mass1 / totalMass : 0.5f;
        
        // Penetration correction with small bias to prevent jitter
        const float penetrationSlop = 0.01f;
        float correctedPenetration = std::max(result.overlapAmount - penetrationSlop, 0.0f);
        
        // Move bodies apart using MTV
        if (!body1.isStatic) {
            body1.position = body1.position - result.overlapAxis * correctedPenetration * percent1;
        }
        
        if (!body2.isStatic) {
            body2.position = body2.position + result.overlapAxis * correctedPenetration * percent2;
        }
        
        // Correct velocities to prevent objects from moving toward each other
        Math::Vector2 relativeVelocity = body2.velocity - body1.velocity;
        float velocityAlongNormal = DotProduct(relativeVelocity, result.overlapAxis);
        
        // Don't resolve if velocities are separating
        if (velocityAlongNormal > 0.0f) {
            return;
        }
        
        // Calculate restitution (bounciness)
        float restitution = 0.0f; // Perfectly inelastic for now
        float j = -(1.0f + restitution) * velocityAlongNormal;
        j /= (body1.isStatic ? 0.0f : 1.0f / body1.mass) + (body2.isStatic ? 0.0f : 1.0f / body2.mass);
        
        Math::Vector2 impulse = result.overlapAxis * j;
        
        if (!body1.isStatic) {
            body1.velocity = body1.velocity - impulse * (body1.isStatic ? 0.0f : 1.0f / body1.mass);
        }
        
        if (!body2.isStatic) {
            body2.velocity = body2.velocity + impulse * (body2.isStatic ? 0.0f : 1.0f / body2.mass);
        }
    }
    
    bool Physics::IsBodyGrounded(const Body& body, const CollisionResult& collisionResult, float threshold)
    {
        if (!collisionResult.collided) {
            return false;
        }
        
        // A body is grounded if the collision normal points upward (against gravity)
        // In a y-positive-down coordinate system, this means the normal's y-component is negative
        // We check if the absolute value of y-component is greater than the threshold
        return (collisionResult.overlapAxis.y < 0.0f && std::abs(collisionResult.overlapAxis.y) > threshold);
    }
}