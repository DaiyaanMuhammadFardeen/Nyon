#include "nyon/utils/Physics.h"
#include <algorithm>
#include <cmath>

namespace Nyon::Utils
{
    const float Physics::Gravity = 980.0f; // pixels/s^2
    
    void Physics::ApplyGravity(Body& body)
    {
        // This function is kept for backward compatibility but should not be used
        // Gravity is now applied directly in UpdateBody to avoid accumulation
    }
    
    void Physics::UpdateBody(Body& body, float deltaTime)
    {
        if (!body.isStatic)
        {
            // Apply gravity directly to velocity instead of accumulating acceleration
            body.velocity.y += Gravity * deltaTime;
            
            // Apply other accelerations to velocity
            body.velocity.x += body.acceleration.x * deltaTime;
            body.velocity.y += body.acceleration.y * deltaTime;
            
            // Apply velocity to position (Symplectic Euler)
            body.position.x += body.velocity.x * deltaTime;
            body.position.y += body.velocity.y * deltaTime;
            
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
        bool collisionY = y1 < y2 + h2 && y1 + h1 > h2;
        
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
            return {0.0f, 0.0f};
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
    
    // Check if two projected intervals overlap
    bool Physics::CheckOverlap(float min1, float max1, float min2, float max2)
    {
        return !(max1 < min2 || max2 < min1);
    }
    
    // Main SAT-based collision detection function
    bool Physics::CheckPolygonCollision(const Polygon& poly1, const Math::Vector2& pos1,
                                       const Polygon& poly2, const Math::Vector2& pos2)
    {
        // Check if either polygon is empty
        if (poly1.empty() || poly2.empty()) {
            return false;
        }
        
        // Get all potential separating axes from both polygons
        std::vector<Math::Vector2> axes;
        
        // Get axes from poly1 edges
        for (size_t i = 0; i < poly1.size(); ++i) {
            size_t nextIdx = (i + 1) % poly1.size();
            Math::Vector2 edge = poly1[nextIdx] - poly1[i];
            Math::Vector2 normal = GetEdgeNormal(edge);
            // Normalize the normal vector
            float length = std::sqrt(normal.x * normal.x + normal.y * normal.y);
            if (length > 0.0f) {
                normal.x /= length;
                normal.y /= length;
            }
            axes.push_back(normal);
        }
        
        // Get axes from poly2 edges
        for (size_t i = 0; i < poly2.size(); ++i) {
            size_t nextIdx = (i + 1) % poly2.size();
            Math::Vector2 edge = poly2[nextIdx] - poly2[i];
            Math::Vector2 normal = GetEdgeNormal(edge);
            // Normalize the normal vector
            float length = std::sqrt(normal.x * normal.x + normal.y * normal.y);
            if (length > 0.0f) {
                normal.x /= length;
                normal.y /= length;
            }
            axes.push_back(normal);
        }
        
        // Check for separation along each axis
        for (const auto& axis : axes) {
            auto proj1 = ProjectPolygonOntoAxis(poly1, pos1, axis);
            auto proj2 = ProjectPolygonOntoAxis(poly2, pos2, axis);
            
            // If projections don't overlap, we've found a separating axis
            if (!CheckOverlap(proj1.first, proj1.second, proj2.first, proj2.second)) {
                return false; // No collision
            }
        }
        
        // If we've checked all axes and found no separation, the polygons are colliding
        return true;
    }
}