#include "nyon/utils/Physics.h"

namespace Nyon::Utils
{
    const float Physics::Gravity = 980.0f; // pixels/s^2
    
    void Physics::ApplyGravity(Body& body)
    {
        if (!body.isStatic)
        {
            body.acceleration.y += Gravity;
        }
    }
    
    void Physics::UpdateBody(Body& body, float deltaTime)
    {
        if (!body.isStatic)
        {
            // Apply acceleration to velocity
            body.velocity.x += body.acceleration.x * deltaTime;
            body.velocity.y += body.acceleration.y * deltaTime;
            
            // Apply velocity to position
            body.position.x += body.velocity.x * deltaTime;
            body.position.y += body.velocity.y * deltaTime;
            
            // Reset acceleration after applying
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
}