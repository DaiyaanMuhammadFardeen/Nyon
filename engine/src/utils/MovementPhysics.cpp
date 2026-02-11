#include "nyon/utils/MovementPhysics.h"
#include <cmath>

namespace Nyon::Utils
{
    void MovementPhysics::ApplyForce(Physics::Body& body, const Math::Vector2& force)
    {
        if (!body.isStatic) {
            // F = ma, so a = F/m
            body.acceleration.x += force.x / body.mass;
            body.acceleration.y += force.y / body.mass;
        }
    }
    
    void MovementPhysics::ApplyImpulse(Physics::Body& body, const Math::Vector2& impulse)
    {
        if (!body.isStatic) {
            // Apply impulse directly to velocity
            body.velocity.x += impulse.x;
            body.velocity.y += impulse.y;
        }
    }
    
    void MovementPhysics::SetVelocity(Physics::Body& body, const Math::Vector2& velocity)
    {
        if (!body.isStatic) {
            body.velocity = velocity;
        }
    }
    
    float MovementPhysics::GetSpeed(const Physics::Body& body)
    {
        return std::sqrt(body.velocity.x * body.velocity.x + body.velocity.y * body.velocity.y);
    }
    
    float MovementPhysics::GetVelocityAngle(const Physics::Body& body)
    {
        return std::atan2(body.velocity.y, body.velocity.x);
    }
    
    void MovementPhysics::LimitSpeed(Physics::Body& body, float maxSpeed)
    {
        float currentSpeed = GetSpeed(body);
        if (currentSpeed > maxSpeed && currentSpeed > 0.0f) {
            float ratio = maxSpeed / currentSpeed;
            body.velocity.x *= ratio;
            body.velocity.y *= ratio;
        }
    }
}