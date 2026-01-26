#pragma once

#include "nyon/math/Vector2.h"

namespace Nyon::Utils
{
    class Physics
    {
    public:
        static const float Gravity;
        
        struct Body
        {
            Math::Vector2 position;
            Math::Vector2 velocity;
            Math::Vector2 acceleration;
            float mass;
            bool isStatic;
            
            Body() : position(0.0f, 0.0f), velocity(0.0f, 0.0f), acceleration(0.0f, 0.0f), mass(1.0f), isStatic(false) {}
        };
        
        static void ApplyGravity(Body& body);
        static void UpdateBody(Body& body, float deltaTime);
        static bool CheckCollision(const Body& body1, const Math::Vector2& size1, 
                                  const Body& body2, const Math::Vector2& size2);
    };
}