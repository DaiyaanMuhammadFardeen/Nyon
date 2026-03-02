#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"

namespace Nyon::ECS
{
    /**
     * @brief Transform component holding position, rotation, and scale.
     * 
     * Separated from physics to allow non-physical entities and hierarchical transforms.
     */
    struct TransformComponent
    {
        Math::Vector2 position = {0.0f, 0.0f};
        Math::Vector2 previousPosition = {0.0f, 0.0f}; // For interpolation
        Math::Vector2 scale = {1.0f, 1.0f};
        float rotation = 0.0f; // in radians
        float previousRotation = 0.0f; // For interpolation
        
        TransformComponent() = default;
        TransformComponent(const Math::Vector2& pos) : position(pos), previousPosition(pos) {}
        TransformComponent(const Math::Vector2& pos, const Math::Vector2& scl) 
            : position(pos), previousPosition(pos), scale(scl) {}
        TransformComponent(const Math::Vector2& pos, const Math::Vector2& scl, float rot)
            : position(pos), previousPosition(pos), scale(scl), rotation(rot), previousRotation(rot) {}
        
        // Update previous state before physics update
        void PrepareForUpdate()
        {
            previousPosition = position;
            previousRotation = rotation;
        }
        
        // Get interpolated position for smooth rendering
        Math::Vector2 GetInterpolatedPosition(float alpha) const
        {
            return previousPosition + (position - previousPosition) * alpha;
        }
        
        // Get interpolated rotation for smooth rendering
        float GetInterpolatedRotation(float alpha) const
        {
            return previousRotation + (rotation - previousRotation) * alpha;
        }
    };
}