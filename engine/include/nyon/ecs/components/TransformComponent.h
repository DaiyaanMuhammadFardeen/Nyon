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
        Math::Vector2 scale = {1.0f, 1.0f};
        float rotation = 0.0f; // in radians
        
        TransformComponent() = default;
        TransformComponent(const Math::Vector2& pos) : position(pos) {}
        TransformComponent(const Math::Vector2& pos, const Math::Vector2& scl) 
            : position(pos), scale(scl) {}
        TransformComponent(const Math::Vector2& pos, const Math::Vector2& scl, float rot)
            : position(pos), scale(scl), rotation(rot) {}
    };
}