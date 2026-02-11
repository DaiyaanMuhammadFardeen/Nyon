#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include <string>

namespace Nyon::ECS
{
    /**
     * @brief Render component for visual representation of entities.
     * 
     * Holds rendering properties like size, color, and visual state.
     */
    struct RenderComponent
    {
        Math::Vector2 size = {32.0f, 32.0f};    // Visual size
        Math::Vector3 color = {1.0f, 1.0f, 1.0f}; // Render color
        Math::Vector2 origin = {0.0f, 0.0f};    // Origin point for rendering
        std::string texturePath = "";           // Texture file path (empty = solid color)
        bool visible = true;                    // Whether to render this entity
        int layer = 0;                          // Render layer (higher = front)
        
        RenderComponent() = default;
        RenderComponent(const Math::Vector2& sz) : size(sz) {}
        RenderComponent(const Math::Vector2& sz, const Math::Vector3& col) 
            : size(sz), color(col) {}
        RenderComponent(const Math::Vector2& sz, const Math::Vector3& col, const std::string& tex)
            : size(sz), color(col), texturePath(tex) {}
    };
}