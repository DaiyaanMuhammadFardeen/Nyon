#pragma once
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include <string>
namespace Nyon::ECS {
    /**
     * @brief Render component for visual representation of entities.
     * 
     * Holds rendering properties like size, color, and visual state.
     */
    struct RenderComponent {
        enum class ShapeType {
            Rectangle = 0,
            Circle = 1,
            Polygon = 2
        };
        Math::Vector2 size = {32.0f, 32.0f};     
        Math::Vector3 color = {1.0f, 1.0f, 1.0f};  
        Math::Vector2 origin = {0.0f, 0.0f};     
        ShapeType shapeType = ShapeType::Rectangle;  
        std::string texturePath = "";            
        bool visible = true;                     
        int layer = 0;                           
        RenderComponent() = default;
        RenderComponent(const Math::Vector2& sz) : size(sz) {}
        RenderComponent(const Math::Vector2& sz, const Math::Vector3& col) 
            : size(sz), color(col) {}
        RenderComponent(const Math::Vector2& sz, const Math::Vector3& col, const std::string& tex)
            : size(sz), color(col), texturePath(tex) {}
        RenderComponent(const Math::Vector2& sz, const Math::Vector3& col, ShapeType shape)
            : size(sz), color(col), shapeType(shape) {}
    };
}