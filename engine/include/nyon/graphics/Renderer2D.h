#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include <vector>

namespace Nyon::Graphics
{
    struct Vertex
    {
        float x, y;
        float r, g, b;
    };

    class Renderer2D
    {
    public:
        static void Init();
        static void Shutdown();
        
        static void BeginScene();
        static void EndScene();
        
        static void DrawQuad(const Math::Vector2& position, const Math::Vector2& size, const Math::Vector2& origin, const Math::Vector3& color);
        static void DrawLine(const Math::Vector2& start, const Math::Vector2& end, const Math::Vector3& color);
        
        static void Flush();
    };
}