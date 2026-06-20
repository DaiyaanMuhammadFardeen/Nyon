#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <glm/glm.hpp>

namespace Nyon::Graphics
{
    /**
     * @brief Comprehensive 2D shape types matching physics colliders
     */
    enum class ShapeType
    {
        Circle,
        Polygon,
        Capsule,
        Segment,
        Chain,
        Rectangle,
        Ellipse,
        Arc,
        Sector
    };

    /**
     * @brief Vertex with position, color, and texture coordinates
     */
    struct Vertex
    {
        float x, y;      // Position
        float r, g, b;   // Color
        float u, v;      // Texture coordinates
        float nx, ny;    // Normal (for lighting effects)
        
        Vertex() : x(0), y(0), r(1), g(1), b(1), u(0), v(0), nx(0), ny(0) {}
        Vertex(float px, float py, float cr, float cg, float cb)
            : x(px), y(py), r(cr), g(cg), b(cb), u(0), v(0), nx(0), ny(0) {}
        Vertex(float px, float py, float cr, float cg, float cb, float tu, float tv)
            : x(px), y(py), r(cr), g(cg), b(cb), u(tu), v(tv), nx(0), ny(0) {}
    };

    /**
     * @brief Shape description for rendering
     */
    struct ShapeDescriptor
    {
        ShapeType type = ShapeType::Circle;
        Math::Vector2 position = {0.0f, 0.0f};
        float rotation = 0.0f;
        Math::Vector2 scale = {1.0f, 1.0f};
        Math::Vector3 color = {1.0f, 1.0f, 1.0f};
        float thickness = 1.0f;  // For line-based shapes
        bool filled = true;
        int segments = 32;       // For curved shapes
        
        // Shape-specific parameters
        union ShapeParams
        {
            struct { float radius; } circle;
            struct { float width, height; } rect;
            struct { float radiusX, radiusY; } ellipse;
            struct { float innerRadius, outerRadius, angleStart, angleEnd; } sector;
            struct { float startX, startY, endX, endY; } segment;
            struct { float centerX, centerY, radius, angleStart, angleEnd; } arc;
            
            ShapeParams() : circle{0.0f} {}
        } params;
        
        std::vector<Math::Vector2> vertices; // For polygons and chains
    };

    /**
     * @brief Camera for 2D rendering with zoom and pan
     */
    struct Camera2D
    {
        Math::Vector2 position = {0.0f, 0.0f};
        float zoom = 1.0f;
        float rotation = 0.0f;
        float nearPlane = -1.0f;
        float farPlane = 1.0f;
        
        glm::mat4 GetViewMatrix() const;
        glm::mat4 GetProjectionMatrix(float screenWidth, float screenHeight) const;
        glm::mat4 GetViewProjectionMatrix(float screenWidth, float screenHeight) const;
        
        Math::Vector2 ScreenToWorld(Math::Vector2 screenPos, float screenWidth, float screenHeight) const;
        Math::Vector2 WorldToScreen(Math::Vector2 worldPos, float screenWidth, float screenHeight) const;
    };

    /**
     * @brief Advanced 2D renderer with full OpenGL pipeline
     */
    class Renderer2D
    {
    public:
        static void Init();
        static void Shutdown();
        
        static void BeginScene(const Camera2D& camera = Camera2D());
        static void EndScene();
        
        // Get the current active camera
        static const Camera2D& GetActiveCamera();
        
        // Set screen dimensions for VP matrix calculation
        static void SetScreenDimensions(float width, float height);
        
        // === BASIC SHAPES ===
        static void DrawQuad(const Math::Vector2& position, 
                            const Math::Vector2& size, 
                            const Math::Vector2& origin,
                            const Math::Vector3& color,
                            float rotation = 0.0f);
        
        static void DrawCircle(const Math::Vector2& center, 
                              float radius, 
                              const Math::Vector3& color, 
                              int segments = 32);
        
        static void DrawPolygon(const std::vector<Math::Vector2>& vertices, 
                               const Math::Vector3& color);
        
        static void DrawLine(const Math::Vector2& start, 
                            const Math::Vector2& end, 
                            const Math::Vector3& color,
                            float thickness = 1.0f);
        
        // === ADVANCED SHAPES ===
        static void DrawCapsule(const Math::Vector2& center1,
                               const Math::Vector2& center2,
                               float radius,
                               const Math::Vector3& color,
                               int segments = 16);
        
        static void DrawSegment(const Math::Vector2& point1,
                               const Math::Vector2& point2,
                               float thickness,
                               const Math::Vector3& color);
        
        static void DrawChain(const std::vector<Math::Vector2>& vertices,
                             const Math::Vector3& color,
                             float thickness = 1.0f,
                             bool closed = false);
        
        static void DrawEllipse(const Math::Vector2& center,
                               float radiusX,
                               float radiusY,
                               const Math::Vector3& color,
                               int segments = 32);
        
        static void DrawArc(const Math::Vector2& center,
                           float radius,
                           float angleStart,
                           float angleEnd,
                           const Math::Vector3& color,
                           float thickness = 1.0f,
                           int segments = 32);
        
        static void DrawSector(const Math::Vector2& center,
                              float radius,
                              float angleStart,
                              float angleEnd,
                              const Math::Vector3& color,
                              int segments = 32);
        
        // === FILLED SHAPES ===
        static void DrawSolidCircle(const Math::Vector2& center,
                                   float radius,
                                   const Math::Vector3& color,
                                   int segments = 32);
        
        static void DrawSolidPolygon(const std::vector<Math::Vector2>& vertices,
                                    const Math::Vector3& color);
        
        static void DrawSolidCapsule(const Math::Vector2& center1,
                                    const Math::Vector2& center2,
                                    float radius,
                                    const Math::Vector3& color,
                                    int segments = 16);
        
        static void DrawSolidEllipse(const Math::Vector2& center,
                                    float radiusX,
                                    float radiusY,
                                    const Math::Vector3& color,
                                    int segments = 32);
        
        static void DrawSolidSector(const Math::Vector2& center,
                                   float radius,
                                   float angleStart,
                                   float angleEnd,
                                   const Math::Vector3& color,
                                   int segments = 32);
        
        // === GENERIC SHAPE ===
        static void DrawShape(const ShapeDescriptor& shape);
        
        // === PHYSICS DEBUG VISUALIZATION ===
        static void DrawManifold(const Math::Vector2& contactPoint,
                                const Math::Vector2& normal,
                                float separation,
                                bool isTouching);
        
        static void DrawAABB(const Math::Vector2& min,
                            const Math::Vector2& max,
                            const Math::Vector3& color);
        
        static void DrawTransform(const Math::Vector2& position,
                                 float rotation,
                                 float scale = 1.0f);
        
        // === BATCH CONTROL ===
        static void Flush();
        static void SetLineWidth(float width);
        static float GetLineWidth();
        
        // === STATE MANAGEMENT ===
        static void EnableBlending(bool enable);
        static void EnableDepthTest(bool enable);
        static void EnableCulling(bool enable);
        
    private:
        struct Impl;
        static std::unique_ptr<Impl> s_Instance;
    };
}
