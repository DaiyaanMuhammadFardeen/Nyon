#pragma once

#include "nyon/graphics/Renderer2D.h"
#include "nyon/math/Vector2.h"
#include "nyon/ecs/EntityManager.h"

namespace Nyon::ECS
{
    /**
     * @brief Camera component for 2D rendering with zoom, pan, and rotation support.
     * 
     * This component wraps Graphics::Camera2D and provides ECS integration.
     * Multiple cameras can exist in the scene, with one active camera used for rendering.
     */
    struct CameraComponent
    {
        Graphics::Camera2D camera;
        
        // Camera metadata
        bool isActive = true;              // Whether this is the currently active camera
        float priority = 0.0f;             // Higher priority cameras take precedence
        int layer = 0;                     // Camera rendering layer
        
        // Viewport settings (normalized coordinates 0-1)
        float viewportX = 0.0f;
        float viewportY = 0.0f;
        float viewportWidth = 1.0f;
        float viewportHeight = 1.0f;
        
        // Camera behavior
        bool followTarget = false;         // Whether to follow a target entity
        EntityID targetEntity = INVALID_ENTITY;  // Entity to follow
        Math::Vector2 followOffset = {0.0f, 0.0f};  // Offset from target
        float followSmoothness = 0.0f;     // 0 = instant, >0 = lerp factor
        
        // Screen dimensions cache (updated by render system)
        mutable float cachedScreenWidth = 800.0f;
        mutable float cachedScreenHeight = 600.0f;
        
        // === CONSTRUCTORS ===
        CameraComponent() = default;
        
        CameraComponent(float screenWidth, float screenHeight)
        {
            cachedScreenWidth = screenWidth;
            cachedScreenHeight = screenHeight;
        }
        
        // === CAMERA CONTROL ===
        void SetPosition(const Math::Vector2& pos)
        {
            camera.position = pos;
        }
        
        void Move(const Math::Vector2& delta)
        {
            camera.position = camera.position + delta;
        }
        
        void SetZoom(float zoom)
        {
            camera.zoom = zoom;
        }
        
        void Zoom(float delta)
        {
            camera.zoom += delta;
        }
        
        void SetRotation(float angle)
        {
            camera.rotation = angle;
        }
        
        void Rotate(float delta)
        {
            camera.rotation += delta;
        }
        
        // === MATRIX ACCESSORS ===
        glm::mat4 GetViewMatrix() const
        {
            return camera.GetViewMatrix();
        }
        
        glm::mat4 GetProjectionMatrix() const
        {
            return camera.GetProjectionMatrix(cachedScreenWidth, cachedScreenHeight);
        }
        
        glm::mat4 GetViewProjectionMatrix() const
        {
            return camera.GetViewProjectionMatrix(cachedScreenWidth, cachedScreenHeight);
        }
        
        // === COORDINATE CONVERSION ===
        Math::Vector2 ScreenToWorld(const Math::Vector2& screenPos) const
        {
            return camera.ScreenToWorld(screenPos, cachedScreenWidth, cachedScreenHeight);
        }
        
        Math::Vector2 WorldToScreen(const Math::Vector2& worldPos) const
        {
            return camera.WorldToScreen(worldPos, cachedScreenWidth, cachedScreenHeight);
        }
        
        // === UTILITY ===
        void UpdateScreenDimensions(float width, float height)
        {
            cachedScreenWidth = width;
            cachedScreenHeight = height;
        }
    };
}
