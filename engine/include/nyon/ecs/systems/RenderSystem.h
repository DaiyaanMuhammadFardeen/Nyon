#pragma once

#include "nyon/ecs/System.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/CameraComponent.h"
#include "nyon/graphics/Renderer2D.h"
#include "nyon/core/Application.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

namespace Nyon::ECS
{
    /**
     * @brief Rendering system that draws all entities with RenderComponent.
     * 
     * Handles interpolation between physics frames for smooth rendering.
     */
    class RenderSystem : public System
    {
    public:
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override
        {
            System::Initialize(entityManager, componentStore);
            // Renderer2D::Init() is called in Application::Init()
            // Avoid duplicate initialization
            m_Alpha = 1.0f; // Default to current state
        }
        
        void Update(float deltaTime) override
        {
            if (!m_EntityManager || !m_ComponentStore) return;
            
            // Clear screen with dark blue background
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            
            // Find and use the active camera
            const CameraComponent* activeCamera = nullptr;
            const auto& cameraEntities = m_ComponentStore->GetEntitiesWithComponent<CameraComponent>();
            
            for (EntityID camEntity : cameraEntities)
            {
                const auto& camera = m_ComponentStore->GetComponent<CameraComponent>(camEntity);
                if (camera.isActive)
                {
                    activeCamera = &camera;
                    break;
                }
            }
            
            // Update camera screen dimensions from window
            GLFWwindow* window = nullptr;
            try { window = Application::Get().GetWindow(); } catch (...) {}
            int width = 1280, height = 720;
            if (window) glfwGetFramebufferSize(window, &width, &height);
            
            if (activeCamera)
            {
                // Update camera's cached dimensions
                const_cast<CameraComponent*>(activeCamera)->UpdateScreenDimensions(static_cast<float>(width), static_cast<float>(height));
                // Use camera's view-projection matrix
                Graphics::Renderer2D::BeginScene(activeCamera->camera);
            }
            else
            {
                // No camera - use default orthographic projection based on window size
                Graphics::Camera2D defaultCamera;
                defaultCamera.position = {0.0f, 0.0f};
                defaultCamera.zoom = 1.0f;
                defaultCamera.rotation = 0.0f;
                Graphics::Renderer2D::BeginScene(defaultCamera);
            }
            
            // Render all entities with render components
            const auto& renderEntities = m_ComponentStore->GetEntitiesWithComponent<RenderComponent>();
            
#ifdef _DEBUG
            static int s_RenderDebugCounter = 0;
            if (++s_RenderDebugCounter >= 10) {
                std::cerr << "[RENDER@" << s_RenderDebugCounter << "] Drawing " << renderEntities.size() << " entities:" << std::endl;
                s_RenderDebugCounter = 0;
            }
#endif
            
            for (EntityID entity : renderEntities)
            {
                if (!m_ComponentStore->HasComponent<TransformComponent>(entity)) continue;
                
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entity);
                const auto& render = m_ComponentStore->GetComponent<RenderComponent>(entity);
                
#ifdef _DEBUG
                static int s_RenderEntityCounter = 0;
                if (++s_RenderEntityCounter >= 10) {
                    std::cerr << "  Entity[" << entity << "] pos=(" << transform.position.x << "," << transform.position.y 
                              << ") size=(" << render.size.x << "," << render.size.y 
                              << ") color=(" << render.color.x << "," << render.color.y << "," << render.color.z 
                              << ") shape=" << (int)render.shapeType 
                              << ") visible=" << render.visible << std::endl;
                    s_RenderEntityCounter = 0;
                }
#endif
                
                if (render.visible)
                {
                    // Use interpolated position for smooth rendering
                    Math::Vector2 interpPosition = transform.GetInterpolatedPosition(m_Alpha);
                    float interpRotation = transform.GetInterpolatedRotation(m_Alpha);
                    
                    // Draw based on shape type
                    switch (render.shapeType)
                    {
                        case RenderComponent::ShapeType::Circle:
                        {
                            // For circles, use the smaller dimension as diameter
                            float radius = std::min(render.size.x, render.size.y) * 0.5f;
                            Graphics::Renderer2D::DrawSolidCircle(
                                interpPosition,
                                radius,
                                render.color
                            );
                            break;
                        }
                        
                        case RenderComponent::ShapeType::Rectangle:
                        default:
                            Graphics::Renderer2D::DrawQuad(
                                interpPosition,
                                render.size,
                                render.origin,
                                render.color,
                                interpRotation
                            );
                            break;
                    }
                }
            }
            
            Graphics::Renderer2D::EndScene();
        }
        
        // Set interpolation alpha value (0.0 = previous state, 1.0 = current state)
        void SetInterpolationAlpha(float alpha) { m_Alpha = alpha; }
        
        void Shutdown() override
        {
            Graphics::Renderer2D::Shutdown();
        }
        
    private:
        float m_Alpha = 1.0f; // Interpolation factor between previous and current state
    };
}
