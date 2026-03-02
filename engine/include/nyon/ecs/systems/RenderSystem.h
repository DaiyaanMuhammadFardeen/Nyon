#pragma once

#include "nyon/ecs/System.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/graphics/Renderer2D.h"
#include <glad/glad.h>

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
            glClearColor(0.1f, 0.1f, 0.3f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            
            Graphics::Renderer2D::BeginScene();
            
            // Render all entities with render components
            const auto& renderEntities = m_ComponentStore->GetEntitiesWithComponent<RenderComponent>();
            
            for (EntityID entity : renderEntities)
            {
                if (!m_ComponentStore->HasComponent<TransformComponent>(entity)) continue;
                
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entity);
                const auto& render = m_ComponentStore->GetComponent<RenderComponent>(entity);
                
                if (render.visible)
                {
                    // Use interpolated position for smooth rendering
                    Math::Vector2 interpPosition = transform.GetInterpolatedPosition(m_Alpha);
                    float interpRotation = transform.GetInterpolatedRotation(m_Alpha);
                    
                    Graphics::Renderer2D::DrawQuad(
                        interpPosition,
                        render.size,
                        render.origin,
                        render.color
                    );
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