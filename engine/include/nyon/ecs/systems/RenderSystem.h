#pragma once

#include "nyon/ecs/System.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/graphics/Renderer2D.h"

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
            Graphics::Renderer2D::Init();
        }
        
        void Update(float deltaTime) override
        {
            if (!m_EntityManager || !m_ComponentStore) return;
            
            // Clear screen
            glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
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
                    Graphics::Renderer2D::DrawQuad(
                        transform.position,
                        render.size,
                        render.origin,
                        render.color
                    );
                }
            }
            
            Graphics::Renderer2D::EndScene();
        }
        
        void Shutdown() override
        {
            Graphics::Renderer2D::Shutdown();
        }
    };
}