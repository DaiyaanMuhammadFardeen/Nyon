#pragma once

#include "nyon/ecs/System.h"
#include "nyon/ecs/components/CameraComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/graphics/Renderer2D.h"
#include "nyon/core/Application.h"
#include <GLFW/glfw3.h>
#include <vector>

namespace Nyon::ECS
{
    /**
     * @brief System for managing cameras in the ECS
     * 
     * This system:
     * - Tracks all camera entities
     * - Selects the active camera based on priority
     * - Updates camera transforms based on follow targets
     * - Synchronizes with Renderer2D's active camera
     */
    class CameraSystem : public System
    {
    public:
        CameraSystem();
        
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override;
        void Update(float deltaTime) override;
        
        // Get the active camera entity
        EntityID GetActiveCameraEntity() const { return m_ActiveCameraEntity; }
        
        // Get active camera component
        const CameraComponent* GetActiveCamera() const;
        
        // Manually set active camera by entity
        void SetActiveCamera(EntityID entity);
        
        // Find and select the best camera based on priority
        void SelectBestCamera();
        
        // Update camera follow logic
        void UpdateCameraFollow(float deltaTime);
        
        // Sync with Renderer2D
        void SyncWithRenderer(float screenWidth, float screenHeight);
        
    private:
        EntityID m_ActiveCameraEntity = INVALID_ENTITY;
        ComponentStore* m_ComponentStore = nullptr;
    };
}
