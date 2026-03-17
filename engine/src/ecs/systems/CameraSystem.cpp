#include "nyon/ecs/systems/CameraSystem.h"
#include "nyon/core/Application.h"
#include <algorithm>
#include <GLFW/glfw3.h>
#include <cfloat>

namespace Nyon::ECS
{
    CameraSystem::CameraSystem() : m_ActiveCameraEntity(INVALID_ENTITY), m_ComponentStore(nullptr)
    {
    }
    
    void CameraSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore)
    {
        m_ComponentStore = &componentStore;
        
        // Select initial camera if cameras exist
        SelectBestCamera();
    }
    
    void CameraSystem::Update(float deltaTime)
    {
        if (!m_ComponentStore) return;
        
        // Update camera follow logic
        UpdateCameraFollow(deltaTime);
        
        // Sync with renderer
        SyncWithRenderer(1280.0f, 720.0f); // Default dimensions, will be updated by application
    }
    
    const CameraComponent* CameraSystem::GetActiveCamera() const
    {
        if (!m_ComponentStore || m_ActiveCameraEntity == INVALID_ENTITY)
            return nullptr;
            
        if (!m_ComponentStore->HasComponent<CameraComponent>(m_ActiveCameraEntity))
            return nullptr;
            
        return &m_ComponentStore->GetComponent<CameraComponent>(m_ActiveCameraEntity);
    }
    
    void CameraSystem::SetActiveCamera(EntityID entity)
    {
        if (!m_ComponentStore) return;
        
        if (m_ComponentStore->HasComponent<CameraComponent>(entity))
        {
            m_ActiveCameraEntity = entity;
            
            // Mark all other cameras as inactive
            const auto& cameraEntities = m_ComponentStore->GetEntitiesWithComponent<CameraComponent>();
            for (auto camEntity : cameraEntities)
            {
                if (camEntity != m_ActiveCameraEntity)
                {
                    auto& cam = m_ComponentStore->GetComponent<CameraComponent>(camEntity);
                    cam.isActive = false;
                }
            }
            
            // Mark active camera as active
            auto& activeCam = m_ComponentStore->GetComponent<CameraComponent>(m_ActiveCameraEntity);
            activeCam.isActive = true;
        }
    }
    
    void CameraSystem::SelectBestCamera()
    {
        if (!m_ComponentStore) return;
        
        const auto& cameraEntities = m_ComponentStore->GetEntitiesWithComponent<CameraComponent>();
        if (cameraEntities.empty()) return;
        
        // Find camera with highest priority
        EntityID bestEntity = INVALID_ENTITY;
        float highestPriority = -FLT_MAX;
        
        for (auto entity : cameraEntities)
        {
            const auto& cam = m_ComponentStore->GetComponent<CameraComponent>(entity);
            if (cam.priority > highestPriority)
            {
                highestPriority = cam.priority;
                bestEntity = entity;
            }
        }
        
        if (bestEntity != INVALID_ENTITY)
        {
            SetActiveCamera(bestEntity);
        }
    }
    
    void CameraSystem::UpdateCameraFollow(float deltaTime)
    {
        if (!m_ComponentStore || m_ActiveCameraEntity == INVALID_ENTITY) return;
        
        auto& camera = m_ComponentStore->GetComponent<CameraComponent>(m_ActiveCameraEntity);
        
        if (!camera.followTarget || camera.targetEntity == INVALID_ENTITY) return;
        
        // Check if target entity has a transform
        if (!m_ComponentStore->HasComponent<TransformComponent>(camera.targetEntity))
            return;
        
        const auto& targetTransform = m_ComponentStore->GetComponent<TransformComponent>(camera.targetEntity);
        
        // Calculate target position with offset
        Math::Vector2 targetPos = targetTransform.position + camera.followOffset;
        
        // Smooth follow using lerp
        if (camera.followSmoothness > 0.0f)
        {
            float t = 1.0f - std::pow(1.0f - camera.followSmoothness, deltaTime * 60.0f);
            camera.camera.position = Math::Vector2::Lerp(camera.camera.position, targetPos, t);
        }
        else
        {
            // Instant follow
            camera.camera.position = targetPos;
        }
    }
    
    void CameraSystem::SyncWithRenderer(float screenWidth, float screenHeight)
    {
        const CameraComponent* camera = GetActiveCamera();
        if (!camera) return;
        
        // Update Renderer2D's camera with ECS camera data
        Graphics::Camera2D rendererCamera;
        rendererCamera.position = camera->camera.position;
        rendererCamera.zoom = camera->camera.zoom;
        rendererCamera.rotation = camera->camera.rotation;
        rendererCamera.nearPlane = camera->camera.nearPlane;
        rendererCamera.farPlane = camera->camera.farPlane;
        
        // Update screen dimensions in camera component
        auto& mutableCamera = m_ComponentStore->GetComponent<CameraComponent>(m_ActiveCameraEntity);
        mutableCamera.UpdateScreenDimensions(screenWidth, screenHeight);
        
        // Begin scene with updated camera
        Graphics::Renderer2D::BeginScene(rendererCamera);
    }
}
