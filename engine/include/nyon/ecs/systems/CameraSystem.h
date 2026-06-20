#pragma once
#include "nyon/ecs/System.h"
#include "nyon/ecs/components/CameraComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/graphics/Renderer2D.h"
#include "nyon/core/Application.h"
#include <GLFW/glfw3.h>
#include <vector>
namespace Nyon::ECS {
    class CameraSystem : public System {
    public:
        CameraSystem();
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override;
        void Update(float deltaTime) override;
        EntityID GetActiveCameraEntity() const { return m_ActiveCameraEntity; }
        const CameraComponent* GetActiveCamera() const;
        void SetActiveCamera(EntityID entity);
        void SelectBestCamera();
        void UpdateCameraFollow(float deltaTime);
        void SyncWithRenderer(float screenWidth, float screenHeight);
    private:
        EntityID m_ActiveCameraEntity = INVALID_ENTITY;
        ComponentStore* m_ComponentStore = nullptr; }; }
