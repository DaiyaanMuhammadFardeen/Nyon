#include "nyon/ecs/systems/CameraSystem.h"
#include "nyon/core/Application.h"
#include <algorithm>
#include <GLFW/glfw3.h>
#include <cfloat>
namespace Nyon::ECS {
    CameraSystem::CameraSystem() : m_ActiveCameraEntity(INVALID_ENTITY), m_ComponentStore(nullptr) { }
    void CameraSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore) {
        m_ComponentStore = &componentStore;
        SelectBestCamera(); }
    void CameraSystem::Update(float deltaTime) {
        if (!m_ComponentStore) return;
        UpdateCameraFollow(deltaTime);
        SyncWithRenderer(1280.0f, 720.0f);   }
    const CameraComponent* CameraSystem::GetActiveCamera() const {
        if (!m_ComponentStore || m_ActiveCameraEntity == INVALID_ENTITY)
            return nullptr;
        if (!m_ComponentStore->HasComponent<CameraComponent>(m_ActiveCameraEntity))
            return nullptr;
        return &m_ComponentStore->GetComponent<CameraComponent>(m_ActiveCameraEntity); }
    void CameraSystem::SetActiveCamera(EntityID entity) {
        if (!m_ComponentStore) return;
        if (m_ComponentStore->HasComponent<CameraComponent>(entity)) {
            m_ActiveCameraEntity = entity;
            const auto& cameraEntities = m_ComponentStore->GetEntitiesWithComponent<CameraComponent>();
            for (auto camEntity : cameraEntities) {
                if (camEntity != m_ActiveCameraEntity) {
                    auto& cam = m_ComponentStore->GetComponent<CameraComponent>(camEntity);
                    cam.isActive = false; } }
            auto& activeCam = m_ComponentStore->GetComponent<CameraComponent>(m_ActiveCameraEntity);
            activeCam.isActive = true; } }
    void CameraSystem::SelectBestCamera() {
        if (!m_ComponentStore) return;
        const auto& cameraEntities = m_ComponentStore->GetEntitiesWithComponent<CameraComponent>();
        if (cameraEntities.empty()) return;
        EntityID bestEntity = INVALID_ENTITY;
        float highestPriority = -FLT_MAX;
        for (auto entity : cameraEntities) {
            const auto& cam = m_ComponentStore->GetComponent<CameraComponent>(entity);
            if (cam.priority > highestPriority) {
                highestPriority = cam.priority;
                bestEntity = entity; } }
        if (bestEntity != INVALID_ENTITY) {
            SetActiveCamera(bestEntity); } }
    void CameraSystem::UpdateCameraFollow(float deltaTime) {
        if (!m_ComponentStore || m_ActiveCameraEntity == INVALID_ENTITY) return;
        auto& camera = m_ComponentStore->GetComponent<CameraComponent>(m_ActiveCameraEntity);
        if (!camera.followTarget || camera.targetEntity == INVALID_ENTITY) return;
        if (!m_ComponentStore->HasComponent<TransformComponent>(camera.targetEntity))
            return;
        const auto& targetTransform = m_ComponentStore->GetComponent<TransformComponent>(camera.targetEntity);
        Math::Vector2 targetPos = targetTransform.position + camera.followOffset;
        if (camera.followSmoothness > 0.0f) {
            float t = 1.0f - std::pow(1.0f - camera.followSmoothness, deltaTime * 60.0f);
            camera.camera.position = Math::Vector2::Lerp(camera.camera.position, targetPos, t); }
        else {
            camera.camera.position = targetPos; } }
    void CameraSystem::SyncWithRenderer(float screenWidth, float screenHeight) {
        const CameraComponent* camera = GetActiveCamera();
        if (!camera) return;
        Graphics::Camera2D rendererCamera;
        rendererCamera.position = camera->camera.position;
        rendererCamera.zoom = camera->camera.zoom;
        rendererCamera.rotation = camera->camera.rotation;
        rendererCamera.nearPlane = camera->camera.nearPlane;
        rendererCamera.farPlane = camera->camera.farPlane;
        auto& mutableCamera = m_ComponentStore->GetComponent<CameraComponent>(m_ActiveCameraEntity);
        mutableCamera.UpdateScreenDimensions(screenWidth, screenHeight);
        Graphics::Renderer2D::BeginScene(rendererCamera); } }
