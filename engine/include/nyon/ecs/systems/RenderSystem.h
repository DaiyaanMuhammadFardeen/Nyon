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
namespace Nyon::ECS {
    class RenderSystem : public System {
    public:
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override {
            System::Initialize(entityManager, componentStore);
            m_Alpha = 1.0f;   }
        void Update(float deltaTime) override {
            if (!m_EntityManager || !m_ComponentStore) return;
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            const CameraComponent* activeCamera = nullptr;
            const auto& cameraEntities = m_ComponentStore->GetEntitiesWithComponent<CameraComponent>();
            for (EntityID camEntity : cameraEntities) {
                const auto& camera = m_ComponentStore->GetComponent<CameraComponent>(camEntity);
                if (camera.isActive) {
                    activeCamera = &camera;
                    break; } }
            GLFWwindow* window = nullptr;
            try { window = Application::Get().GetWindow(); } catch (...) {}
            int width = 1280, height = 720;
            if (window) glfwGetFramebufferSize(window, &width, &height);
            if (activeCamera) {
                const_cast<CameraComponent*>(activeCamera)->UpdateScreenDimensions(static_cast<float>(width), static_cast<float>(height));
                Graphics::Renderer2D::BeginScene(activeCamera->camera); }
            else {
                Graphics::Camera2D defaultCamera;
                defaultCamera.position = {0.0f, 0.0f};
                defaultCamera.zoom = 1.0f;
                defaultCamera.rotation = 0.0f;
                Graphics::Renderer2D::BeginScene(defaultCamera); }
            const auto& renderEntities = m_ComponentStore->GetEntitiesWithComponent<RenderComponent>();
#ifdef _DEBUG
            static int s_RenderDebugCounter = 0;
            if (++s_RenderDebugCounter >= 10) {
                std::cerr << "[RENDER@" << s_RenderDebugCounter << "] Drawing " << renderEntities.size() << " entities:" << std::endl;
                s_RenderDebugCounter = 0; }
#endif
            for (EntityID entity : renderEntities) {
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
                    s_RenderEntityCounter = 0; }
#endif
                if (render.visible) {
                    Math::Vector2 interpPosition = transform.GetInterpolatedPosition(m_Alpha);
                    float interpRotation = transform.GetInterpolatedRotation(m_Alpha);
                    switch (render.shapeType) {
                        case RenderComponent::ShapeType::Circle: {
                            float radius = std::min(render.size.x, render.size.y) * 0.5f;
                            Graphics::Renderer2D::DrawSolidCircle(
                                interpPosition,
                                radius,
                                render.color
                            );
                            break; }
                        case RenderComponent::ShapeType::Rectangle:
                        default:
                            Graphics::Renderer2D::DrawQuad(
                                interpPosition,
                                render.size,
                                render.origin,
                                render.color,
                                interpRotation
                            );
                            break; } } }
            Graphics::Renderer2D::EndScene(); }
        void SetInterpolationAlpha(float alpha) { m_Alpha = alpha; }
        void Shutdown() override {
            Graphics::Renderer2D::Shutdown(); }
    private:
        float m_Alpha = 1.0f;   }; }
