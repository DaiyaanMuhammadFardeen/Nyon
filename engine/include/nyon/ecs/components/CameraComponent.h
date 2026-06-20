#pragma once
#include "nyon/graphics/Renderer2D.h"
#include "nyon/math/Vector2.h"
#include "nyon/ecs/EntityManager.h"
namespace Nyon::ECS {
    struct CameraComponent {
        Graphics::Camera2D camera;
        bool isActive = true;               
        float priority = 0.0f;              
        int layer = 0;                      
        float viewportX = 0.0f;
        float viewportY = 0.0f;
        float viewportWidth = 1.0f;
        float viewportHeight = 1.0f;
        bool followTarget = false;          
        EntityID targetEntity = INVALID_ENTITY;   
        Math::Vector2 followOffset = {0.0f, 0.0f};   
        float followSmoothness = 0.0f;      
        mutable float cachedScreenWidth = 800.0f;
        mutable float cachedScreenHeight = 600.0f;
        CameraComponent() = default;
        CameraComponent(float screenWidth, float screenHeight) {
            cachedScreenWidth = screenWidth;
            cachedScreenHeight = screenHeight; }
        void SetPosition(const Math::Vector2& pos) {
            camera.position = pos; }
        void Move(const Math::Vector2& delta) {
            camera.position = camera.position + delta; }
        void SetZoom(float zoom) {
            camera.zoom = zoom; }
        void Zoom(float delta) {
            camera.zoom += delta; }
        void SetRotation(float angle) {
            camera.rotation = angle; }
        void Rotate(float delta) {
            camera.rotation += delta; }
        glm::mat4 GetViewMatrix() const {
            return camera.GetViewMatrix(); }
        glm::mat4 GetProjectionMatrix() const {
            return camera.GetProjectionMatrix(cachedScreenWidth, cachedScreenHeight); }
        glm::mat4 GetViewProjectionMatrix() const {
            return camera.GetViewProjectionMatrix(cachedScreenWidth, cachedScreenHeight); }
        Math::Vector2 ScreenToWorld(const Math::Vector2& screenPos) const {
            return camera.ScreenToWorld(screenPos, cachedScreenWidth, cachedScreenHeight); }
        Math::Vector2 WorldToScreen(const Math::Vector2& worldPos) const {
            return camera.WorldToScreen(worldPos, cachedScreenWidth, cachedScreenHeight); }
        void UpdateScreenDimensions(float width, float height) {
            cachedScreenWidth = width;
            cachedScreenHeight = height; } }; }
