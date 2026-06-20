#pragma once
#include <functional>
#include "nyon/ecs/EntityManager.h"
namespace Nyon::ECS {
    class BehaviorComponent {
    public:
        using UpdateFunction = std::function<void(EntityID entity, float deltaTime)>;
        using CollisionFunction = std::function<void(EntityID entity, EntityID other)>;
        BehaviorComponent() = default;
        void SetUpdateFunction(UpdateFunction func) { m_UpdateFunc = func; }
        void SetCollisionFunction(CollisionFunction func) { m_CollisionFunc = func; }
        void Update(EntityID entity, float deltaTime) {
            if (m_UpdateFunc) {
                m_UpdateFunc(entity, deltaTime); } }
        void OnCollision(EntityID entity, EntityID other) {
            if (m_CollisionFunc) {
                m_CollisionFunc(entity, other); } }
    private:
        UpdateFunction m_UpdateFunc;
        CollisionFunction m_CollisionFunc; }; }