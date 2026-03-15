#pragma once
#include "nyon/ecs/System.h"
#include "nyon/ecs/components/BehaviorComponent.h"
#include "nyon/utils/InputManager.h"
namespace Nyon::ECS {
    /**
     * @brief Input system that processes user input and applies it to entities.
     * 
     * Handles keyboard, mouse, and gamepad input and translates it
     * into entity actions through behavior components.
     */
    class InputSystem : public System {
    public:
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override {
            System::Initialize(entityManager, componentStore);
        }
        void Update(float deltaTime) override {
            if (!m_EntityManager || !m_ComponentStore) return;
            Utils::InputManager::Update();
            const auto& behaviorEntities = m_ComponentStore->GetEntitiesWithComponent<BehaviorComponent>();
            for (EntityID entity : behaviorEntities)
            {
                auto& behavior = m_ComponentStore->GetComponent<BehaviorComponent>(entity);
                behavior.Update(entity, deltaTime);
            }
        }
    };
}