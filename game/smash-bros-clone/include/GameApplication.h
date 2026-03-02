#pragma once

#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/BehaviorComponent.h"
#include "nyon/utils/InputManager.h"
#include <vector>

class GameApplication : public Nyon::ECSApplication
{
public:
    GameApplication();

protected:
    void OnECSStart() override;
    void OnECSUpdate(float deltaTime) override;

private:
    // ECS entity IDs for game objects
    Nyon::ECS::EntityID m_PlayerEntity;
    std::vector<Nyon::ECS::EntityID> m_PhysicsEntities;
    
    // Physics systems
    Nyon::ECS::PhysicsWorldComponent* m_PhysicsWorld;
    Nyon::ECS::ConstraintSolverSystem* m_ConstraintSolver;
    Nyon::ECS::CollisionPipelineSystem* m_CollisionPipeline;
    Nyon::ECS::TransformPhysicsSyncSystem* m_TransformSync;
    Nyon::ECS::DebugRenderSystem* m_DebugRenderer;
    
    // Demo state
    bool m_ShowDebugPhysics;
    
    // Game constants
    static constexpr float PLAYER_SPEED = 300.0f;
    static constexpr float JUMP_FORCE = -400.0f;
    
    // Setup methods
    void SetupPhysicsWorld();
    void SetupDebugControls();
    
    // Player behavior functions
    void SetupPlayerBehavior();
    
    // Helper methods
    void CreatePlayer();
    void CreatePhysicsDemoScene();
};