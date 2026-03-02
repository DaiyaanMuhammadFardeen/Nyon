#pragma once

#include "nyon/core/ECSApplication.h"
#include "nyon/core/Application.h"
#include "nyon/utils/InputManager.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/systems/CollisionPipelineSystem.h"
#include "nyon/ecs/systems/ConstraintSolverSystem.h"
// REMOVED: TransformPhysicsSyncSystem.h - double integration issue fixed
#include "nyon/ecs/systems/PhysicsIntegrationSystem.h"
// REMOVED: ContinuousCollisionSystem.h - using CollisionPipelineSystem instead
#include "nyon/ecs/systems/DebugRenderSystem.h"
#include <vector>
#include <random>

namespace Game
{
    class PhysicsDemoGame : public Nyon::ECSApplication
    {
    public:
        PhysicsDemoGame();
        ~PhysicsDemoGame() = default;
        
        void OnECSStart() override;
        void OnECSUpdate(float deltaTime) override;
        
    private:
        // Scene creation methods
        void CreateEnvironment();
        void CreateInteractiveObjects();
        void CreatePlayer();
        void CreateObstacles();
        void RestartScene(); // New method
        
        // Object creation utilities
        Nyon::ECS::EntityID CreateBox(const Nyon::Math::Vector2& position, 
                                     const Nyon::Math::Vector2& size,
                                     float density = 1.0f,
                                     bool isStatic = false);
        
        Nyon::ECS::EntityID CreateCircle(const Nyon::Math::Vector2& position,
                                        float radius,
                                        float density = 1.0f,
                                        bool isStatic = false);
        
        Nyon::ECS::EntityID CreateCapsule(const Nyon::Math::Vector2& position,
                                         float height,
                                         float radius,
                                         float density = 1.0f,
                                         bool isStatic = false);
        
        // Game mechanics
        void HandlePlayerInput(float deltaTime);
        void SpawnRandomObject();
        
        // Game state
        Nyon::ECS::EntityID m_PlayerId;
        std::vector<Nyon::ECS::EntityID> m_GameEntities;
        std::mt19937 m_RandomGenerator;
        std::uniform_real_distribution<float> m_FloatDistribution;
        
        // Game controls
        bool m_ShowDebug = true;
        float m_SpawnTimer = 0.0f;
        const float SPAWN_INTERVAL = 2.0f;
    };
}