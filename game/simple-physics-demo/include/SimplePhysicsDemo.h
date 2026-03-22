#pragma once
#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/components/BehaviorComponent.h"
#include <random>
class SimplePhysicsDemo : public Nyon::ECSApplication {
public:
    SimplePhysicsDemo();
protected:
    void OnECSStart() override;
    void OnECSFixedUpdate(float deltaTime) override;
private:
    void CreateWorld();
    void CreatePlatform();
    void CreateSlopedPlatform();
    void CreatePlayerQuad();
    void CreateSpawnedQuad(float x, float y);
    void CreateSpawnedCircle(float x, float y);
    void HandlePlayerInput(float deltaTime);
    bool IsPlayerGrounded();
    void SpawnQuadAtMousePosition();
    void DespawnOutOfBoundsObjects();
    Nyon::ECS::EntityID m_PlatformEntity { 0 };
    Nyon::ECS::EntityID m_PlayerEntity   { 0 };   
    std::vector<Nyon::ECS::EntityID> m_SpawnedQuads;
    std::mt19937 m_Rng{ std::random_device{}() };
    float m_NextAutoSpawnTime { 0.0f };
    float m_SpawnIntervalMin { 0.5f };
    float m_SpawnIntervalMax { 1.5f };
    bool  m_CollisionDetected  { false };
    float m_FirstCollisionTime { 0.0f  };
    float m_SimTime            { 0.0f  };
    static constexpr float LOG_INTERVAL_S  { 0.5f  };
    float m_NextLogTime                    { 0.0f  };
    static constexpr float PLAYER_MOVE_SPEED = 300.0f;   
    static constexpr float PLAYER_JUMP_FORCE = 900.0f;   
    static constexpr float GRAVITY_SCALE     = 980.0f;   
    static constexpr float DESPAWN_Y_THRESHOLD = -100.0f;    };