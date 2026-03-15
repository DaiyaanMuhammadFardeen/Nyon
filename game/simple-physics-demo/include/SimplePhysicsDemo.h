#pragma once

#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/components/BehaviorComponent.h"

#include <random>

// ============================================================================
//  SimplePhysicsDemo
//  A box falls under gravity and lands on a static platform.
//  This demo verifies:
//    - Gravity integration (Y-up, negative-Y = down)
//    - Polygon-polygon narrow-phase collision detection
//    - Impulse-based velocity resolution
//    - Baumgarte position correction
//    - Interpolated rendering vs. physics positions
// ============================================================================
class SimplePhysicsDemo : public Nyon::ECSApplication
{
public:
    SimplePhysicsDemo();

protected:
    // Called once before the first physics tick (world + entities set up here).
    void OnECSStart() override;

    // Called every fixed physics tick (1/60 s).
    void OnECSFixedUpdate(float deltaTime) override;

private:
    // ---------- setup helpers -----------------------------------------------
    void CreateWorld();
    void CreatePlatform();
    void CreatePlayerQuad();
    void CreateSpawnedQuad(float x, float y);
    void CreateSpawnedCircle(float x, float y);
    
    // ---------- input handling ----------------------------------------------
    void HandlePlayerInput(float deltaTime);
    bool IsPlayerGrounded();
    void SpawnQuadAtMousePosition();

    // ---------- runtime state -----------------------------------------------

    // Entity IDs stored so we can read their transforms for logging.
    Nyon::ECS::EntityID m_PlatformEntity { 0 };
    Nyon::ECS::EntityID m_PlayerEntity   { 0 };  // Controllable player quad
    
    // Track spawned quads for cleanup (optional)
    std::vector<Nyon::ECS::EntityID> m_SpawnedQuads;

    // Random number generator for spawning random circles
    std::mt19937 m_Rng{ std::random_device{}() };

    // Auto-spawn random circles at random intervals
    float m_NextAutoSpawnTime { 0.0f };
    float m_SpawnIntervalMin { 0.5f };
    float m_SpawnIntervalMax { 1.5f };

    // Collision bookkeeping.
    bool  m_CollisionDetected  { false };
    float m_FirstCollisionTime { 0.0f  };

    // Accumulated simulation time (increments by FIXED_TIMESTEP each tick).
    float m_SimTime            { 0.0f  };

    // How long the demo runs before closing automatically.
    static constexpr float DEMO_DURATION_S { 60.0f };

    // Print box position to stderr every N seconds of sim time.
    static constexpr float LOG_INTERVAL_S  { 0.5f  };
    float m_NextLogTime                    { 0.0f  };
    
    // Player movement configuration
    static constexpr float PLAYER_MOVE_SPEED = 300.0f;  // pixels per second
    static constexpr float PLAYER_JUMP_FORCE = 600.0f;  // impulse force
    static constexpr float GRAVITY_SCALE     = 980.0f;  // gravity in pixels/s²
};