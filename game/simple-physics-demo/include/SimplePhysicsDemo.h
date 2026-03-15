#pragma once

#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/EntityManager.h"

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
    void CreateFallingBox();

    // ---------- runtime state -----------------------------------------------

    // Entity IDs stored so we can read their transforms for logging.
    Nyon::ECS::EntityID m_PlatformEntity { 0 };
    Nyon::ECS::EntityID m_BoxEntity      { 0 };

    // One-shot flag: configure the debug renderer on the first physics tick
    // (systems are not yet registered when OnECSStart runs).
    bool  m_DebugConfigured    { false };

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
};