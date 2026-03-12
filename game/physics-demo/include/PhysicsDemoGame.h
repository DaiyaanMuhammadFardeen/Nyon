#pragma once

#include "nyon/core/ECSApplication.h"

/**
 * @brief Comprehensive physics and rendering showcase for the Nyon engine.
 *
 * This demo is intended as an "all-in-one" demonstration of:
 * - ECS entity/component usage
 * - Rigid body physics (stacks, ramps, bouncing, mixed shapes)
 * - Materials (friction / restitution)
 * - Sleeping / stability scenarios
 * - Physics debug rendering and statistics
 * - Input-driven spawning and control
 * - Custom overlay rendering
 */
class PhysicsDemoGame : public Nyon::ECSApplication
{
public:
    PhysicsDemoGame();

protected:
    void OnECSStart() override;
    void OnECSFixedUpdate(float deltaTime) override;
    void OnECSUpdate(float deltaTime) override;

private:
    // World/scenario builders
    void CreatePhysicsWorld();
    void CreateBounds();
    void CreateStackTest();
    void CreateRampFrictionTest();
    void CreateBounceTest();

    // Runtime helpers
    void HandleGlobalInput(float deltaTime);
    void SpawnBoxAt(float x, float y, float size, float restitution, float friction);
    void SpawnBallAt(float x, float y, float radius, float restitution, float friction);

private:
    bool m_Paused = false;
    float m_TimeScale = 1.0f;
};

