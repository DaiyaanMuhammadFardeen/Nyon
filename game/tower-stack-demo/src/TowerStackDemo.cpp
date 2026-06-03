#include "TowerStackDemo.h"

#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/CameraComponent.h"
#include "nyon/utils/InputManager.h"

#include <iostream>
#include <algorithm>
#include <cmath>

using namespace Nyon;

// ============================================================================
//  Constructor
// ============================================================================
TowerStackDemo::TowerStackDemo()
    : ECSApplication("Tower Stack", 1280, 720)
{
    m_BlockColors = {
        { 0.2f, 0.6f, 1.0f },   // blue
        { 0.2f, 0.8f, 0.3f },   // green
        { 1.0f, 0.8f, 0.0f },   // yellow
        { 1.0f, 0.4f, 0.2f },   // orange
        { 0.8f, 0.3f, 1.0f },   // purple
        { 1.0f, 0.2f, 0.2f },   // red
        { 0.2f, 1.0f, 0.8f },   // cyan
    };
}

// ============================================================================
//  OnECSStart  –  runs once before the first frame
// ============================================================================
void TowerStackDemo::OnECSStart()
{
    std::cerr << "\n=== TOWER STACK ===\n";
    std::cerr << "Press SPACE to drop the block. Try to stack as high as you can!\n";
    std::cerr << "Press R to restart after game over.\n\n";

    CreateWorld();
    CreateCamera();
    CreatePlatform();
    SpawnActiveBlock();

    m_HighestBlockY = PLATFORM_Y + PLATFORM_HEIGHT / 2.0f;
}

// ============================================================================
//  CreateWorld  –  physics configuration
// ============================================================================
void TowerStackDemo::CreateWorld()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    m_WorldEntity = entities.CreateEntity();

    ECS::PhysicsWorldComponent world;
    world.gravity                = { 0.0f, -1200.0f };
    world.timeStep               = 1.0f / 60.0f;
    world.velocityIterations     = 8;
    world.positionIterations     = 4;
    world.linearSlop             = 0.3f;
    world.maxLinearCorrection    = 4.0f;
    world.enableSleep            = true;
    world.enableWarmStarting     = true;
    world.enableContinuous       = false;
    world.baumgarteBeta          = 0.2f;

    cs.AddComponent(m_WorldEntity, std::move(world));
}

// ============================================================================
//  CreateCamera  –  fixed start, follows tower
// ============================================================================
void TowerStackDemo::CreateCamera()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    m_CameraEntity = entities.CreateEntity();

    ECS::CameraComponent cam(1280.0f, 720.0f);
    cam.isActive = true;
    cam.camera.zoom = 1.0f;
    cam.camera.position = { 0.0f, 0.0f };

    cs.AddComponent(m_CameraEntity, std::move(cam));
}

// ============================================================================
//  CreatePlatform  –  static base at bottom of the screen
// ============================================================================
void TowerStackDemo::CreatePlatform()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    m_PlatformEntity = entities.CreateEntity();

    float halfW = PLATFORM_WIDTH / 2.0f;
    float halfH = PLATFORM_HEIGHT / 2.0f;

    // Transform
    ECS::TransformComponent t;
    t.position = { 640.0f, PLATFORM_Y };
    t.previousPosition = t.position;

    // Static body
    ECS::PhysicsBodyComponent body;
    body.isStatic = true;
    body.UpdateMassProperties();

    // Collider
    ECS::ColliderComponent::PolygonShape shape({
        { -halfW, -halfH },
        {  halfW, -halfH },
        {  halfW,  halfH },
        { -halfW,  halfH }
    });

    ECS::ColliderComponent collider(shape);
    collider.material.friction = FRICTION;
    collider.material.restitution = RESTITUTION;
    collider.material.density = 0.0f;

    // Render
    ECS::RenderComponent render({ PLATFORM_WIDTH, PLATFORM_HEIGHT }, { 0.4f, 0.4f, 0.4f });
    render.origin = { halfW, halfH };

    cs.AddComponent(m_PlatformEntity, std::move(t));
    cs.AddComponent(m_PlatformEntity, std::move(body));
    cs.AddComponent(m_PlatformEntity, std::move(collider));
    cs.AddComponent(m_PlatformEntity, std::move(render));
}

// ============================================================================
//  SpawnActiveBlock  –  creates a new sliding block above the tower
// ============================================================================
void TowerStackDemo::SpawnActiveBlock()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    m_ActiveBlock = entities.CreateEntity();

    float spawnY = m_HighestBlockY + BLOCK_HEIGHT + 1.0f;

    // Transform
    ECS::TransformComponent t;
    t.position = { 640.0f, spawnY };
    t.previousPosition = t.position;

    // Render only (no physics yet - added on drop)
    const auto& color = m_BlockColors[m_ColorIndex % m_BlockColors.size()];
    m_ColorIndex++;

    ECS::RenderComponent render({ BLOCK_WIDTH, BLOCK_HEIGHT }, color);
    render.origin = { BLOCK_WIDTH / 2.0f, BLOCK_HEIGHT / 2.0f };

    cs.AddComponent(m_ActiveBlock, std::move(t));
    cs.AddComponent(m_ActiveBlock, std::move(render));
}

// ============================================================================
//  DropActiveBlock  –  release the block as a dynamic physics body
// ============================================================================
void TowerStackDemo::DropActiveBlock()
{
    auto& cs = GetComponentStore();

    if (!cs.HasComponent<ECS::TransformComponent>(m_ActiveBlock))
        return;

    // Get the current position (where the block was sliding)
    const auto& t = cs.GetComponent<ECS::TransformComponent>(m_ActiveBlock);
    float dropX = t.position.x;
    float dropY = t.position.y;

    // Add physics body
    ECS::PhysicsBodyComponent body;
    body.SetMass(1.0f);
    float inertia = body.mass * (BLOCK_WIDTH * BLOCK_WIDTH + BLOCK_HEIGHT * BLOCK_HEIGHT) / 12.0f;
    body.SetInertia(inertia);
    body.UpdateMassProperties();
    body.isAwake = true;
    body.allowSleep = true;
    body.velocity = { 0.0f, 0.0f };
    body.angularVelocity = 0.0f;
    body.drag = 0.0f;
    body.angularDamping = 0.5f;
    body.friction = FRICTION;
    body.restitution = RESTITUTION;

    // Add collider
    float halfW = BLOCK_WIDTH / 2.0f;
    float halfH = BLOCK_HEIGHT / 2.0f;

    ECS::ColliderComponent::PolygonShape shape({
        { -halfW, -halfH },
        {  halfW, -halfH },
        {  halfW,  halfH },
        { -halfW,  halfH }
    });

    ECS::ColliderComponent collider(shape);
    collider.material.friction = FRICTION;
    collider.material.restitution = RESTITUTION;
    collider.material.density = BLOCK_DENSITY;

    cs.AddComponent(m_ActiveBlock, std::move(body));
    cs.AddComponent(m_ActiveBlock, std::move(collider));

    // Track this block
    m_Blocks.push_back(m_ActiveBlock);

    // Update highest block Y
    if (dropY > m_HighestBlockY)
        m_HighestBlockY = dropY;

    // Increment score
    m_Score++;

    // Spawn next block
    SpawnActiveBlock();
}

// ============================================================================
//  UpdateCamera  –  smooth camera follow and fall zoom-out
// ============================================================================
void TowerStackDemo::UpdateCamera(float deltaTime)
{
    auto& cs = GetComponentStore();
    if (!cs.HasComponent<ECS::CameraComponent>(m_CameraEntity))
        return;

    auto& cam = cs.GetComponent<ECS::CameraComponent>(m_CameraEntity);

    if (m_State == GameState::PLAYING)
    {
        // Smoothly follow the top of the tower
        float targetY = std::max(0.0f, m_HighestBlockY - 400.0f);
        float lerpFactor = 1.0f - std::pow(0.01f, deltaTime);
        cam.camera.position.y += (targetY - cam.camera.position.y) * lerpFactor;
        cam.camera.zoom = 1.0f;
    }
    else if (m_State == GameState::FALLING)
    {
        // Animate zoom-out and pan down
        float t = m_FallTimer / FALL_DURATION;
        float smoothT = t * t * (3.0f - 2.0f * t);  // smoothstep

        float zoom = m_StartCamZoom + (FALL_TARGET_ZOOM - m_StartCamZoom) * smoothT;
        float camY = m_StartCamY + (FALL_TARGET_CAM_Y - m_StartCamY) * smoothT;

        cam.camera.zoom = zoom;
        cam.camera.position.y = camY;
    }
}

// ============================================================================
//  CheckGameOver  –  detect if any block fell below the screen
// ============================================================================
void TowerStackDemo::CheckGameOver()
{
    auto& cs = GetComponentStore();

    for (auto blockId : m_Blocks)
    {
        if (cs.HasComponent<ECS::TransformComponent>(blockId))
        {
            const auto& t = cs.GetComponent<ECS::TransformComponent>(blockId);
            if (t.position.y < -200.0f)
            {
                // Tower collapsed
                m_State = GameState::FALLING;
                m_FallTimer = 0.0f;

                // Record camera state for animation
                if (cs.HasComponent<ECS::CameraComponent>(m_CameraEntity))
                {
                    const auto& cam = cs.GetComponent<ECS::CameraComponent>(m_CameraEntity);
                    m_StartCamZoom = cam.camera.zoom;
                    m_StartCamY = cam.camera.position.y;
                }

                std::cerr << "\n*** TOWER COLLAPSED! Score: " << m_Score << " ***\n";
                return;
            }
        }
    }
}

// ============================================================================
//  CleanupFallenBlocks  –  destroy blocks far below the play area
// ============================================================================
void TowerStackDemo::CleanupFallenBlocks()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    auto it = m_Blocks.begin();
    while (it != m_Blocks.end())
    {
        if (cs.HasComponent<ECS::TransformComponent>(*it))
        {
            const auto& t = cs.GetComponent<ECS::TransformComponent>(*it);
            if (t.position.y < -1500.0f)
            {
                entities.DestroyEntity(*it, cs);
                it = m_Blocks.erase(it);
                continue;
            }
        }
        ++it;
    }
}

// ============================================================================
//  ResetGame  –  full reset to initial state
// ============================================================================
void TowerStackDemo::ResetGame()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    // Destroy all placed blocks
    for (auto blockId : m_Blocks)
    {
        entities.DestroyEntity(blockId, cs);
    }
    m_Blocks.clear();

    // Destroy the current active block
    if (m_ActiveBlock != 0)
    {
        entities.DestroyEntity(m_ActiveBlock, cs);
        m_ActiveBlock = 0;
    }

    // Reset camera
    if (cs.HasComponent<ECS::CameraComponent>(m_CameraEntity))
    {
        auto& cam = cs.GetComponent<ECS::CameraComponent>(m_CameraEntity);
        cam.camera.position = { 0.0f, 0.0f };
        cam.camera.zoom = 1.0f;
    }

    // Reset state
    m_State = GameState::PLAYING;
    m_Score = 0;
    m_SlideDirection = 1.0f;
    m_HighestBlockY = PLATFORM_Y + PLATFORM_HEIGHT / 2.0f;
    m_FallTimer = 0.0f;
    m_ColorIndex = 0;

    SpawnActiveBlock();

    std::cerr << "\n=== GAME RESET ===\n";
    std::cerr << "Press SPACE to drop the block.\n\n";
}

// ============================================================================
//  OnECSFixedUpdate  –  runs every 1/60 s tick
// ============================================================================
void TowerStackDemo::OnECSFixedUpdate(float deltaTime)
{
    auto& cs = GetComponentStore();

    // --- State-independent logic ---
    CleanupFallenBlocks();
    UpdateCamera(deltaTime);

    // --- State machine ---
    switch (m_State)
    {
        case GameState::PLAYING:
        {
            // 1. Slide the active block
            if (cs.HasComponent<ECS::TransformComponent>(m_ActiveBlock))
            {
                auto& t = cs.GetComponent<ECS::TransformComponent>(m_ActiveBlock);

                t.position.x += SLIDE_SPEED * m_SlideDirection * deltaTime;
                t.previousPosition.x = t.position.x;

                // Bounce off screen edges (with margin)
                if (t.position.x > 1180.0f)
                {
                    t.position.x = 1180.0f;
                    m_SlideDirection = -1.0f;
                }
                else if (t.position.x < 100.0f)
                {
                    t.position.x = 100.0f;
                    m_SlideDirection = 1.0f;
                }

                // Keep the block at the correct Y (above the tower)
                float targetY = m_HighestBlockY + BLOCK_HEIGHT * 3.0f + 30.0f;
                t.position.y = targetY;
                t.previousPosition.y = targetY;
            }

            // 2. Handle drop
            if (Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE))
            {
                DropActiveBlock();
            }

            // 3. Check for tower collapse
            CheckGameOver();

            break;
        }

        case GameState::FALLING:
        {
            m_FallTimer += deltaTime;

            // After the fall animation, transition to game over
            if (m_FallTimer >= FALL_DURATION)
            {
                m_State = GameState::GAME_OVER;
                std::cerr << "\n=== GAME OVER ===\n";
                std::cerr << "Final score: " << m_Score << "\n";
                std::cerr << "Press R to restart.\n\n";
            }

            break;
        }

        case GameState::GAME_OVER:
        {
            // Keep cleaning up fallen blocks
            // Wait for restart
            break;
        }
    }

    // --- Common input ---
    if (Utils::InputManager::IsKeyPressed(GLFW_KEY_R) &&
        (m_State == GameState::GAME_OVER || m_State == GameState::FALLING))
    {
        ResetGame();
    }
}
