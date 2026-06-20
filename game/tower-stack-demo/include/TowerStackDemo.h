#pragma once

#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/math/Vector3.h"

#include <vector>
#include <random>

// ============================================================================
//  TowerStackDemo
//  A physics-based tower stacking game:
//    - Blocks slide left-right above the tower
//    - Press SPACE to drop the block onto the tower
//    - If the tower is unbalanced, it tips over and collapses
//    - Camera follows the tower upward, zooms out on collapse
// ============================================================================
class TowerStackDemo : public Nyon::ECSApplication
{
public:
    TowerStackDemo();

protected:
    void OnECSStart() override;
    void OnECSFixedUpdate(float deltaTime) override;

private:
    enum class GameState { PLAYING, FALLING, GAME_OVER };

    // ---------- setup helpers -----------------------------------------------
    void CreateWorld();
    void CreateCamera();
    void CreatePlatform();
    void SpawnActiveBlock();

    // ---------- game logic --------------------------------------------------
    void DropActiveBlock();
    void UpdateCamera(float deltaTime);
    void CheckGameOver();
    void CleanupFallenBlocks();
    void ResetGame();

    // ---------- runtime state -----------------------------------------------
    Nyon::ECS::EntityID m_WorldEntity  { 0 };
    Nyon::ECS::EntityID m_CameraEntity { 0 };
    Nyon::ECS::EntityID m_PlatformEntity { 0 };
    Nyon::ECS::EntityID m_ActiveBlock  { 0 };

    std::vector<Nyon::ECS::EntityID> m_Blocks;

    // Game state
    GameState m_State           { GameState::PLAYING };
    int m_Score                 { 0 };
    float m_SlideDirection      { 1.0f };
    float m_HighestBlockY       { 0.0f };
    float m_FallTimer           { 0.0f };

    // Camera animation
    float m_StartCamZoom        { 1.0f };
    float m_StartCamY           { 0.0f };
    float m_TargetCamZoom       { 1.0f };
    float m_TargetCamY          { 0.0f };
    float m_CamAnimTimer        { 0.0f };

    // Block color cycling
    std::vector<Nyon::Math::Vector3> m_BlockColors;
    int m_ColorIndex            { 0 };

    // Random number generator
    std::mt19937 m_Rng{ std::random_device{}() };

    // ---------- constants ---------------------------------------------------
    static constexpr float PLATFORM_WIDTH    = 400.0f;
    static constexpr float PLATFORM_HEIGHT   = 30.0f;
    static constexpr float PLATFORM_Y        = 50.0f;

    static constexpr float BLOCK_WIDTH       = 120.0f;
    static constexpr float BLOCK_HEIGHT      = 30.0f;
    static constexpr float SLIDE_SPEED       = 350.0f;
    static constexpr float BLOCK_DENSITY     = 0.5f;
    static constexpr float FRICTION          = 0.6f;
    static constexpr float RESTITUTION       = 0.0f;

    static constexpr float FALL_DURATION     = 2.0f;  // camera zoom-out duration
    static constexpr float FALL_TARGET_ZOOM  = 0.3f;
    static constexpr float FALL_TARGET_CAM_Y = -200.0f;
};
