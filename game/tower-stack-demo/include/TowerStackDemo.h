#pragma once
#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/math/Vector3.h"
#include <vector>
#include <random>
class TowerStackDemo : public Nyon::ECSApplication {
public:
    TowerStackDemo();
protected:
    void OnECSStart() override;
    void OnECSFixedUpdate(float deltaTime) override;
private:
    enum class GameState { PLAYING, FALLING, GAME_OVER };
    void CreateWorld();
    void CreateCamera();
    void CreatePlatform();
    void SpawnActiveBlock();
    void DropActiveBlock();
    void UpdateCamera(float deltaTime);
    void CheckGameOver();
    void CleanupFallenBlocks();
    void ResetGame();
    Nyon::ECS::EntityID m_WorldEntity  { 0 };
    Nyon::ECS::EntityID m_CameraEntity { 0 };
    Nyon::ECS::EntityID m_PlatformEntity { 0 };
    Nyon::ECS::EntityID m_ActiveBlock  { 0 };
    std::vector<Nyon::ECS::EntityID> m_Blocks;
    GameState m_State           { GameState::PLAYING };
    int m_Score                 { 0 };
    float m_SlideDirection      { 1.0f };
    float m_HighestBlockY       { 0.0f };
    float m_FallTimer           { 0.0f };
    float m_StartCamZoom        { 1.0f };
    float m_StartCamY           { 0.0f };
    float m_TargetCamZoom       { 1.0f };
    float m_TargetCamY          { 0.0f };
    float m_CamAnimTimer        { 0.0f };
    std::vector<Nyon::Math::Vector3> m_BlockColors;
    int m_ColorIndex            { 0 };
    std::mt19937 m_Rng{ std::random_device{}() };
    static constexpr float PLATFORM_WIDTH    = 400.0f;
    static constexpr float PLATFORM_HEIGHT   = 30.0f;
    static constexpr float PLATFORM_Y        = 50.0f;
    static constexpr float BLOCK_WIDTH       = 120.0f;
    static constexpr float BLOCK_HEIGHT      = 30.0f;
    static constexpr float SLIDE_SPEED       = 350.0f;
    static constexpr float BLOCK_DENSITY     = 0.5f;
    static constexpr float FRICTION          = 0.6f;
    static constexpr float RESTITUTION       = 0.0f;
    static constexpr float FALL_DURATION     = 2.0f;   
    static constexpr float FALL_TARGET_ZOOM  = 0.3f;
    static constexpr float FALL_TARGET_CAM_Y = -200.0f; };
