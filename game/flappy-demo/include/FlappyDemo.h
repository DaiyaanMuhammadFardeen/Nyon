#pragma once

#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/math/Vector3.h"

#include <vector>
#include <random>

// ============================================================================
//  FlappyDemo
//  A Flappy Bird clone built on the Nyon ECS engine:
//    - Bird falls under gravity, tap SPACE to flap (upward impulse)
//    - Pipes scroll from right to left with random gaps
//    - Collision with pipe or screen bottom = game over
//    - Score is incremented each time the bird passes a pipe pair
// ============================================================================
class FlappyDemo : public Nyon::ECSApplication
{
public:
    FlappyDemo();

protected:
    void OnECSStart() override;
    void OnECSFixedUpdate(float deltaTime) override;

private:
    enum class GameState { MENU, PLAYING, GAME_OVER };

    // ---------- setup helpers -----------------------------------------------
    void CreateWorld();
    void CreateCamera();
    void CreateBird();

    // ---------- pipes -------------------------------------------------------
    void SpawnPipePair();
    Nyon::ECS::EntityID CreatePipeSegment(float x, float centerY, float height);
    void DestroyOffscreenPipes();

    // ---------- game logic --------------------------------------------------
    void HandleInput();
    void CheckCollisions();
    void UpdateScore(float birdX);
    void ResetGame();

    // ---------- runtime state -----------------------------------------------
    Nyon::ECS::EntityID m_WorldEntity  { 0 };
    Nyon::ECS::EntityID m_BirdEntity   { 0 };
    Nyon::ECS::EntityID m_CameraEntity { 0 };

    // Pipe tracking: store both top and bottom pipe entity IDs
    std::vector<Nyon::ECS::EntityID> m_Pipes;
    float m_PipeSpawnTimer { 0.0f };

    // Game state
    GameState m_State { GameState::MENU };
    int m_Score             { 0 };
    float m_LastScoredX     { -1000.0f };

    // Random number generator
    std::mt19937 m_Rng{ std::random_device{}() };

    // ---------- constants ---------------------------------------------------
    static constexpr float BIRD_RADIUS       = 18.0f;
    static constexpr float BIRD_X            = 250.0f;
    static constexpr float FLAP_IMPULSE      = 420.0f;

    static constexpr float SCROLL_SPEED      = 200.0f;
    static constexpr float PIPE_WIDTH        = 80.0f;
    static constexpr float PIPE_GAP          = 180.0f;
    static constexpr float PIPE_SPAWN_INTERVAL = 1.6f;
    static constexpr float PIPE_MIN_HEIGHT   = 60.0f;
    static constexpr float PIPE_MAX_HEIGHT   = 400.0f;

    static constexpr float GROUND_Y          = 0.0f;  // bottom of screen
};
