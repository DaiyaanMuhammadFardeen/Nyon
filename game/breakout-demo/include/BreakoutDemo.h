#pragma once

#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/math/Vector3.h"

#include <vector>
#include <random>
#include <set>

// ============================================================================
//  BreakoutDemo
//  A classic breakout game with:
//    - Paddle controlled by arrow keys
//    - Ball that bounces off walls, paddle, and bricks
//    - Random cohesive brick formation of square bricks
//    - Score tracking and game state management
// ============================================================================
class BreakoutDemo : public Nyon::ECSApplication
{
public:
    BreakoutDemo();

protected:
    void OnECSStart() override;
    void OnECSFixedUpdate(float deltaTime) override;

private:
    // ---------- setup helpers -----------------------------------------------
    void CreateWorld();
    void CreateWalls();
    void CreatePaddle();
    void CreateBall();

    // ---------- brick generation --------------------------------------------
    void GenerateBricks();
    void CreateBrick(float x, float y, const Nyon::Math::Vector3& color);
    std::vector<Nyon::Math::Vector3> m_BrickColors;

    // ---------- game logic --------------------------------------------------
    void HandleInput(float deltaTime);
    void CheckBrickCollisions();
    void ResetBall();
    void ResetGame();

    // ---------- runtime state -----------------------------------------------
    Nyon::ECS::EntityID m_PaddleEntity { 0 };
    Nyon::ECS::EntityID m_BallEntity   { 0 };

    std::vector<Nyon::ECS::EntityID> m_Bricks;

    // Game configuration
    static constexpr float PADDLE_WIDTH      = 120.0f;
    static constexpr float PADDLE_HEIGHT     = 20.0f;
    static constexpr float PADDLE_Y          = 80.0f;
    static constexpr float PADDLE_SPEED      = 500.0f;

    static constexpr float BALL_RADIUS       = 10.0f;
    static constexpr float BALL_SPEED        = 350.0f;

    // Square brick configuration
    static constexpr float BRICK_SIZE        = 40.0f;
    static constexpr float BRICK_GAP         = 4.0f;
    static constexpr float BRICK_START_Y     = 280.0f;

    // Random shape generation
    static constexpr int   SHAPE_MAX_COLS    = 14;
    static constexpr int   SHAPE_MAX_ROWS    = 8;
    static constexpr int   TARGET_BRICK_MIN  = 50;
    static constexpr int   TARGET_BRICK_MAX  = 70;

    // Game state
    int m_Score { 0 };
    bool m_GameWon { false };
    bool m_BallLaunched { false };

    // Random number generator
    std::mt19937 m_Rng{ std::random_device{}() };
};
