#pragma once

#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/math/Vector3.h"

#include <vector>
#include <random>

// ============================================================================
//  BreakoutDemo
//  A classic breakout game with:
//    - Paddle controlled by mouse or arrow keys
//    - Ball that bounces off walls, paddle, and bricks
//    - Grid of breakable bricks
//    - Score tracking and game state management
// ============================================================================
class BreakoutDemo : public Nyon::ECSApplication
{
public:
    BreakoutDemo();

protected:
    // Called once before the first physics tick (world + entities set up here).
    void OnECSStart() override;

    // Called every fixed physics tick (1/60 s).
    void OnECSFixedUpdate(float deltaTime) override;

private:
    // ---------- setup helpers -----------------------------------------------
    void CreateWorld();
    void CreateWalls();
    void CreatePaddle();
    void CreateBall();
    void CreateBricks();
    
    // ---------- game logic --------------------------------------------------
    void HandleInput(float deltaTime);
    void CheckBrickCollisions();
    void ResetBall();
    void ResetGame();
    
    // ---------- brick creation ----------------------------------------------
    void CreateBrick(float x, float y, int row, int col);

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
    
    static constexpr float BRICK_WIDTH       = 75.0f;
    static constexpr float BRICK_HEIGHT      = 30.0f;
    static constexpr int   BRICK_ROWS        = 5;
    static constexpr int   BRICK_COLS        = 10;
    static constexpr float BRICK_SPACING_X   = 5.0f;
    static constexpr float BRICK_SPACING_Y   = 5.0f;
    static constexpr float BRICK_START_X     = 50.0f;
    static constexpr float BRICK_START_Y     = 400.0f;

    // Game state
    int m_Score { 0 };
    bool m_GameWon { false };
    bool m_BallLaunched { false };  // Ball sticks to paddle until space is pressed
    
    // Random number generator
    std::mt19937 m_Rng{ std::random_device{}() };
};
