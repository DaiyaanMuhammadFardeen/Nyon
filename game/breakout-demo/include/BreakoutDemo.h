#pragma once
#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/math/Vector3.h"
#include <vector>
#include <random>
#include <set>
class BreakoutDemo : public Nyon::ECSApplication {
public:
    BreakoutDemo();
protected:
    void OnECSStart() override;
    void OnECSFixedUpdate(float deltaTime) override;
private:
    void CreateWorld();
    void CreateWalls();
    void CreatePaddle();
    void CreateBall();
    void GenerateBricks();
    void CreateBrick(float x, float y, const Nyon::Math::Vector3& color);
    std::vector<Nyon::Math::Vector3> m_BrickColors;
    void HandleInput(float deltaTime);
    void CheckBrickCollisions();
    void ResetBall();
    void ResetGame();
    Nyon::ECS::EntityID m_PaddleEntity { 0 };
    Nyon::ECS::EntityID m_BallEntity   { 0 };
    std::vector<Nyon::ECS::EntityID> m_Bricks;
    static constexpr float PADDLE_WIDTH      = 120.0f;
    static constexpr float PADDLE_HEIGHT     = 20.0f;
    static constexpr float PADDLE_Y          = 80.0f;
    static constexpr float PADDLE_SPEED      = 500.0f;
    static constexpr float BALL_RADIUS       = 10.0f;
    static constexpr float BALL_SPEED        = 250.0f;
    static constexpr float BRICK_SIZE        = 40.0f;
    static constexpr float BRICK_GAP         = 4.0f;
    static constexpr float BRICK_START_Y     = 600.0f;
    static constexpr int   SHAPE_MAX_COLS    = 14;
    static constexpr int   SHAPE_MAX_ROWS    = 8;
    static constexpr int   TARGET_BRICK_MIN  = 50;
    static constexpr int   TARGET_BRICK_MAX  = 70;
    int m_Score { 0 };
    bool m_GameWon { false };
    bool m_BallLaunched { false };
    std::mt19937 m_Rng{ std::random_device{}() }; };
