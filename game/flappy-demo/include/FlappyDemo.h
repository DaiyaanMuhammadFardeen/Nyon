#pragma once
#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/math/Vector3.h"
#include <vector>
#include <random>
class FlappyDemo : public Nyon::ECSApplication {
public:
    FlappyDemo();
protected:
    void OnECSStart() override;
    void OnECSFixedUpdate(float deltaTime) override;
private:
    enum class GameState { MENU, PLAYING, GAME_OVER };
    void CreateWorld();
    void CreateCamera();
    void CreateBird();
    void SpawnPipePair();
    Nyon::ECS::EntityID CreatePipeSegment(float x, float centerY, float height);
    void DestroyOffscreenPipes();
    void HandleInput();
    void CheckCollisions();
    void UpdateScore(float birdX);
    void ResetGame();
    Nyon::ECS::EntityID m_WorldEntity  { 0 };
    Nyon::ECS::EntityID m_BirdEntity   { 0 };
    Nyon::ECS::EntityID m_CameraEntity { 0 };
    std::vector<Nyon::ECS::EntityID> m_Pipes;
    float m_PipeSpawnTimer { 0.0f };
    GameState m_State { GameState::MENU };
    int m_Score             { 0 };
    float m_LastScoredX     { -1000.0f };
    std::mt19937 m_Rng{ std::random_device{}() };
    static constexpr float BIRD_RADIUS       = 18.0f;
    static constexpr float BIRD_X            = 250.0f;
    static constexpr float FLAP_IMPULSE      = 420.0f;
    static constexpr float SCROLL_SPEED      = 200.0f;
    static constexpr float PIPE_WIDTH        = 80.0f;
    static constexpr float PIPE_GAP          = 180.0f;
    static constexpr float PIPE_SPAWN_INTERVAL = 1.6f;
    static constexpr float PIPE_MIN_HEIGHT   = 60.0f;
    static constexpr float PIPE_MAX_HEIGHT   = 400.0f;
    static constexpr float GROUND_Y          = 0.0f;    };
