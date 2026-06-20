#include "BreakoutDemo.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/BehaviorComponent.h"
#include "nyon/utils/InputManager.h"
#include "nyon/math/Vector3.h"
#include <iostream>
#include <algorithm>
using namespace Nyon;
BreakoutDemo::BreakoutDemo()
    : ECSApplication("Nyon – Breakout Demo", 1280, 720) { }
void BreakoutDemo::OnECSStart() {
    CreateWorld();
    CreateWalls();
    CreatePaddle();
    CreateBall();
    GenerateBricks();
    std::cerr << "[BREAKOUT] Game initialized! Use Arrow Keys to move paddle.\n";
    std::cerr << "[BREAKOUT] Press SPACE to launch ball. Press R to restart.\n"; }
void BreakoutDemo::OnECSFixedUpdate(float deltaTime) {
    HandleInput(deltaTime);
    CheckBrickCollisions();
    if (!m_GameWon && m_Bricks.empty()) {
        m_GameWon = true;
        std::cerr << "\n*** YOU WIN! Score: " << m_Score << " ***\n";
        std::cerr << "Press R to play again!\n"; }
    auto& cs = GetComponentStore();
    if (cs.HasComponent<ECS::TransformComponent>(m_BallEntity)) {
        const auto& ballTransform = cs.GetComponent<ECS::TransformComponent>(m_BallEntity);
        if (ballTransform.position.y < 0.0f) {
            std::cerr << "[BREAKOUT] Ball lost! Resetting...\n";
            ResetBall(); } } }
void BreakoutDemo::CreateWorld() {
    auto& entities  = GetEntityManager();
    auto& cs        = GetComponentStore();
    ECS::EntityID worldEntity = entities.CreateEntity();
    ECS::PhysicsWorldComponent world;
    world.gravity = { 0.0f, 0.0f };   
    world.timeStep              = 1.0f / 60.0f;
    world.velocityIterations    = 8;
    world.positionIterations    = 3;
    world.linearSlop            = 0.5f;
    world.maxLinearCorrection   = 2.0f;
    world.enableSleep           = false;   
    world.enableWarmStarting    = true;
    world.enableContinuous      = false;
    cs.AddComponent(worldEntity, std::move(world)); }
void BreakoutDemo::CreateWalls() {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    constexpr float wallThickness = 50.0f; {
        ECS::EntityID topWall = entities.CreateEntity();
        ECS::TransformComponent t;
        t.position = { width / 2.0f, height + wallThickness / 2.0f };
        t.previousPosition = t.position;
        t.rotation = 0.0f;
        t.previousRotation = 0.0f;
        ECS::PhysicsBodyComponent body;
        body.isStatic = true;
        body.UpdateMassProperties();
        ECS::ColliderComponent::PolygonShape shape({ { -width / 2.0f - wallThickness, -wallThickness / 2.0f }, {  width / 2.0f + wallThickness, -wallThickness / 2.0f }, {  width / 2.0f + wallThickness,  wallThickness / 2.0f }, { -width / 2.0f - wallThickness,  wallThickness / 2.0f } });
        ECS::ColliderComponent collider(shape);
        collider.material.friction = 0.0f;
        collider.material.restitution = 1.0f;   
        collider.material.density = 0.0f;
        cs.AddComponent(topWall, std::move(t));
        cs.AddComponent(topWall, std::move(body));
        cs.AddComponent(topWall, std::move(collider)); } {
        ECS::EntityID leftWall = entities.CreateEntity();
        ECS::TransformComponent t;
        t.position = { -wallThickness / 2.0f, height / 2.0f };
        t.previousPosition = t.position;
        t.rotation = 0.0f;
        t.previousRotation = 0.0f;
        ECS::PhysicsBodyComponent body;
        body.isStatic = true;
        body.UpdateMassProperties();
        ECS::ColliderComponent::PolygonShape shape({ { -wallThickness / 2.0f, -height / 2.0f - wallThickness }, {  wallThickness / 2.0f, -height / 2.0f - wallThickness }, {  wallThickness / 2.0f,  height / 2.0f + wallThickness }, { -wallThickness / 2.0f,  height / 2.0f + wallThickness } });
        ECS::ColliderComponent collider(shape);
        collider.material.friction = 0.0f;
        collider.material.restitution = 1.0f;
        collider.material.density = 0.0f;
        cs.AddComponent(leftWall, std::move(t));
        cs.AddComponent(leftWall, std::move(body));
        cs.AddComponent(leftWall, std::move(collider)); } {
        ECS::EntityID rightWall = entities.CreateEntity();
        ECS::TransformComponent t;
        t.position = { width + wallThickness / 2.0f, height / 2.0f };
        t.previousPosition = t.position;
        t.rotation = 0.0f;
        t.previousRotation = 0.0f;
        ECS::PhysicsBodyComponent body;
        body.isStatic = true;
        body.UpdateMassProperties();
        ECS::ColliderComponent::PolygonShape shape({ { -wallThickness / 2.0f, -height / 2.0f - wallThickness }, {  wallThickness / 2.0f, -height / 2.0f - wallThickness }, {  wallThickness / 2.0f,  height / 2.0f + wallThickness }, { -wallThickness / 2.0f,  height / 2.0f + wallThickness } });
        ECS::ColliderComponent collider(shape);
        collider.material.friction = 0.0f;
        collider.material.restitution = 1.0f;
        collider.material.density = 0.0f;
        cs.AddComponent(rightWall, std::move(t));
        cs.AddComponent(rightWall, std::move(body));
        cs.AddComponent(rightWall, std::move(collider)); } }
void BreakoutDemo::CreatePaddle() {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    m_PaddleEntity = entities.CreateEntity();
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    ECS::TransformComponent t;
    t.position = { width / 2.0f, PADDLE_Y };
    t.previousPosition = t.position;   
    t.rotation = 0.0f;
    t.previousRotation = 0.0f;
    ECS::PhysicsBodyComponent body;
    body.isStatic = true;
    body.UpdateMassProperties();
    ECS::ColliderComponent::PolygonShape paddleShape({ { -PADDLE_WIDTH / 2.0f, -PADDLE_HEIGHT / 2.0f }, {  PADDLE_WIDTH / 2.0f, -PADDLE_HEIGHT / 2.0f }, {  PADDLE_WIDTH / 2.0f,  PADDLE_HEIGHT / 2.0f }, { -PADDLE_WIDTH / 2.0f,  PADDLE_HEIGHT / 2.0f } });
    ECS::ColliderComponent paddleCollider(paddleShape);
    paddleCollider.material.friction = 0.0f;
    paddleCollider.material.restitution = 1.0f;
    paddleCollider.material.density = 0.0f;
    ECS::RenderComponent paddleRender({ PADDLE_WIDTH, PADDLE_HEIGHT }, { 0.2f, 0.6f, 1.0f });
    paddleRender.origin = { PADDLE_WIDTH / 2.0f, PADDLE_HEIGHT / 2.0f };
    cs.AddComponent(m_PaddleEntity, std::move(t));
    cs.AddComponent(m_PaddleEntity, std::move(body));
    cs.AddComponent(m_PaddleEntity, std::move(paddleCollider));
    cs.AddComponent(m_PaddleEntity, std::move(paddleRender)); }
void BreakoutDemo::CreateBall() {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    m_BallEntity = entities.CreateEntity();
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    ECS::TransformComponent t;
    t.position = { width / 2.0f, 100.0f };   
    t.previousPosition = t.position;
    t.rotation = 0.0f;
    t.previousRotation = 0.0f;
    ECS::PhysicsBodyComponent body;
    body.SetMass(1.0f);
    body.SetInertia(10.0f);   
    body.UpdateMassProperties();
    body.isAwake = true;
    body.allowSleep = false;
    body.motionLocks.lockRotation = false;   
    body.drag = 0.0f;   
    body.angularDamping = 0.1f;   
    body.velocity = { 0.0f, 0.0f };
    body.angularVelocity = 0.0f;
    ECS::ColliderComponent::CircleShape ballShape;
    ballShape.center = { 0.0f, 0.0f };
    ballShape.radius = BALL_RADIUS;
    ECS::ColliderComponent ballCollider(ballShape);
    ballCollider.material.friction = 0.0f;
    ballCollider.material.restitution = 1.0f;   
    ballCollider.material.density = 0.001f;
    ECS::RenderComponent ballRender({ BALL_RADIUS * 2.0f, BALL_RADIUS * 2.0f }, { 1.0f, 1.0f, 1.0f });
    ballRender.origin = { BALL_RADIUS, BALL_RADIUS };
    ballRender.shapeType = ECS::RenderComponent::ShapeType::Circle;
    cs.AddComponent(m_BallEntity, std::move(t));
    cs.AddComponent(m_BallEntity, std::move(body));
    cs.AddComponent(m_BallEntity, std::move(ballCollider));
    cs.AddComponent(m_BallEntity, std::move(ballRender)); }
void BreakoutDemo::GenerateBricks() {
    m_Bricks.clear();
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    m_BrickColors = { { 1.0f, 0.0f, 0.0f }, { 1.0f, 0.5f, 0.0f }, { 1.0f, 1.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.5f, 1.0f }, { 0.6f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.5f },    };
    const float cellSize = BRICK_SIZE + BRICK_GAP;
    const int gridCols = SHAPE_MAX_COLS;
    const int gridRows = SHAPE_MAX_ROWS;
    std::vector<std::vector<bool>> placed(gridRows, std::vector<bool>(gridCols, false));
    std::uniform_int_distribution<int> countDist(TARGET_BRICK_MIN, TARGET_BRICK_MAX);
    int targetCount = countDist(m_Rng);
    std::uniform_int_distribution<int> colDist(3, gridCols - 4);
    std::uniform_int_distribution<int> rowDist(2, gridRows - 3);
    int seedCol = colDist(m_Rng);
    int seedRow = rowDist(m_Rng);
    placed[seedRow][seedCol] = true;
    int placedCount = 1;
    std::uniform_int_distribution<int> dirDist(0, 3);
    const int dirs[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
    int attempts = 0;
    while (placedCount < targetCount && attempts < targetCount * 10) {
        ++attempts;
        std::vector<std::pair<int,int>> candidates;
        for (int r = 0; r < gridRows; ++r) {
            for (int c = 0; c < gridCols; ++c) {
                if (!placed[r][c]) continue;
                for (int d = 0; d < 4; ++d) {
                    int nr = r + dirs[d][0];
                    int nc = c + dirs[d][1];
                    if (nr >= 0 && nr < gridRows && nc >= 0 && nc < gridCols && !placed[nr][nc]) {
                        candidates.emplace_back(c, r);
                        break; } } } }
        if (candidates.empty()) break;
        std::uniform_int_distribution<int> pickDist(0, static_cast<int>(candidates.size()) - 1);
        auto [pc, pr] = candidates[pickDist(m_Rng)];
        std::vector<std::pair<int,int>> freeNeighbors;
        for (int d = 0; d < 4; ++d) {
            int nr = pr + dirs[d][0];
            int nc = pc + dirs[d][1];
            if (nr >= 0 && nr < gridRows && nc >= 0 && nc < gridCols && !placed[nr][nc]) {
                freeNeighbors.emplace_back(nc, nr); } }
        if (freeNeighbors.empty()) continue;
        std::uniform_int_distribution<int> fnDist(0, static_cast<int>(freeNeighbors.size()) - 1);
        auto [nc, nr] = freeNeighbors[fnDist(m_Rng)];
        placed[nr][nc] = true;
        ++placedCount; }
    float totalWidth = static_cast<float>(gridCols) * cellSize;
    float startX = (static_cast<float>(width) - totalWidth) / 2.0f;
    std::vector<Math::Vector3> shuffledColors = m_BrickColors;
    std::shuffle(shuffledColors.begin(), shuffledColors.end(), m_Rng);
    int brickIndex = 0;
    for (int r = 0; r < gridRows; ++r) {
        for (int c = 0; c < gridCols; ++c) {
            if (!placed[r][c]) continue;
            float x = startX + static_cast<float>(c) * cellSize + BRICK_SIZE / 2.0f;
            float y = BRICK_START_Y - static_cast<float>(r) * cellSize + BRICK_SIZE / 2.0f;
            const auto& color = shuffledColors[brickIndex % shuffledColors.size()];
            CreateBrick(x, y, color);
            ++brickIndex; } }
    std::cerr << "[BREAKOUT] Generated " << m_Bricks.size() << " bricks in cohesive shape\n"; }
void BreakoutDemo::CreateBrick(float x, float y, const Math::Vector3& color) {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    ECS::EntityID brickEntity = entities.CreateEntity();
    m_Bricks.push_back(brickEntity);
    ECS::TransformComponent t;
    t.position = { x, y };
    t.previousPosition = t.position;
    t.rotation = 0.0f;
    t.previousRotation = 0.0f;
    ECS::PhysicsBodyComponent body;
    body.isStatic = true;
    body.UpdateMassProperties();
    float half = BRICK_SIZE / 2.0f;
    ECS::ColliderComponent::PolygonShape brickShape({ { -half, -half }, {  half, -half }, {  half,  half }, { -half,  half } });
    ECS::ColliderComponent brickCollider(brickShape);
    brickCollider.material.friction = 0.0f;
    brickCollider.material.restitution = 1.0f;
    brickCollider.material.density = 0.0f;
    ECS::RenderComponent brickRender({ BRICK_SIZE, BRICK_SIZE }, color);
    brickRender.origin = { half, half };
    cs.AddComponent(brickEntity, std::move(t));
    cs.AddComponent(brickEntity, std::move(body));
    cs.AddComponent(brickEntity, std::move(brickCollider));
    cs.AddComponent(brickEntity, std::move(brickRender)); }
void BreakoutDemo::HandleInput(float deltaTime) {
    auto& cs = GetComponentStore();
    if (!cs.HasComponent<ECS::TransformComponent>(m_PaddleEntity))
        return;
    auto& paddleTransform = cs.GetComponent<ECS::TransformComponent>(m_PaddleEntity);
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    float moveX = 0.0f;
    if (Utils::InputManager::IsKeyDown(GLFW_KEY_LEFT) || Utils::InputManager::IsKeyDown(GLFW_KEY_A))
        moveX -= 1.0f;
    if (Utils::InputManager::IsKeyDown(GLFW_KEY_RIGHT) || Utils::InputManager::IsKeyDown(GLFW_KEY_D))
        moveX += 1.0f;
    paddleTransform.position.x += moveX * PADDLE_SPEED * deltaTime;
    float halfWidth = PADDLE_WIDTH / 2.0f;
    paddleTransform.position.x = std::max(halfWidth, std::min(paddleTransform.position.x, width - halfWidth));
    paddleTransform.previousPosition.x = paddleTransform.position.x;
    if (!m_BallLaunched && cs.HasComponent<ECS::TransformComponent>(m_BallEntity)) {
        auto& ballTransform = cs.GetComponent<ECS::TransformComponent>(m_BallEntity);
        auto& ballBody = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BallEntity);
        ballTransform.position.x = paddleTransform.position.x;
        ballTransform.position.y = PADDLE_Y + PADDLE_HEIGHT / 2.0f + BALL_RADIUS + 2.0f;
        ballTransform.previousPosition = ballTransform.position;
        ballBody.velocity = { 0.0f, 0.0f }; }
    if (!m_BallLaunched && Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE)) {
        m_BallLaunched = true;
        if (cs.HasComponent<ECS::PhysicsBodyComponent>(m_BallEntity)) {
            auto& ballBody = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BallEntity);
            std::uniform_real_distribution<float> velDist(0.5f, 0.9f);
            float dir = (m_Rng() % 2 == 0) ? 1.0f : -1.0f;
            ballBody.velocity.x = BALL_SPEED * velDist(m_Rng) * dir;
            ballBody.velocity.y = BALL_SPEED; } }
    if (Utils::InputManager::IsKeyPressed(GLFW_KEY_R))
        ResetGame(); }
void BreakoutDemo::CheckBrickCollisions() {
    auto& cs = GetComponentStore();
    auto worldEntities = cs.GetEntitiesWithComponent<ECS::PhysicsWorldComponent>();
    if (worldEntities.empty())
        return;
    const auto& world = cs.GetComponent<ECS::PhysicsWorldComponent>(worldEntities[0]);
    std::vector<ECS::EntityID> bricksToDestroy;
    for (size_t i = 0; i < world.contactManifolds.size(); ++i) {
        const auto& manifold = world.contactManifolds[i];
        if (manifold.points.empty())
            continue;
        std::cerr << "[DIAG]  Manifold: A=" << manifold.entityIdA << " B=" << manifold.entityIdB << " pts=" << manifold.points.size() << "\n";
        bool isBallA = (manifold.entityIdA == m_BallEntity);
        bool isBallB = (manifold.entityIdB == m_BallEntity);
        if (!isBallA && !isBallB)
            continue;
        ECS::EntityID otherEntity = isBallA ? manifold.entityIdB : manifold.entityIdA;
        auto it = std::find(m_Bricks.begin(), m_Bricks.end(), otherEntity);
        if (it != m_Bricks.end()) {
            bricksToDestroy.push_back(otherEntity);
            m_Score += 10;    } }
    if (!bricksToDestroy.empty()) {
        auto& entities = GetEntityManager();
        for (auto brickId : bricksToDestroy) {
            entities.DestroyEntity(brickId, cs); }
        for (auto brickId : bricksToDestroy) {
            auto it = std::remove(m_Bricks.begin(), m_Bricks.end(), brickId);
            m_Bricks.erase(it, m_Bricks.end()); }
        std::cerr << "[BREAKOUT] Destroyed " << bricksToDestroy.size() << " bricks! Score: " << m_Score << "\n"; } }
void BreakoutDemo::ResetBall() {
    auto& cs = GetComponentStore();
    if (!cs.HasComponent<ECS::TransformComponent>(m_BallEntity))
        return;
    auto& ballTransform = cs.GetComponent<ECS::TransformComponent>(m_BallEntity);
    auto& ballBody = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BallEntity);
    const auto& paddleTransform = cs.GetComponent<ECS::TransformComponent>(m_PaddleEntity);
    ballTransform.position.x = paddleTransform.position.x;
    ballTransform.position.y = PADDLE_Y + PADDLE_HEIGHT / 2.0f + BALL_RADIUS + 2.0f;
    ballTransform.previousPosition = ballTransform.position;   
    ballBody.velocity = { 0.0f, 0.0f };
    ballBody.angularVelocity = 0.0f;
    ballBody.isStatic = false;
    ballBody.allowSleep = false;
    ballBody.isAwake = true;
    m_BallLaunched = false; }
void BreakoutDemo::ResetGame() {
    std::cerr << "[BREAKOUT] Resetting game...\n";
    auto& entities = GetEntityManager();
    auto& cs = GetComponentStore();
    for (auto brickId : m_Bricks) {
        entities.DestroyEntity(brickId, cs); }
    m_Bricks.clear();
    GenerateBricks();
    ResetBall();
    m_Score = 0;
    m_GameWon = false;
    std::cerr << "[BREAKOUT] Game reset! Press SPACE to launch ball.\n"; }
