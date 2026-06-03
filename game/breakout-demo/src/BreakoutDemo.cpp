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

// ============================================================================
//  Constructor
// ============================================================================
BreakoutDemo::BreakoutDemo()
    : ECSApplication("Nyon – Breakout Demo", 1280, 720)
{
}

// ============================================================================
//  OnECSStart  –  world + entity setup
// ============================================================================
void BreakoutDemo::OnECSStart()
{
    CreateWorld();
    CreateWalls();
    CreatePaddle();
    CreateBall();
    GenerateBricks();
    
    std::cerr << "[BREAKOUT] Game initialized! Use Arrow Keys to move paddle.\n";
    std::cerr << "[BREAKOUT] Press SPACE to launch ball. Press R to restart.\n";
}

// ============================================================================
//  OnECSFixedUpdate  –  runs at exactly 1/60 s per tick
// ============================================================================
void BreakoutDemo::OnECSFixedUpdate(float deltaTime)
{
    HandleInput(deltaTime);
    
    // Check for brick collisions and destroy them
    CheckBrickCollisions();
    
    // Check win condition
    if (!m_GameWon && m_Bricks.empty())
    {
        m_GameWon = true;
        std::cerr << "\n*** YOU WIN! Score: " << m_Score << " ***\n";
        std::cerr << "Press R to play again!\n";
    }
    
    // Reset ball if it falls below paddle
    auto& cs = GetComponentStore();
    if (cs.HasComponent<ECS::TransformComponent>(m_BallEntity))
    {
        const auto& ballTransform = cs.GetComponent<ECS::TransformComponent>(m_BallEntity);
        if (ballTransform.position.y < 0.0f)
        {
            std::cerr << "[BREAKOUT] Ball lost! Resetting...\n";
            ResetBall();
        }
    }
}

// ============================================================================
//  CreateWorld
// ============================================================================
void BreakoutDemo::CreateWorld()
{
    auto& entities  = GetEntityManager();
    auto& cs        = GetComponentStore();

    ECS::EntityID worldEntity = entities.CreateEntity();

    ECS::PhysicsWorldComponent world;
    world.gravity = { 0.0f, 0.0f };  // No gravity in Breakout — ball maintains constant speed
    
    world.timeStep              = 1.0f / 60.0f;
    world.velocityIterations    = 8;
    world.positionIterations    = 3;
    world.linearSlop            = 0.5f;
    world.maxLinearCorrection   = 2.0f;
    world.enableSleep           = false;  // Keep everything awake for responsiveness
    world.enableWarmStarting    = true;
    world.enableContinuous      = false;

    cs.AddComponent(worldEntity, std::move(world));
}

// ============================================================================
//  CreateWalls  –  invisible walls around the play area
// ============================================================================
void BreakoutDemo::CreateWalls()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    
    // Wall thickness
    constexpr float wallThickness = 50.0f;
    
    // Top wall
    {
        ECS::EntityID topWall = entities.CreateEntity();
        
        ECS::TransformComponent t;
        t.position = { width / 2.0f, height + wallThickness / 2.0f };
        t.previousPosition = t.position;
        t.rotation = 0.0f;
        t.previousRotation = 0.0f;
        
        ECS::PhysicsBodyComponent body;
        body.isStatic = true;
        body.UpdateMassProperties();
        
        ECS::ColliderComponent::PolygonShape shape({
            { -width / 2.0f - wallThickness, -wallThickness / 2.0f },
            {  width / 2.0f + wallThickness, -wallThickness / 2.0f },
            {  width / 2.0f + wallThickness,  wallThickness / 2.0f },
            { -width / 2.0f - wallThickness,  wallThickness / 2.0f }
        });
        
        ECS::ColliderComponent collider(shape);
        collider.material.friction = 0.0f;
        collider.material.restitution = 1.0f;  // Perfect bounce
        collider.material.density = 0.0f;
        
        cs.AddComponent(topWall, std::move(t));
        cs.AddComponent(topWall, std::move(body));
        cs.AddComponent(topWall, std::move(collider));
    }
    
    // Left wall
    {
        ECS::EntityID leftWall = entities.CreateEntity();
        
        ECS::TransformComponent t;
        t.position = { -wallThickness / 2.0f, height / 2.0f };
        t.previousPosition = t.position;
        t.rotation = 0.0f;
        t.previousRotation = 0.0f;
        
        ECS::PhysicsBodyComponent body;
        body.isStatic = true;
        body.UpdateMassProperties();
        
        ECS::ColliderComponent::PolygonShape shape({
            { -wallThickness / 2.0f, -height / 2.0f - wallThickness },
            {  wallThickness / 2.0f, -height / 2.0f - wallThickness },
            {  wallThickness / 2.0f,  height / 2.0f + wallThickness },
            { -wallThickness / 2.0f,  height / 2.0f + wallThickness }
        });
        
        ECS::ColliderComponent collider(shape);
        collider.material.friction = 0.0f;
        collider.material.restitution = 1.0f;
        collider.material.density = 0.0f;
        
        cs.AddComponent(leftWall, std::move(t));
        cs.AddComponent(leftWall, std::move(body));
        cs.AddComponent(leftWall, std::move(collider));
    }
    
    // Right wall
    {
        ECS::EntityID rightWall = entities.CreateEntity();
        
        ECS::TransformComponent t;
        t.position = { width + wallThickness / 2.0f, height / 2.0f };
        t.previousPosition = t.position;
        t.rotation = 0.0f;
        t.previousRotation = 0.0f;
        
        ECS::PhysicsBodyComponent body;
        body.isStatic = true;
        body.UpdateMassProperties();
        
        ECS::ColliderComponent::PolygonShape shape({
            { -wallThickness / 2.0f, -height / 2.0f - wallThickness },
            {  wallThickness / 2.0f, -height / 2.0f - wallThickness },
            {  wallThickness / 2.0f,  height / 2.0f + wallThickness },
            { -wallThickness / 2.0f,  height / 2.0f + wallThickness }
        });
        
        ECS::ColliderComponent collider(shape);
        collider.material.friction = 0.0f;
        collider.material.restitution = 1.0f;
        collider.material.density = 0.0f;
        
        cs.AddComponent(rightWall, std::move(t));
        cs.AddComponent(rightWall, std::move(body));
        cs.AddComponent(rightWall, std::move(collider));
    }
}

// ============================================================================
//  CreatePaddle
// ============================================================================
void BreakoutDemo::CreatePaddle()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    m_PaddleEntity = entities.CreateEntity();

    // Transform
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    
    ECS::TransformComponent t;
    t.position = { width / 2.0f, PADDLE_Y };
    t.previousPosition = t.position;  // CRITICAL: Set previousPosition to prevent rubberbanding
    t.rotation = 0.0f;
    t.previousRotation = 0.0f;

    // Physics body (static - we control position directly via transform)
    ECS::PhysicsBodyComponent body;
    body.isStatic = true;
    body.UpdateMassProperties();

    // Collider
    ECS::ColliderComponent::PolygonShape paddleShape({
        { -PADDLE_WIDTH / 2.0f, -PADDLE_HEIGHT / 2.0f },
        {  PADDLE_WIDTH / 2.0f, -PADDLE_HEIGHT / 2.0f },
        {  PADDLE_WIDTH / 2.0f,  PADDLE_HEIGHT / 2.0f },
        { -PADDLE_WIDTH / 2.0f,  PADDLE_HEIGHT / 2.0f }
    });

    ECS::ColliderComponent paddleCollider(paddleShape);
    paddleCollider.material.friction = 0.0f;
    paddleCollider.material.restitution = 1.0f;
    paddleCollider.material.density = 0.0f;

    // Render
    ECS::RenderComponent paddleRender({ PADDLE_WIDTH, PADDLE_HEIGHT }, { 0.2f, 0.6f, 1.0f });
    paddleRender.origin = { PADDLE_WIDTH / 2.0f, PADDLE_HEIGHT / 2.0f };

    cs.AddComponent(m_PaddleEntity, std::move(t));
    cs.AddComponent(m_PaddleEntity, std::move(body));
    cs.AddComponent(m_PaddleEntity, std::move(paddleCollider));
    cs.AddComponent(m_PaddleEntity, std::move(paddleRender));
}

// ============================================================================
//  CreateBall
// ============================================================================
void BreakoutDemo::CreateBall()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    m_BallEntity = entities.CreateEntity();

    // Start floating below the bricks with initial velocity
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    
    ECS::TransformComponent t;
    t.position = { width / 2.0f, BRICK_START_Y - 100.0f };  // Start below brick area
    t.previousPosition = t.position;
    t.rotation = 0.0f;
    t.previousRotation = 0.0f;

    // Physics body - will fall due to gravity
    ECS::PhysicsBodyComponent body;
    body.SetMass(1.0f);
    body.SetInertia(10.0f);  // Allow some rotation for angular effects
    body.UpdateMassProperties();
    body.isAwake = true;
    body.allowSleep = false;
    body.motionLocks.lockRotation = false;  // Allow rotation
    body.drag = 0.0f;  // No drag for consistent ball speed
    body.angularDamping = 0.1f;  // Slight angular damping
    
    // Ball is static until launched via SPACE
    body.velocity = { 0.0f, 0.0f };
    body.angularVelocity = 0.0f;

    // Collider (circle)
    ECS::ColliderComponent::CircleShape ballShape;
    ballShape.center = { 0.0f, 0.0f };
    ballShape.radius = BALL_RADIUS;

    ECS::ColliderComponent ballCollider(ballShape);
    ballCollider.material.friction = 0.0f;
    ballCollider.material.restitution = 1.0f;  // Perfect elastic bounce
    ballCollider.material.density = 0.001f;

    // Render
    ECS::RenderComponent ballRender({ BALL_RADIUS * 2.0f, BALL_RADIUS * 2.0f }, { 1.0f, 1.0f, 1.0f });
    ballRender.origin = { BALL_RADIUS, BALL_RADIUS };
    ballRender.shapeType = ECS::RenderComponent::ShapeType::Circle;

    cs.AddComponent(m_BallEntity, std::move(t));
    cs.AddComponent(m_BallEntity, std::move(body));
    cs.AddComponent(m_BallEntity, std::move(ballCollider));
    cs.AddComponent(m_BallEntity, std::move(ballRender));
}

// ============================================================================
//  GenerateBricks  –  random cohesive formation of square bricks
// ============================================================================
void BreakoutDemo::GenerateBricks()
{
    m_Bricks.clear();

    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);

    // Prepare color palette
    m_BrickColors = {
        { 1.0f, 0.0f, 0.0f },  // Red
        { 1.0f, 0.5f, 0.0f },  // Orange
        { 1.0f, 1.0f, 0.0f },  // Yellow
        { 0.0f, 1.0f, 0.0f },  // Green
        { 0.0f, 0.5f, 1.0f },  // Blue
        { 0.6f, 0.0f, 1.0f },  // Purple
        { 1.0f, 0.0f, 0.5f },  // Pink
    };

    const float cellSize = BRICK_SIZE + BRICK_GAP;
    const int gridCols = SHAPE_MAX_COLS;
    const int gridRows = SHAPE_MAX_ROWS;

    // Grid of placed cells
    std::vector<std::vector<bool>> placed(gridRows, std::vector<bool>(gridCols, false));

    // Target number of bricks, random per game
    std::uniform_int_distribution<int> countDist(TARGET_BRICK_MIN, TARGET_BRICK_MAX);
    int targetCount = countDist(m_Rng);

    // Start from a random cell near the center
    std::uniform_int_distribution<int> colDist(3, gridCols - 4);
    std::uniform_int_distribution<int> rowDist(2, gridRows - 3);
    int seedCol = colDist(m_Rng);
    int seedRow = rowDist(m_Rng);

    placed[seedRow][seedCol] = true;
    int placedCount = 1;

    // BFS flood fill: keep adding random neighbors until we reach target
    std::uniform_int_distribution<int> dirDist(0, 3);
    const int dirs[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};

    int attempts = 0;
    while (placedCount < targetCount && attempts < targetCount * 10)
    {
        ++attempts;

        // Collect all placed cells with at least one free neighbor
        std::vector<std::pair<int,int>> candidates;
        for (int r = 0; r < gridRows; ++r)
        {
            for (int c = 0; c < gridCols; ++c)
            {
                if (!placed[r][c]) continue;
                for (int d = 0; d < 4; ++d)
                {
                    int nr = r + dirs[d][0];
                    int nc = c + dirs[d][1];
                    if (nr >= 0 && nr < gridRows && nc >= 0 && nc < gridCols && !placed[nr][nc])
                    {
                        candidates.emplace_back(c, r);
                        break;
                    }
                }
            }
        }

        if (candidates.empty()) break;

        // Pick a random placed cell that has a free neighbor
        std::uniform_int_distribution<int> pickDist(0, static_cast<int>(candidates.size()) - 1);
        auto [pc, pr] = candidates[pickDist(m_Rng)];

        // Pick a random free neighbor
        std::vector<std::pair<int,int>> freeNeighbors;
        for (int d = 0; d < 4; ++d)
        {
            int nr = pr + dirs[d][0];
            int nc = pc + dirs[d][1];
            if (nr >= 0 && nr < gridRows && nc >= 0 && nc < gridCols && !placed[nr][nc])
            {
                freeNeighbors.emplace_back(nc, nr);
            }
        }
        if (freeNeighbors.empty()) continue;

        std::uniform_int_distribution<int> fnDist(0, static_cast<int>(freeNeighbors.size()) - 1);
        auto [nc, nr] = freeNeighbors[fnDist(m_Rng)];
        placed[nr][nc] = true;
        ++placedCount;
    }

    // Compute world position for each placed cell
    float totalWidth = static_cast<float>(gridCols) * cellSize;
    float startX = (static_cast<float>(width) - totalWidth) / 2.0f;

    // Shuffle color assignment for visual variety
    std::vector<Math::Vector3> shuffledColors = m_BrickColors;
    std::shuffle(shuffledColors.begin(), shuffledColors.end(), m_Rng);

    int brickIndex = 0;
    for (int r = 0; r < gridRows; ++r)
    {
        for (int c = 0; c < gridCols; ++c)
        {
            if (!placed[r][c]) continue;

            float x = startX + static_cast<float>(c) * cellSize + BRICK_SIZE / 2.0f;
            float y = BRICK_START_Y - static_cast<float>(r) * cellSize + BRICK_SIZE / 2.0f;
            const auto& color = shuffledColors[brickIndex % shuffledColors.size()];
            CreateBrick(x, y, color);
            ++brickIndex;
        }
    }

    std::cerr << "[BREAKOUT] Generated " << m_Bricks.size() << " bricks in cohesive shape\n";
}

// ============================================================================
//  CreateBrick  –  create a single square brick entity
// ============================================================================
void BreakoutDemo::CreateBrick(float x, float y, const Math::Vector3& color)
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    ECS::EntityID brickEntity = entities.CreateEntity();
    m_Bricks.push_back(brickEntity);

    // Transform
    ECS::TransformComponent t;
    t.position = { x, y };
    t.previousPosition = t.position;
    t.rotation = 0.0f;
    t.previousRotation = 0.0f;

    // Physics body (static)
    ECS::PhysicsBodyComponent body;
    body.isStatic = true;
    body.UpdateMassProperties();

    // Square collider
    float half = BRICK_SIZE / 2.0f;
    ECS::ColliderComponent::PolygonShape brickShape({
        { -half, -half },
        {  half, -half },
        {  half,  half },
        { -half,  half }
    });

    ECS::ColliderComponent brickCollider(brickShape);
    brickCollider.material.friction = 0.0f;
    brickCollider.material.restitution = 1.0f;
    brickCollider.material.density = 0.0f;

    // Render
    ECS::RenderComponent brickRender({ BRICK_SIZE, BRICK_SIZE }, color);
    brickRender.origin = { half, half };

    cs.AddComponent(brickEntity, std::move(t));
    cs.AddComponent(brickEntity, std::move(body));
    cs.AddComponent(brickEntity, std::move(brickCollider));
    cs.AddComponent(brickEntity, std::move(brickRender));
}

// ============================================================================
//  HandleInput
// ============================================================================
void BreakoutDemo::HandleInput(float deltaTime)
{
    auto& cs = GetComponentStore();
    
    if (!cs.HasComponent<ECS::TransformComponent>(m_PaddleEntity))
        return;
    
    auto& paddleTransform = cs.GetComponent<ECS::TransformComponent>(m_PaddleEntity);
    
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    
    // Paddle movement with arrow keys
    float moveX = 0.0f;
    if (Utils::InputManager::IsKeyDown(GLFW_KEY_LEFT) || Utils::InputManager::IsKeyDown(GLFW_KEY_A))
        moveX -= 1.0f;
    if (Utils::InputManager::IsKeyDown(GLFW_KEY_RIGHT) || Utils::InputManager::IsKeyDown(GLFW_KEY_D))
        moveX += 1.0f;
    
    paddleTransform.position.x += moveX * PADDLE_SPEED * deltaTime;
    
    // Clamp to screen bounds
    float halfWidth = PADDLE_WIDTH / 2.0f;
    paddleTransform.position.x = std::max(halfWidth, std::min(paddleTransform.position.x, width - halfWidth));
    paddleTransform.previousPosition.x = paddleTransform.position.x;
    
    // If ball not launched, keep it on paddle
    if (!m_BallLaunched && cs.HasComponent<ECS::TransformComponent>(m_BallEntity))
    {
        auto& ballTransform = cs.GetComponent<ECS::TransformComponent>(m_BallEntity);
        auto& ballBody = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BallEntity);
        ballTransform.position.x = paddleTransform.position.x;
        ballTransform.position.y = PADDLE_Y + PADDLE_HEIGHT / 2.0f + BALL_RADIUS + 2.0f;
        ballTransform.previousPosition = ballTransform.position;
        ballBody.velocity = { 0.0f, 0.0f };
    }
    
    // Launch ball with SPACE
    if (!m_BallLaunched && Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE))
    {
        m_BallLaunched = true;
        if (cs.HasComponent<ECS::PhysicsBodyComponent>(m_BallEntity))
        {
            auto& ballBody = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BallEntity);
            std::uniform_real_distribution<float> velDist(0.5f, 0.9f);
            float dir = (m_Rng() % 2 == 0) ? 1.0f : -1.0f;
            ballBody.velocity.x = BALL_SPEED * velDist(m_Rng) * dir;
            ballBody.velocity.y = BALL_SPEED;
        }
    }
    
    // Reset game with R
    if (Utils::InputManager::IsKeyPressed(GLFW_KEY_R))
        ResetGame();
}

// ============================================================================
//  CheckBrickCollisions  –  detect and destroy bricks hit by ball
// ============================================================================
void BreakoutDemo::CheckBrickCollisions()
{
    auto& cs = GetComponentStore();
    
    // Find the physics world entity
    auto worldEntities = cs.GetEntitiesWithComponent<ECS::PhysicsWorldComponent>();
    if (worldEntities.empty())
        return;
    
    const auto& world = cs.GetComponent<ECS::PhysicsWorldComponent>(worldEntities[0]);
    
    // Track which bricks to destroy
    std::vector<ECS::EntityID> bricksToDestroy;
    
    // Check all contact manifolds
    for (size_t i = 0; i < world.contactManifolds.size(); ++i)
    {
        const auto& manifold = world.contactManifolds[i];
        
        if (manifold.points.empty())
            continue;
        
        // Check if ball is involved in this collision
        bool isBallA = (manifold.entityIdA == m_BallEntity);
        bool isBallB = (manifold.entityIdB == m_BallEntity);
        
        if (!isBallA && !isBallB)
            continue;
        
        // Get the other entity (the brick or wall)
        ECS::EntityID otherEntity = isBallA ? manifold.entityIdB : manifold.entityIdA;
        
        // Check if it's a brick
        auto it = std::find(m_Bricks.begin(), m_Bricks.end(), otherEntity);
        if (it != m_Bricks.end())
        {
            // Brick hit - mark for destruction
            bricksToDestroy.push_back(otherEntity);
            m_Score += 10;  // 10 points per brick
        }
    }
    
    // Destroy marked bricks AFTER iterating (avoid iterator invalidation)
    if (!bricksToDestroy.empty())
    {
        auto& entities = GetEntityManager();
        for (auto brickId : bricksToDestroy)
        {
            entities.DestroyEntity(brickId, cs);
        }
        
        // Remove from tracking list
        for (auto brickId : bricksToDestroy)
        {
            auto it = std::remove(m_Bricks.begin(), m_Bricks.end(), brickId);
            m_Bricks.erase(it, m_Bricks.end());
        }
        
        std::cerr << "[BREAKOUT] Destroyed " << bricksToDestroy.size() << " bricks! Score: " << m_Score << "\n";
    }
}

// ============================================================================
//  ResetBall
// ============================================================================
void BreakoutDemo::ResetBall()
{
    auto& cs = GetComponentStore();
    
    if (!cs.HasComponent<ECS::TransformComponent>(m_BallEntity))
        return;
    
    auto& ballTransform = cs.GetComponent<ECS::TransformComponent>(m_BallEntity);
    auto& ballBody = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BallEntity);
    
    // Reset position to paddle
    const auto& paddleTransform = cs.GetComponent<ECS::TransformComponent>(m_PaddleEntity);
    ballTransform.position.x = paddleTransform.position.x;
    ballTransform.position.y = PADDLE_Y + PADDLE_HEIGHT / 2.0f + BALL_RADIUS + 2.0f;
    ballTransform.previousPosition = ballTransform.position;  // CRITICAL: Prevent twitching
    
    // Stop velocity
    ballBody.velocity = { 0.0f, 0.0f };
    ballBody.angularVelocity = 0.0f;
    
    // Reset collision-related state
    ballBody.isStatic = false;
    ballBody.allowSleep = false;
    ballBody.isAwake = true;
    
    m_BallLaunched = false;
}

// ============================================================================
//  ResetGame
// ============================================================================
void BreakoutDemo::ResetGame()
{
    std::cerr << "[BREAKOUT] Resetting game...\n";
    
    auto& entities = GetEntityManager();
    auto& cs = GetComponentStore();
    
    // Destroy all bricks
    for (auto brickId : m_Bricks)
    {
        entities.DestroyEntity(brickId, cs);
    }
    m_Bricks.clear();
    
    // Recreate bricks
    GenerateBricks();
    
    // Reset ball
    ResetBall();
    
    // Reset score and game state
    m_Score = 0;
    m_GameWon = false;
    
    std::cerr << "[BREAKOUT] Game reset! Press SPACE to launch ball.\n";
}
