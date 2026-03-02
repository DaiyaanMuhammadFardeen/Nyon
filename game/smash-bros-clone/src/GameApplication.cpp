#include "GameApplication.h"
#include "nyon/ecs/systems/PhysicsWorldComponent.h"
#include "nyon/ecs/systems/ConstraintSolverSystem.h"
#include "nyon/ecs/systems/CollisionPipelineSystem.h"
#include "nyon/ecs/systems/TransformPhysicsSyncSystem.h"
#include "nyon/ecs/systems/DebugRenderSystem.h"
#include <iostream>
#include <cmath>

GameApplication::GameApplication()
    : ECSApplication("Nyon Physics Demo - Box2D Inspired", 1280, 720)
      , m_PlayerEntity(Nyon::ECS::INVALID_ENTITY)
      , m_ShowDebugPhysics(true)
{
    std::cerr << "[DEBUG] GameApplication constructor called" << std::endl;
    std::cout << "Game application initialized with ECS" << std::endl;
}

void GameApplication::OnECSStart()
{
    std::cerr << "[DEBUG] GameApplication::OnECSStart() called" << std::endl;

    // Setup physics world
    SetupPhysicsWorld();
    
    // Create game entities
    CreatePlayer();
    CreatePhysicsDemoScene();

    // Setup behaviors
    SetupPlayerBehavior();
    SetupPlatformBehaviors();
    SetupDebugControls();

    std::cerr << "[DEBUG] GameApplication::OnECSStart() completed" << std::endl;
}

void GameApplication::OnECSUpdate(float deltaTime)
{
    std::cerr << "[DEBUG] GameApplication::OnECSUpdate() called" << std::endl;
    std::cerr << "[DEBUG] GameApplication::OnECSUpdate() completed" << std::endl;
}

void GameApplication::CreatePlayer()
{
    auto& entityManager = GetEntityManager();
    auto& componentStore = GetComponentStore();

    // Create player entity
    m_PlayerEntity = entityManager.CreateEntity();

    // Add transform component
    Nyon::ECS::TransformComponent transform({100.0f, 300.0f});
    componentStore.AddComponent(m_PlayerEntity, std::move(transform));

    // Add physics body component
    Nyon::ECS::PhysicsBodyComponent physics(1.0f, false); // mass=1.0, not static
    physics.friction = 0.2f;  // Increased friction for better ground control
    componentStore.AddComponent(m_PlayerEntity, std::move(physics));

    // Add collider component (polygon shape)
    Nyon::Math::Vector2 playerSize(32.0f, 32.0f);
    Nyon::ECS::ColliderComponent::PolygonShape playerShape = {
        {0.0f, 0.0f},
        {playerSize.x, 0.0f},
        {playerSize.x, playerSize.y},
        {0.0f, playerSize.y}
    };

    Nyon::ECS::ColliderComponent collider(playerShape);
    collider.color = {0.0f, 0.8f, 1.0f}; // Cyan
    componentStore.AddComponent(m_PlayerEntity, std::move(collider));

    // Add render component
    Nyon::ECS::RenderComponent render(playerSize, {0.0f, 0.8f, 1.0f}); // Cyan
    componentStore.AddComponent(m_PlayerEntity, std::move(render));

    std::cout << "Created player entity: " << m_PlayerEntity << std::endl;
}

void GameApplication::CreatePlatforms()
{
    auto& entityManager = GetEntityManager();
    auto& componentStore = GetComponentStore();

    // Platform configurations
    struct PlatformConfig {
        Nyon::Math::Vector2 position;
        Nyon::Math::Vector2 size;
        Nyon::Math::Vector3 color;
    };

    std::vector<PlatformConfig> platforms = {
        {{50.0f, 400.0f}, {200.0f, 32.0f}, {0.6f, 0.4f, 0.2f}},   // Brown - Starting platform
        {{350.0f, 350.0f}, {150.0f, 32.0f}, {0.8f, 0.6f, 0.4f}},  // Light brown - Floating platform
        {{600.0f, 280.0f}, {180.0f, 32.0f}, {0.4f, 0.8f, 0.4f}},  // Green - Higher platform
        {{850.0f, 220.0f}, {120.0f, 32.0f}, {0.8f, 0.2f, 0.2f}},  // Red - Gap platform
        {{1000.0f, 150.0f}, {300.0f, 32.0f}, {0.2f, 0.2f, 0.8f}}  // Blue - Final platform
    };

    for (const auto& config : platforms)
    {
        Nyon::ECS::EntityID platformEntity = entityManager.CreateEntity();
        m_PlatformEntities.push_back(platformEntity);

        // Add transform component
        Nyon::ECS::TransformComponent transform(config.position);
        componentStore.AddComponent(platformEntity, std::move(transform));

        // Add physics body component (static)
        Nyon::ECS::PhysicsBodyComponent physics(1.0f, true); // mass=1.0, static=true
        componentStore.AddComponent(platformEntity, std::move(physics));

        // Add collider component
        Nyon::ECS::ColliderComponent::PolygonShape platformShape = {
            {0.0f, 0.0f},
            {config.size.x, 0.0f},
            {config.size.x, config.size.y},
            {0.0f, config.size.y}
        };

        Nyon::ECS::ColliderComponent collider(platformShape);
        collider.color = config.color;
        componentStore.AddComponent(platformEntity, std::move(collider));

        // Add render component
        Nyon::ECS::RenderComponent render(config.size, config.color);
        componentStore.AddComponent(platformEntity, std::move(render));

        std::cout << "Created platform entity: " << platformEntity << std::endl;
    }
}

void GameApplication::SetupPlayerBehavior()
{
    auto& componentStore = GetComponentStore();

    // Create behavior component for player input and movement
    Nyon::ECS::BehaviorComponent behavior;

    // Set update function for input handling
    behavior.SetUpdateFunction([this](Nyon::ECS::EntityID entity, float deltaTime) {
            auto& componentStore = this->GetComponentStore();

            if (!componentStore.HasComponent<Nyon::ECS::PhysicsBodyComponent>(entity)) return;

            auto& physics = componentStore.GetComponent<Nyon::ECS::PhysicsBodyComponent>(entity);

            // Horizontal movement with acceleration
            float targetSpeed = 0.0f;
            const float MOVE_SPEED = 300.0f;
            const float ACCELERATION = 2000.0f;
            const float DECELERATION = 1500.0f;

            if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_A) || Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_LEFT))
            {
            targetSpeed = -MOVE_SPEED;
            }
            else if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_D) || Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_RIGHT))
            {
            targetSpeed = MOVE_SPEED;
            }

            // Smooth acceleration/deceleration
            if (targetSpeed != 0.0f) {
                float speedDiff = targetSpeed - physics.velocity.x;
                physics.velocity.x += speedDiff * ACCELERATION * deltaTime;
                if (std::abs(physics.velocity.x) > std::abs(targetSpeed)) {
                    physics.velocity.x = targetSpeed;
                }
            } else {
                // Deceleration when no input
                if (physics.velocity.x > 0) {
                    physics.velocity.x = std::max(0.0f, physics.velocity.x - DECELERATION * deltaTime);
                } else if (physics.velocity.x < 0) {
                    physics.velocity.x = std::min(0.0f, physics.velocity.x + DECELERATION * deltaTime);
                }
            }

            // Jumping with coyote time
            static float coyoteTimer = 0.0f;
            const float COYOTE_TIME = 0.1f;
            const float JUMP_FORCE = -400.0f;

            // Update coyote timer using stable grounded detection
            if (physics.IsStablyGrounded()) {
                coyoteTimer = COYOTE_TIME;
            } else {
                coyoteTimer -= deltaTime;
            }

            // Jump input handling
            if ((Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE) ||
                        Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_UP)) &&
                    (physics.IsStablyGrounded() || coyoteTimer > 0.0f))
            {
                physics.velocity.y = JUMP_FORCE;
                // Don't immediately set grounded to false - let collision system handle it
                coyoteTimer = 0.0f;
                std::cout << "[INPUT] Player jumped!" << std::endl;
            }

            std::cerr << "[DEBUG] Player velocity: (" << physics.velocity.x << ", " << physics.velocity.y
                << ") stably grounded: " << physics.IsStablyGrounded()
                << " frames: " << physics.groundedFrames << std::endl;
    });

    componentStore.AddComponent(m_PlayerEntity, std::move(behavior));
}

void GameApplication::SetupPlatformBehaviors()
{
    // Platforms are static, so they don't need behaviors
}

void GameApplication::SetupPhysicsWorld()
{
    auto& entityManager = GetEntityManager();
    auto& componentStore = GetComponentStore();
    
    // Create physics world entity
    Nyon::ECS::EntityID worldEntity = entityManager.CreateEntity();
    m_PhysicsWorld = componentStore.AddComponent<Nyon::ECS::PhysicsWorldComponent>(worldEntity);
    
    // Configure physics world
    m_PhysicsWorld->gravity = Nyon::Math::Vector2{0.0f, 980.0f};
    m_PhysicsWorld->timeStep = 1.0f / 60.0f;
    m_PhysicsWorld->velocityIterations = 8;
    m_PhysicsWorld->positionIterations = 3;
    
    // Enable debug visualization
    m_PhysicsWorld->drawShapes = m_ShowDebugPhysics;
    m_PhysicsWorld->drawAABBs = false;
    m_PhysicsWorld->drawContacts = false;
    
    std::cout << "Physics world configured\n";
}

void GameApplication::CreatePhysicsDemoScene()
{
    auto& entityManager = GetEntityManager();
    auto& componentStore = GetComponentStore();
    
    // Create ground platform
    CreateGroundPlatform();
    
    // Create falling objects
    CreateFallingObjects();
    
    // Create stacked boxes
    CreateStackedBoxes();
    
    // Create rolling ball
    CreateRollingBall();
    
    std::cout << "Created physics demo scene with " << m_PhysicsEntities.size() << " entities\n";
}

void GameApplication::CreateGroundPlatform()
{
    auto& entityManager = GetEntityManager();
    auto& componentStore = GetComponentStore();
    
    Nyon::ECS::EntityID groundEntity = entityManager.CreateEntity();
    m_PhysicsEntities.push_back(groundEntity);
    
    // Transform
    Nyon::ECS::TransformComponent transform({0.0f, 500.0f});
    componentStore.AddComponent(groundEntity, std::move(transform));
    
    // Static physics body
    Nyon::ECS::PhysicsBodyComponent physics(0.0f, true);
    physics.friction = 0.8f;
    componentStore.AddComponent(groundEntity, std::move(physics));
    
    // Ground collider
    Nyon::ECS::ColliderComponent::PolygonShape groundShape = {
        {-1000.0f, 0.0f}, {1000.0f, 0.0f},
        {1000.0f, 32.0f}, {-1000.0f, 32.0f}
    };
    Nyon::ECS::ColliderComponent collider(groundShape);
    collider.color = {0.5f, 0.5f, 0.5f}; // Gray
    componentStore.AddComponent(groundEntity, std::move(collider));
    
    // Render component
    Nyon::ECS::RenderComponent render({2000.0f, 32.0f}, {0.5f, 0.5f, 0.5f});
    componentStore.AddComponent(groundEntity, std::move(render));
}

void GameApplication::CreateFallingObjects()
{
    auto& entityManager = GetEntityManager();
    auto& componentStore = GetComponentStore();
    
    // Create boxes at different positions
    std::vector<Nyon::Math::Vector2> positions = {
        {100.0f, 100.0f}, {150.0f, 50.0f}, {200.0f, 75.0f},
        {250.0f, 25.0f}, {300.0f, 125.0f}
    };
    
    for (const auto& pos : positions)
    {
        Nyon::ECS::EntityID boxEntity = entityManager.CreateEntity();
        m_PhysicsEntities.push_back(boxEntity);
        
        // Transform
        Nyon::ECS::TransformComponent transform(pos);
        componentStore.AddComponent(boxEntity, std::move(transform));
        
        // Dynamic physics body
        Nyon::ECS::PhysicsBodyComponent physics(1.0f, false);
        physics.friction = 0.3f;
        physics.restitution = 0.2f;
        componentStore.AddComponent(boxEntity, std::move(physics));
        
        // Box collider
        Nyon::ECS::ColliderComponent::PolygonShape boxShape = {
            {-16.0f, -16.0f}, {16.0f, -16.0f},
            {16.0f, 16.0f}, {-16.0f, 16.0f}
        };
        Nyon::ECS::ColliderComponent collider(boxShape);
        collider.color = {0.0f, 1.0f, 0.0f}; // Green
        componentStore.AddComponent(boxEntity, std::move(collider));
        
        // Render component
        Nyon::ECS::RenderComponent render({32.0f, 32.0f}, {0.0f, 1.0f, 0.0f});
        componentStore.AddComponent(boxEntity, std::move(render));
    }
}

void GameApplication::CreateStackedBoxes()
{
    auto& entityManager = GetEntityManager();
    auto& componentStore = GetComponentStore();
    
    // Stack 5 boxes vertically
    Nyon::Math::Vector2 basePos(400.0f, 400.0f);
    for (int i = 0; i < 5; ++i)
    {
        Nyon::ECS::EntityID boxEntity = entityManager.CreateEntity();
        m_PhysicsEntities.push_back(boxEntity);
        
        Nyon::Math::Vector2 pos = basePos - Nyon::Math::Vector2{0.0f, static_cast<float>(i * 34)};
        
        // Transform
        Nyon::ECS::TransformComponent transform(pos);
        componentStore.AddComponent(boxEntity, std::move(transform));
        
        // Dynamic physics body
        Nyon::ECS::PhysicsBodyComponent physics(2.0f, false);
        physics.friction = 0.4f;
        physics.restitution = 0.1f;
        componentStore.AddComponent(boxEntity, std::move(physics));
        
        // Box collider
        Nyon::ECS::ColliderComponent::PolygonShape boxShape = {
            {-20.0f, -20.0f}, {20.0f, -20.0f},
            {20.0f, 20.0f}, {-20.0f, 20.0f}
        };
        Nyon::ECS::ColliderComponent collider(boxShape);
        collider.color = {1.0f, 1.0f, 0.0f}; // Yellow
        componentStore.AddComponent(boxEntity, std::move(collider));
        
        // Render component
        Nyon::ECS::RenderComponent render({40.0f, 40.0f}, {1.0f, 1.0f, 0.0f});
        componentStore.AddComponent(boxEntity, std::move(render));
    }
}

void GameApplication::CreateRollingBall()
{
    auto& entityManager = GetEntityManager();
    auto& componentStore = GetComponentStore();
    
    Nyon::ECS::EntityID ballEntity = entityManager.CreateEntity();
    m_PhysicsEntities.push_back(ballEntity);
    
    // Transform
    Nyon::ECS::TransformComponent transform({500.0f, 300.0f});
    componentStore.AddComponent(ballEntity, std::move(transform));
    
    // Dynamic physics body
    Nyon::ECS::PhysicsBodyComponent physics(0.5f, false);
    physics.friction = 0.2f;
    physics.restitution = 0.6f;
    componentStore.AddComponent(ballEntity, std::move(physics));
    
    // Circle collider
    Nyon::ECS::ColliderComponent collider(16.0f);
    collider.color = {1.0f, 0.0f, 0.0f}; // Red
    componentStore.AddComponent(ballEntity, std::move(collider));
    
    // Render component
    Nyon::ECS::RenderComponent render({32.0f, 32.0f}, {1.0f, 0.0f, 0.0f});
    componentStore.AddComponent(ballEntity, std::move(render));
}

void GameApplication::SetupDebugControls()
{
    auto& componentStore = GetComponentStore();
    
    // Create behavior component for debug controls
    Nyon::ECS::BehaviorComponent behavior;
    
    behavior.SetUpdateFunction([this](Nyon::ECS::EntityID entity, float deltaTime) {
        // Toggle debug visualization
        if (Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_F1))
        {
            m_ShowDebugPhysics = !m_ShowDebugPhysics;
            if (m_PhysicsWorld)
            {
                m_PhysicsWorld->drawShapes = m_ShowDebugPhysics;
            }
            std::cout << "Debug visualization " << (m_ShowDebugPhysics ? "enabled" : "disabled") << "\n";
        }
        
        // Reset scene
        if (Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_R))
        {
            std::cout << "Resetting physics scene...\n";
            // In a real implementation, this would recreate entities
        }
    });
    
    // Add to a dummy entity for global controls
    auto& entityManager = GetEntityManager();
    Nyon::ECS::EntityID controlEntity = entityManager.CreateEntity();
    componentStore.AddComponent(controlEntity, std::move(behavior));
}
