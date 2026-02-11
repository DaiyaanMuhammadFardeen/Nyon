#include "GameApplication.h"
#include <iostream>
#include <cmath>

GameApplication::GameApplication()
    : ECSApplication("Smash Bros Clone - ECS Version", 1280, 720)
    , m_PlayerEntity(Nyon::ECS::INVALID_ENTITY)
{
    std::cerr << "[DEBUG] GameApplication constructor called" << std::endl;
    std::cout << "Game application initialized with ECS" << std::endl;
}

void GameApplication::OnECSStart()
{
    std::cerr << "[DEBUG] GameApplication::OnECSStart() called" << std::endl;
    
    // Create game entities
    CreatePlayer();
    CreatePlatforms();
    
    // Setup behaviors
    SetupPlayerBehavior();
    SetupPlatformBehaviors();
    
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