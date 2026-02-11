# Nyon Engine Tutorial: From Zero to Game

Welcome to the Nyon Engine tutorial! This guide will walk you through creating a complete 2D platformer game using the engine's Entity-Component-System (ECS) architecture.

## Table of Contents
1. [Getting Started](#getting-started)
2. [Understanding ECS Fundamentals](#understanding-ecs-fundamentals)
3. [Creating Your First Entity](#creating-your-first-entity)
4. [Building a Platformer](#building-a-platformer)
5. [Advanced ECS Patterns](#advanced-ecs-patterns)
6. [Physics and Collisions](#physics-and-collisions)
7. [Input Handling](#input-handling)
8. [Rendering and Visuals](#rendering-and-visuals)

## Getting Started

First, let's set up our project structure:

```bash
# Create your game directory
mkdir game/my-platformer
cd game/my-platformer
mkdir src include assets

# Create the basic file structure
touch include/PlatformerGame.h
touch src/PlatformerGame.cpp
touch src/main.cpp
touch CMakeLists.txt
```

## Understanding ECS Fundamentals

The Entity-Component-System (ECS) pattern is the heart of modern game engines. Let's break it down:

### Entities
Entities are just unique IDs that serve as containers for components:
```cpp
Nyon::ECS::EntityID player = entityManager.CreateEntity();
Nyon::ECS::EntityID enemy = entityManager.CreateEntity();
```

### Components
Components are pure data structures that hold specific attributes:
```cpp
// Transform component holds position, rotation, scale
TransformComponent transform({100.0f, 100.0f});

// Physics component holds velocity, mass, physical properties
PhysicsBodyComponent physics(1.0f, false); // mass=1.0, not static

// Render component holds visual properties
RenderComponent render({32.0f, 32.0f}, {1.0f, 0.0f, 0.0f}); // red square
```

### Systems
Systems process entities that have specific components:
```cpp
// PhysicsSystem automatically updates all entities with PhysicsBodyComponent
// RenderSystem automatically draws all entities with RenderComponent
// CollisionSystem handles collision detection between entities with ColliderComponent
```

## Creating Your First Entity

Let's create a simple player entity:

```cpp
// PlatformerGame.h
#pragma once

#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"

class PlatformerGame : public Nyon::ECSApplication
{
public:
    PlatformerGame();

protected:
    void OnECSStart() override;
    void OnECSUpdate(float deltaTime) override;

private:
    void CreatePlayer();
    void SetupPlayerControls();
    
    Nyon::ECS::EntityID m_PlayerEntity;
};
```

```cpp
// PlatformerGame.cpp
#include "PlatformerGame.h"
#include "nyon/utils/InputManager.h"
#include <iostream>

PlatformerGame::PlatformerGame()
    : ECSApplication("My Platformer", 1280, 720)
{
    std::cout << "Platformer game initialized!" << std::endl;
}

void PlatformerGame::OnECSStart()
{
    CreatePlayer();
    SetupPlayerControls();
}

void PlatformerGame::CreatePlayer()
{
    auto& entityManager = GetEntityManager();
    auto& componentStore = GetComponentStore();
    
    // Create the player entity
    m_PlayerEntity = entityManager.CreateEntity();
    
    // Add transform component (position at center of screen)
    componentStore.AddComponent(m_PlayerEntity, 
        Nyon::ECS::TransformComponent({640.0f, 360.0f}));
    
    // Add physics component (dynamic body with mass)
    componentStore.AddComponent(m_PlayerEntity,
        Nyon::ECS::PhysicsBodyComponent(1.0f, false));
    
    // Add collider component (square shape for collision)
    std::vector<Nyon::Math::Vector2> playerShape = {
        {0.0f, 0.0f}, {32.0f, 0.0f}, {32.0f, 32.0f}, {0.0f, 32.0f}
    };
    componentStore.AddComponent(m_PlayerEntity,
        Nyon::ECS::ColliderComponent(playerShape));
    
    // Add render component (blue square)
    componentStore.AddComponent(m_PlayerEntity,
        Nyon::ECS::RenderComponent({32.0f, 32.0f}, {0.0f, 0.5f, 1.0f}));
    
    std::cout << "Created player entity: " << m_PlayerEntity << std::endl;
}

void PlatformerGame::SetupPlayerControls()
{
    auto& componentStore = GetComponentStore();
    
    // Create behavior component for input handling
    Nyon::ECS::BehaviorComponent behavior;
    
    behavior.SetUpdateFunction([this](Nyon::ECS::EntityID entity, float deltaTime) {
        // Get the physics component to modify
        auto& physics = componentStore.GetComponent<Nyon::ECS::PhysicsBodyComponent>(entity);
        
        // Handle horizontal movement
        if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_A) || 
            Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_LEFT)) {
            physics.velocity.x = -200.0f; // Move left
        }
        else if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_D) || 
                 Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_RIGHT)) {
            physics.velocity.x = 200.0f;  // Move right
        }
        else {
            physics.velocity.x = 0.0f;    // Stop horizontal movement
        }
        
        // Handle jumping
        if ((Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE) || 
             Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_UP)) && 
            physics.isGrounded) {
            physics.velocity.y = -400.0f;  // Jump upward
            physics.isGrounded = false;    // Mark as airborne
        }
    });
    
    // Attach the behavior to the player
    componentStore.AddComponent(m_PlayerEntity, std::move(behavior));
}

void PlatformerGame::OnECSUpdate(float deltaTime)
{
    // The ECS systems handle most of the work automatically
    // You can add game-specific logic here if needed
    
    // Example: Display player position for debugging
    if (m_PlayerEntity != Nyon::ECS::INVALID_ENTITY) {
        auto& transform = GetComponentStore().GetComponent<Nyon::ECS::TransformComponent>(m_PlayerEntity);
        std::cout << "Player position: (" << transform.position.x << ", " << transform.position.y << ")\r";
        std::cout.flush();
    }
}
```

```cpp
// main.cpp
#include "PlatformerGame.h"

int main()
{
    PlatformerGame game;
    game.Run();
    return 0;
}
```

## Building a Platformer

Now let's add platforms and make it a real platformer:

### Adding Platforms

```cpp
// Add to PlatformerGame.h
private:
    void CreatePlatforms();
    std::vector<Nyon::ECS::EntityID> m_PlatformEntities;

// Add to PlatformerGame.cpp
void PlatformerGame::OnECSStart()
{
    CreatePlayer();
    SetupPlayerControls();
    CreatePlatforms();  // Add this line
}

void PlatformerGame::CreatePlatforms()
{
    auto& entityManager = GetEntityManager();
    auto& componentStore = GetComponentStore();
    
    // Define platform configurations
    struct PlatformConfig {
        Nyon::Math::Vector2 position;
        Nyon::Math::Vector2 size;
        Nyon::Math::Vector3 color;
    };
    
    std::vector<PlatformConfig> platforms = {
        {{100.0f, 500.0f}, {200.0f, 32.0f}, {0.6f, 0.4f, 0.2f}},  // Brown platform
        {{400.0f, 400.0f}, {150.0f, 32.0f}, {0.4f, 0.8f, 0.4f}},  // Green platform
        {{700.0f, 300.0f}, {180.0f, 32.0f}, {0.8f, 0.6f, 0.4f}},  // Light brown platform
    };
    
    for (const auto& config : platforms) {
        Nyon::ECS::EntityID platform = entityManager.CreateEntity();
        m_PlatformEntities.push_back(platform);
        
        // Add components to platform
        componentStore.AddComponent(platform, 
            Nyon::ECS::TransformComponent(config.position));
        
        componentStore.AddComponent(platform,
            Nyon::ECS::PhysicsBodyComponent(1.0f, true)); // Static body
        
        // Create platform shape
        std::vector<Nyon::Math::Vector2> platformShape = {
            {0.0f, 0.0f},
            {config.size.x, 0.0f},
            {config.size.x, config.size.y},
            {0.0f, config.size.y}
        };
        
        Nyon::ECS::ColliderComponent collider(platformShape);
        collider.color = config.color;
        componentStore.AddComponent(platform, std::move(collider));
        
        componentStore.AddComponent(platform,
            Nyon::ECS::RenderComponent(config.size, config.color));
    }
    
    std::cout << "Created " << platforms.size() << " platforms" << std::endl;
}
```

### Improving Player Physics

Let's add better physics behavior:

```cpp
void PlatformerGame::SetupPlayerControls()
{
    auto& componentStore = GetComponentStore();
    
    Nyon::ECS::BehaviorComponent behavior;
    
    behavior.SetUpdateFunction([this](Nyon::ECS::EntityID entity, float deltaTime) {
        auto& physics = componentStore.GetComponent<Nyon::ECS::PhysicsBodyComponent>(entity);
        auto& transform = componentStore.GetComponent<Nyon::ECS::TransformComponent>(entity);
        
        // Apply gravity manually (alternative to PhysicsSystem)
        const float GRAVITY = 980.0f;
        if (!physics.isStatic) {
            physics.velocity.y += GRAVITY * deltaTime;
        }
        
        // Handle horizontal movement with acceleration
        float targetSpeed = 0.0f;
        float acceleration = 2000.0f;
        float deceleration = 1500.0f;
        
        if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_A) || 
            Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_LEFT)) {
            targetSpeed = -200.0f;
        }
        else if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_D) || 
                 Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_RIGHT)) {
            targetSpeed = 200.0f;
        }
        
        // Smooth acceleration/deceleration
        if (targetSpeed != 0.0f) {
            float speedDiff = targetSpeed - physics.velocity.x;
            physics.velocity.x += speedDiff * acceleration * deltaTime;
            if (std::abs(physics.velocity.x) > std::abs(targetSpeed)) {
                physics.velocity.x = targetSpeed;
            }
        } else {
            // Deceleration when no input
            if (physics.velocity.x > 0) {
                physics.velocity.x = std::max(0.0f, physics.velocity.x - deceleration * deltaTime);
            } else if (physics.velocity.x < 0) {
                physics.velocity.x = std::min(0.0f, physics.velocity.x + deceleration * deltaTime);
            }
        }
        
        // Jumping with coyote time
        static float coyoteTimer = 0.0f;
        if (physics.isGrounded) {
            coyoteTimer = 0.1f; // 0.1 seconds of coyote time
        } else {
            coyoteTimer -= deltaTime;
        }
        
        if ((Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE) || 
             Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_UP)) && 
            (physics.isGrounded || coyoteTimer > 0.0f)) {
            physics.velocity.y = -400.0f;
            physics.isGrounded = false;
            coyoteTimer = 0.0f;
        }
        
        // Screen boundaries
        if (transform.position.x < 0) {
            transform.position.x = 0;
            physics.velocity.x = 0;
        }
        if (transform.position.x > 1280 - 32) {
            transform.position.x = 1280 - 32;
            physics.velocity.x = 0;
        }
    });
    
    componentStore.AddComponent(m_PlayerEntity, std::move(behavior));
}
```

## Advanced ECS Patterns

### Component Queries and Filtering

Learn to efficiently query entities with specific component combinations:

```cpp
void PlatformerGame::OnECSUpdate(float deltaTime)
{
    auto& componentStore = GetComponentStore();
    
    // Get all entities with physics bodies
    const auto& physicsEntities = componentStore.GetEntitiesWithComponent<Nyon::ECS::PhysicsBodyComponent>();
    
    // Process only moving entities
    for (auto entity : physicsEntities) {
        if (componentStore.HasComponent<Nyon::ECS::TransformComponent>(entity)) {
            auto& physics = componentStore.GetComponent<Nyon::ECS::PhysicsBodyComponent>(entity);
            auto& transform = componentStore.GetComponent<Nyon::ECS::TransformComponent>(entity);
            
            // Apply custom physics logic
            if (!physics.isStatic && physics.velocity.Length() > 10.0f) {
                // Add air resistance
                physics.velocity *= 0.98f;
            }
        }
    }
}
```

### Custom Components

Create your own components for game-specific data:

```cpp
// include/components/HealthComponent.h
#pragma once

struct HealthComponent {
    float currentHealth = 100.0f;
    float maxHealth = 100.0f;
    bool isAlive = true;
    
    void TakeDamage(float damage) {
        currentHealth -= damage;
        if (currentHealth <= 0) {
            currentHealth = 0;
            isAlive = false;
        }
    }
    
    void Heal(float amount) {
        currentHealth = std::min(maxHealth, currentHealth + amount);
        isAlive = true;
    }
};

// Usage in game:
void PlatformerGame::CreateEnemies()
{
    // Create enemy with health component
    Nyon::ECS::EntityID enemy = GetEntityManager().CreateEntity();
    
    GetComponentStore().AddComponent(enemy, TransformComponent({800.0f, 200.0f}));
    GetComponentStore().AddComponent(enemy, HealthComponent()); // Default 100 HP
    
    // Damage system example
    auto& health = GetComponentStore().GetComponent<HealthComponent>(enemy);
    health.TakeDamage(25.0f);
    std::cout << "Enemy health: " << health.currentHealth << std::endl;
}
```

### System Communication

Systems can communicate through component data:

```cpp
// Damage system that processes damage requests
class DamageSystem : public Nyon::ECS::System
{
public:
    void Update(float deltaTime) override
    {
        const auto& damageEntities = m_ComponentStore->GetEntitiesWithComponent<DamageComponent>();
        
        for (auto entity : damageEntities) {
            auto& damage = m_ComponentStore->GetComponent<DamageComponent>(entity);
            
            if (m_ComponentStore->HasComponent<HealthComponent>(damage.target)) {
                auto& health = m_ComponentStore->GetComponent<HealthComponent>(damage.target);
                health.TakeDamage(damage.amount);
                
                // Remove damage component after applying
                m_ComponentStore->RemoveComponent<DamageComponent>(entity);
            }
        }
    }
};

// Damage component
struct DamageComponent {
    Nyon::ECS::EntityID target;
    float amount;
};
```

## Physics and Collisions

### Advanced Collision Handling

```cpp
void PlatformerGame::SetupPlayerCollisionBehavior()
{
    auto& componentStore = GetComponentStore();
    
    Nyon::ECS::BehaviorComponent collisionBehavior;
    
    collisionBehavior.SetCollisionFunction([this](Nyon::ECS::EntityID entity, Nyon::ECS::EntityID other) {
        auto& componentStore = GetComponentStore();
        
        // Check if colliding with a platform
        if (componentStore.HasComponent<Nyon::ECS::RenderComponent>(other)) {
            auto& otherRender = componentStore.GetComponent<Nyon::ECS::RenderComponent>(other);
            
            // If it's a brown platform (ground)
            if (otherRender.color.r > 0.5f && otherRender.color.g < 0.5f && otherRender.color.b < 0.5f) {
                std::cout << "Landed on ground platform!" << std::endl;
                
                // Apply platform-specific effects
                auto& physics = componentStore.GetComponent<Nyon::ECS::PhysicsBodyComponent>(entity);
                physics.velocity.y *= 0.5f; // Reduce bounce
            }
        }
    });
    
    componentStore.AddComponent(m_PlayerEntity, std::move(collisionBehavior));
}
```

### Different Collider Shapes

```cpp
void PlatformerGame::CreateSpecialObjects()
{
    auto& entityManager = GetEntityManager();
    auto& componentStore = GetComponentStore();
    
    // Create a circular collectible
    Nyon::ECS::EntityID coin = entityManager.CreateEntity();
    componentStore.AddComponent(coin, TransformComponent({500.0f, 300.0f}));
    componentStore.AddComponent(coin, ColliderComponent(16.0f)); // Circle with 16px radius
    componentStore.AddComponent(coin, RenderComponent({32.0f, 32.0f}, {1.0f, 1.0f, 0.0f})); // Yellow
    
    // Create a moving platform with capsule shape
    Nyon::ECS::EntityID movingPlatform = entityManager.CreateEntity();
    componentStore.AddComponent(movingPlatform, TransformComponent({300.0f, 200.0f}));
    
    // Custom capsule collider (you'd need to extend ColliderComponent)
    std::vector<Nyon::Math::Vector2> capsulePoints = {
        {-20, -8}, {20, -8}, {20, 8}, {-20, 8}
    };
    componentStore.AddComponent(movingPlatform, ColliderComponent(capsulePoints));
    componentStore.AddComponent(movingPlatform, PhysicsBodyComponent(1.0f, true));
    componentStore.AddComponent(movingPlatform, RenderComponent({40.0f, 16.0f}, {0.8f, 0.2f, 0.8f}));
}
```

## Input Handling

### Advanced Input System

```cpp
void PlatformerGame::SetupAdvancedInput()
{
    auto& componentStore = GetComponentStore();
    
    Nyon::ECS::BehaviorComponent inputBehavior;
    
    // Track input states
    static bool wasJumping = false;
    static float jumpHoldTimer = 0.0f;
    const float MAX_JUMP_HOLD = 0.2f;
    
    inputBehavior.SetUpdateFunction([this](Nyon::ECS::EntityID entity, float deltaTime) {
        auto& physics = componentStore.GetComponent<Nyon::ECS::PhysicsBodyComponent>(entity);
        
        // Variable jump height - holding jump makes you jump higher
        bool isJumpHeld = Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_SPACE);
        bool jumpPressed = Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE);
        
        if (jumpPressed && physics.isGrounded) {
            physics.velocity.y = -350.0f; // Initial jump force
            jumpHoldTimer = 0.0f;
            wasJumping = true;
        }
        
        if (wasJumping && isJumpHeld && jumpHoldTimer < MAX_JUMP_HOLD) {
            // Continue applying upward force while holding jump
            physics.velocity.y -= 1500.0f * deltaTime;
            jumpHoldTimer += deltaTime;
        }
        
        if (!isJumpHeld || jumpHoldTimer >= MAX_JUMP_HOLD) {
            wasJumping = false;
        }
        
        // Dash ability
        static bool dashCooldown = false;
        static float dashTimer = 0.0f;
        const float DASH_COOLDOWN = 1.0f;
        const float DASH_DURATION = 0.1f;
        
        if (Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_LEFT_SHIFT) && !dashCooldown) {
            // Dash in current facing direction
            float dashDirection = (physics.velocity.x > 0) ? 1.0f : -1.0f;
            physics.velocity.x = dashDirection * 600.0f;
            dashCooldown = true;
            dashTimer = 0.0f;
        }
        
        if (dashCooldown) {
            dashTimer += deltaTime;
            if (dashTimer >= DASH_COOLDOWN) {
                dashCooldown = false;
            }
        }
    });
    
    componentStore.AddComponent(m_PlayerEntity, std::move(inputBehavior));
}
```

## Rendering and Visuals

### Animated Sprites

```cpp
// Custom animated render component
struct AnimatedRenderComponent {
    std::vector<std::string> texturePaths;
    int currentFrame = 0;
    float frameTime = 0.0f;
    float frameDuration = 0.1f;
    bool looping = true;
    bool playing = true;
    
    void UpdateAnimation(float deltaTime) {
        if (!playing || texturePaths.empty()) return;
        
        frameTime += deltaTime;
        if (frameTime >= frameDuration) {
            frameTime = 0.0f;
            currentFrame++;
            
            if (currentFrame >= static_cast<int>(texturePaths.size())) {
                if (looping) {
                    currentFrame = 0;
                } else {
                    currentFrame = texturePaths.size() - 1;
                    playing = false;
                }
            }
        }
    }
    
    const std::string& GetCurrentTexture() const {
        if (texturePaths.empty()) return "";
        return texturePaths[std::clamp(currentFrame, 0, static_cast<int>(texturePaths.size() - 1))];
    }
};

// Usage in animation system
class AnimationSystem : public Nyon::ECS::System
{
public:
    void Update(float deltaTime) override
    {
        const auto& animatedEntities = m_ComponentStore->GetEntitiesWithComponent<AnimatedRenderComponent>();
        
        for (auto entity : animatedEntities) {
            auto& animation = m_ComponentStore->GetComponent<AnimatedRenderComponent>(entity);
            animation.UpdateAnimation(deltaTime);
        }
    }
};
```

### Particle Effects

```cpp
void PlatformerGame::CreateParticles(Nyon::Math::Vector2 position, int count)
{
    auto& entityManager = GetEntityManager();
    auto& componentStore = GetComponentStore();
    
    for (int i = 0; i < count; i++) {
        Nyon::ECS::EntityID particle = entityManager.CreateEntity();
        
        // Random velocity
        float angle = (rand() / float(RAND_MAX)) * 2.0f * 3.14159f;
        float speed = 50.0f + (rand() / float(RAND_MAX)) * 150.0f;
        Nyon::Math::Vector2 velocity(cos(angle) * speed, sin(angle) * speed);
        
        componentStore.AddComponent(particle, TransformComponent(position));
        componentStore.AddComponent(particle, PhysicsBodyComponent(0.1f, false));
        componentStore.GetComponent<PhysicsBodyComponent>(particle).velocity = velocity;
        componentStore.AddComponent(particle, RenderComponent({4.0f, 4.0f}, {1.0f, 1.0f, 1.0f}));
        
        // Add lifetime behavior
        Nyon::ECS::BehaviorComponent lifetimeBehavior;
        lifetimeBehavior.SetUpdateFunction([this, particle](Nyon::ECS::EntityID entity, float deltaTime) {
            static float lifetime = 2.0f;
            lifetime -= deltaTime;
            if (lifetime <= 0.0f) {
                GetEntityManager().DestroyEntity(particle);
            }
        });
        componentStore.AddComponent(particle, std::move(lifetimeBehavior));
    }
}
```

## Complete CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10)
project(my_platformer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# Add executable
add_executable(my_platformer
    src/main.cpp
    src/PlatformerGame.cpp
)

# Include directories
target_include_directories(my_platformer PRIVATE include)
target_include_directories(my_platformer PRIVATE ../engine/include)

# Link libraries
target_link_libraries(my_platformer PRIVATE nyon_engine OpenGL::GL glfw glm)

# Set output directory
set_target_properties(my_platformer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/game/my-platformer
)
```

## Building and Running

```bash
# From your build directory
cd build
cmake ..
make my_platformer

# Run your game
./game/my-platformer/my_platformer
```

## Controls

- **A/D or Arrow Keys**: Move left/right
- **Space or Up Arrow**: Jump (hold for higher jumps)
- **Left Shift**: Dash
- **ESC**: Quit

## Next Steps

Congratulations! You've created a complete platformer using the Nyon Engine's ECS architecture. Here are some ideas to expand your game:

1. **Add Enemies**: Create enemy entities with AI behaviors
2. **Collectibles**: Implement coins, power-ups, and inventory system
3. **Level System**: Create multiple levels with progression
4. **Sound System**: Add audio feedback for actions and events
5. **UI System**: Create menus, HUD, and score displays
6. **Save System**: Implement game state saving and loading
7. **Networking**: Add multiplayer support (advanced)

The ECS architecture makes it easy to add new features by simply creating new components and systems. Experiment with different combinations to see what works best for your game!