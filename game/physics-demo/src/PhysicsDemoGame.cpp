#include "PhysicsDemoGame.h"
#include <iostream>
#include <cmath>

namespace Game
{
    PhysicsDemoGame::PhysicsDemoGame() : 
        ECSApplication("Nyon Physics Demo - Interactive Playground", 1280, 720),
        m_RandomGenerator(std::random_device{}()),
        m_FloatDistribution(0.0f, 1.0f)
    {
    }
    
    void PhysicsDemoGame::OnECSStart()
    {
        std::cout << "[DEBUG] PhysicsDemoGame::OnECSStart() called\n";
        
        // Parent initialization is handled automatically
        
        // First create physics world component BEFORE adding systems
        // This ensures CollisionPipelineSystem and ConstraintSolverSystem can access it during Initialize()
        std::cout << "[DEBUG] Creating physics world component first\n";
        auto& entityManager = GetEntityManager();
        auto& componentStore = GetComponentStore();
        auto worldEntity = entityManager.CreateEntity();
        componentStore.AddComponent<Nyon::ECS::PhysicsWorldComponent>(worldEntity, Nyon::ECS::PhysicsWorldComponent());
        
        // Register physics systems in correct physical order:
        // 1. PhysicsIntegrationSystem (apply forces, integrate velocities, boundary enforcement)
        // 2. CollisionPipelineSystem (broad + narrow phase, generate manifolds)  
        // 3. ConstraintSolverSystem (solve velocity/position constraints)
        // 4. DebugRenderSystem (overlay debug info)
        std::cout << "[DEBUG] Getting system manager\n";
        auto& systemManager = GetSystemManager();
        
        std::cout << "[DEBUG] Adding PhysicsIntegrationSystem\n";
        systemManager.AddSystem(std::make_unique<Nyon::ECS::PhysicsIntegrationSystem>());
        
        std::cout << "[DEBUG] Adding CollisionPipelineSystem\n";
        systemManager.AddSystem(std::make_unique<Nyon::ECS::CollisionPipelineSystem>());
        
        std::cout << "[DEBUG] Adding ConstraintSolverSystem\n";
        systemManager.AddSystem(std::make_unique<Nyon::ECS::ConstraintSolverSystem>());
        
        // REMOVED: ContinuousCollisionSystem to avoid conflict with CollisionPipelineSystem
        // ContinuousCollisionSystem was causing double collision resolution and conflicting corrections
        
        // REMOVED: TransformPhysicsSyncSystem to prevent double position integration
        // TransformPhysicsSyncSystem was causing double integration - position is already 
        // integrated by PhysicsIntegrationSystem with proper deltaTime scaling
        
        std::cout << "[DEBUG] Adding DebugRenderSystem\n";
        systemManager.AddSystem(std::make_unique<Nyon::ECS::DebugRenderSystem>());
        
        std::cout << "[DEBUG] All systems added successfully\n";
        
        // Create game scene
        std::cout << "[DEBUG] Creating environment\n";
        CreateEnvironment();
        std::cout << "[DEBUG] Creating player\n";
        CreatePlayer();
        std::cout << "[DEBUG] Creating interactive objects\n";
        CreateInteractiveObjects();
        std::cout << "[DEBUG] Creating obstacles\n";
        CreateObstacles();
        
        std::cout << "Physics demo game initialized with " << m_GameEntities.size() << " entities\n";
        std::cout << "Controls:\n";
        std::cout << "  Arrow Keys - Move player\n";
        std::cout << "  Space - Jump\n";
        std::cout << "  F1 - Toggle debug rendering\n";
        std::cout << "  R - Reset scene\n";
        std::cout << "  Objects spawn automatically every 2 seconds\n";
    }
    
    void PhysicsDemoGame::OnECSUpdate(float deltaTime)
    {
        // Handle input
        // Using static methods directly from InputManager
        
        // Toggle debug rendering
        if (Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_F1))
        {
            m_ShowDebug = !m_ShowDebug;
            std::cout << "Debug rendering " << (m_ShowDebug ? "enabled" : "disabled") << std::endl;
            
            // Note: System enabling/disabling would require extending the SystemManager API
        }
        
        // Restart scene
        if (Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_R))
        {
            std::cout << "Restarting scene..." << std::endl;
            RestartScene();
        }
        
        // Handle player input
        HandlePlayerInput(deltaTime);
        
        // Spawn random objects periodically
        m_SpawnTimer += deltaTime;
        if (m_SpawnTimer >= SPAWN_INTERVAL)
        {
            SpawnRandomObject();
            m_SpawnTimer = 0.0f;
        }
        
        // Parent update is handled automatically
    }
    
    void PhysicsDemoGame::CreateEnvironment()
    {
        // Create ground platforms
        auto groundId = CreateBox({640, 680}, {1200, 40}, 0.0f, true);
        auto& groundCollider = GetComponentStore().GetComponent<Nyon::ECS::ColliderComponent>(groundId);
        groundCollider.color = {0.3f, 0.6f, 0.3f}; // Green ground
        auto& groundRenderer = GetComponentStore().GetComponent<Nyon::ECS::RenderComponent>(groundId);
        groundRenderer.color = {0.3f, 0.6f, 0.3f}; // Green ground
        m_GameEntities.push_back(groundId);
        
        // Create walls
        auto leftWallId = CreateBox({20, 360}, {40, 720}, 0.0f, true);
        auto rightWallId = CreateBox({1260, 360}, {40, 720}, 0.0f, true);
        auto& leftWallCollider = GetComponentStore().GetComponent<Nyon::ECS::ColliderComponent>(leftWallId);
        auto& rightWallCollider = GetComponentStore().GetComponent<Nyon::ECS::ColliderComponent>(rightWallId);
        leftWallCollider.color = {0.5f, 0.5f, 0.5f};
        rightWallCollider.color = {0.5f, 0.5f, 0.5f};
        
        auto& leftWallRenderer = GetComponentStore().GetComponent<Nyon::ECS::RenderComponent>(leftWallId);
        auto& rightWallRenderer = GetComponentStore().GetComponent<Nyon::ECS::RenderComponent>(rightWallId);
        leftWallRenderer.color = {0.5f, 0.5f, 0.5f};
        rightWallRenderer.color = {0.5f, 0.5f, 0.5f};
        m_GameEntities.push_back(leftWallId);
        m_GameEntities.push_back(rightWallId);
        
        std::cout << "Created environment boundaries\n";
    }
    
    void PhysicsDemoGame::CreatePlayer()
    {
        // Create player character
        m_PlayerId = CreateCircle({640, 500}, 20.0f, 1.0f, false);
        auto& playerBody = GetComponentStore().GetComponent<Nyon::ECS::PhysicsBodyComponent>(m_PlayerId);
        auto& playerCollider = GetComponentStore().GetComponent<Nyon::ECS::ColliderComponent>(m_PlayerId);
        
        // Player-specific properties
        playerBody.friction = 0.1f;
        playerBody.restitution = 0.3f;
        playerBody.drag = 0.05f;
        playerCollider.color = {0.2f, 0.6f, 0.9f}; // Blue player
        
        auto& playerRenderer = GetComponentStore().GetComponent<Nyon::ECS::RenderComponent>(m_PlayerId);
        playerRenderer.color = {0.2f, 0.6f, 0.9f}; // Blue player
        
        m_GameEntities.push_back(m_PlayerId);
        std::cout << "Created player character\n";
    }
    
    void PhysicsDemoGame::CreateInteractiveObjects()
    {
        // Create interactive physics objects
        const int numObjects = 8;
        
        for (int i = 0; i < numObjects; ++i)
        {
            float x = 300.0f + (i % 4) * 150.0f;
            float y = 400.0f + (i / 4) * 80.0f;
            
            Nyon::ECS::EntityID objectId;
            
            // Alternate between different shapes
            switch (i % 3)
            {
                case 0: // Box
                    objectId = CreateBox({x, y}, {30, 30}, 1.0f, false);
                    break;
                case 1: // Circle
                    objectId = CreateCircle({x, y}, 20.0f, 1.0f, false);
                    break;
                case 2: // Capsule
                    objectId = CreateCapsule({x, y}, 40.0f, 15.0f, 1.0f, false);
                    break;
            }
            
            // Color based on type
            auto& collider = GetComponentStore().GetComponent<Nyon::ECS::ColliderComponent>(objectId);
            auto& renderer = GetComponentStore().GetComponent<Nyon::ECS::RenderComponent>(objectId);
            switch (i % 3)
            {
                case 0: 
                    collider.color = {0.9f, 0.3f, 0.3f}; 
                    renderer.color = {0.9f, 0.3f, 0.3f}; // Red boxes
                    break;
                case 1: 
                    collider.color = {0.3f, 0.9f, 0.3f}; 
                    renderer.color = {0.3f, 0.9f, 0.3f}; // Green circles
                    break;
                case 2: 
                    collider.color = {0.9f, 0.9f, 0.3f}; 
                    renderer.color = {0.9f, 0.9f, 0.3f}; // Yellow capsules
                    break;
            }
            
            m_GameEntities.push_back(objectId);
        }
        
        std::cout << "Created interactive objects\n";
    }
    
    void PhysicsDemoGame::CreateObstacles()
    {
        // Create some obstacles and ramps
        auto rampId = CreateBox({1000, 550}, {150, 20}, 0.0f, true);
        auto& rampTransform = GetComponentStore().GetComponent<Nyon::ECS::TransformComponent>(rampId);
        // Convert 25 degrees to radians: 25 * π/180 ≈ 0.436 radians
        rampTransform.rotation = 25.0f * 3.14159f / 180.0f;
        auto& rampCollider = GetComponentStore().GetComponent<Nyon::ECS::ColliderComponent>(rampId);
        rampCollider.color = {0.7f, 0.5f, 0.3f}; // Brown ramp
        auto& rampRenderer = GetComponentStore().GetComponent<Nyon::ECS::RenderComponent>(rampId);
        rampRenderer.color = {0.7f, 0.5f, 0.3f}; // Brown ramp
        m_GameEntities.push_back(rampId);
        
        // Platform above ramp
        auto platformId = CreateBox({1100, 450}, {80, 15}, 0.0f, true);
        auto& platformCollider = GetComponentStore().GetComponent<Nyon::ECS::ColliderComponent>(platformId);
        platformCollider.color = {0.6f, 0.4f, 0.8f}; // Purple platform
        auto& platformRenderer = GetComponentStore().GetComponent<Nyon::ECS::RenderComponent>(platformId);
        platformRenderer.color = {0.6f, 0.4f, 0.8f}; // Purple platform
        m_GameEntities.push_back(platformId);
        
        std::cout << "Created obstacles and platforms\n";
    }
    
    void PhysicsDemoGame::RestartScene()
    {
        auto& entityManager = GetEntityManager();
        auto& componentStore = GetComponentStore();
        
        // Destroy all game entities
        for (auto entityId : m_GameEntities)
        {
            // Simple destruction - in a real implementation, we'd check validity
            entityManager.DestroyEntity(entityId);
        }
        
        // Clear entity list
        m_GameEntities.clear();
        m_PlayerId = 0;
        
        // Reset spawn timer
        m_SpawnTimer = 0.0f;
        
        // Recreate the scene
        CreateEnvironment();
        CreatePlayer();
        CreateInteractiveObjects();
        CreateObstacles();
        
        std::cout << "Scene restarted with " << m_GameEntities.size() << " entities" << std::endl;
    }
    
    void PhysicsDemoGame::HandlePlayerInput(float deltaTime)
    {
        if (!m_PlayerId || !GetComponentStore().HasComponent<Nyon::ECS::PhysicsBodyComponent>(m_PlayerId))
            return;
            
        // Using static methods directly from InputManager
        auto& playerBody = GetComponentStore().GetComponent<Nyon::ECS::PhysicsBodyComponent>(m_PlayerId);
        
        // Tunable player movement parameters
        const float maxRunSpeed = 400.0f;
        const float accel = 6000.0f;
        const float damping = 8.0f;      // how quickly we slow down when no input
        const float jumpImpulse = 12000.0f;
        
        // Desired horizontal velocity based on input
        float targetVelX = 0.0f;
        if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_A))
        {
            targetVelX = -maxRunSpeed;
        }
        else if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_D))
        {
            targetVelX = maxRunSpeed;
        }
        
        // Smoothly move velocity toward target
        float deltaV = targetVelX - playerBody.velocity.x;
        float maxDelta = accel * deltaTime;
        if (deltaV > maxDelta) deltaV = maxDelta;
        else if (deltaV < -maxDelta) deltaV = -maxDelta;
        
        playerBody.velocity.x += deltaV;
        
        // Apply extra damping when there is no input to make stopping snappy
        if (std::abs(targetVelX) < 1e-3f)
        {
            playerBody.velocity.x -= playerBody.velocity.x * std::min(damping * deltaTime, 1.0f);
        }
        
        // Jumping: only when grounded and on key press (not held)
        if (Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE) && playerBody.IsStablyGrounded())
        {
            playerBody.ApplyLinearImpulse({0.0f, -jumpImpulse});
        }
    }
    
    void PhysicsDemoGame::SpawnRandomObject()
    {
        // Spawn random physics objects from the top
        float x = 200.0f + m_FloatDistribution(m_RandomGenerator) * 880.0f;
        float y = 100.0f;
        
        Nyon::ECS::EntityID objectId;
        float size = 15.0f + m_FloatDistribution(m_RandomGenerator) * 20.0f;
        
        // Random shape
        int shapeType = static_cast<int>(m_FloatDistribution(m_RandomGenerator) * 3);
        
        switch (shapeType)
        {
            case 0: // Box
                objectId = CreateBox({x, y}, {size, size}, 1.0f, false);
                break;
            case 1: // Circle
                objectId = CreateCircle({x, y}, size * 0.7f, 1.0f, false);
                break;
            case 2: // Capsule
                objectId = CreateCapsule({x, y}, size * 1.5f, size * 0.5f, 1.0f, false);
                break;
        }
        
        // Random color
        auto& collider = GetComponentStore().GetComponent<Nyon::ECS::ColliderComponent>(objectId);
        collider.color = {
            0.3f + m_FloatDistribution(m_RandomGenerator) * 0.7f,
            0.3f + m_FloatDistribution(m_RandomGenerator) * 0.7f,
            0.3f + m_FloatDistribution(m_RandomGenerator) * 0.7f
        };
        
        m_GameEntities.push_back(objectId);
        std::cout << "Spawned random object\n";
    }
    
    Nyon::ECS::EntityID PhysicsDemoGame::CreateBox(const Nyon::Math::Vector2& position, 
                                                  const Nyon::Math::Vector2& size,
                                                  float density,
                                                  bool isStatic)
    {
        auto& entityManager = GetEntityManager();
        auto& componentStore = GetComponentStore();
        
        auto entityId = entityManager.CreateEntity();
        
        // Add components
        componentStore.AddComponent<Nyon::ECS::TransformComponent>(entityId, Nyon::ECS::TransformComponent(position, {1.0f, 1.0f}, 0.0f));
        componentStore.AddComponent<Nyon::ECS::PhysicsBodyComponent>(entityId, Nyon::ECS::PhysicsBodyComponent());
        componentStore.AddComponent<Nyon::ECS::ColliderComponent>(entityId, Nyon::ECS::ColliderComponent());
        componentStore.AddComponent<Nyon::ECS::RenderComponent>(entityId, Nyon::ECS::RenderComponent());
        
        // Configure collider
        auto& collider = componentStore.GetComponent<Nyon::ECS::ColliderComponent>(entityId);
        collider.type = Nyon::ECS::ColliderComponent::ShapeType::Polygon;
        
        Nyon::ECS::ColliderComponent::PolygonShape boxShape;
        float halfWidth = size.x * 0.5f;
        float halfHeight = size.y * 0.5f;
        
        boxShape.vertices = {
            {-halfWidth, -halfHeight},
            {halfWidth, -halfHeight},
            {halfWidth, halfHeight},
            {-halfWidth, halfHeight}
        };
        
        boxShape.CalculateProperties();
        collider.shape = boxShape;
        collider.density = density;
        
        // Configure physics body (mass and inertia from collider)
        auto& body = componentStore.GetComponent<Nyon::ECS::PhysicsBodyComponent>(entityId);
        body.isStatic = isStatic;
        float area = collider.CalculateArea();
        body.mass = isStatic ? 0.0f : density * area;
        // Inertia for unit density, then scaled by mass / area
        float unitInertia = collider.CalculateInertiaForUnitDensity();
        if (!isStatic && area > 0.0f)
        {
            body.inertia = unitInertia * (body.mass / area);
        }
        body.UpdateMassProperties();
        
        // Configure renderer
        auto& renderer = componentStore.GetComponent<Nyon::ECS::RenderComponent>(entityId);
        renderer.size = size;
        renderer.origin = size * 0.5f; // Center origin
        renderer.visible = true;
        
        return entityId;
    }
    
    Nyon::ECS::EntityID PhysicsDemoGame::CreateCircle(const Nyon::Math::Vector2& position,
                                                     float radius,
                                                     float density,
                                                     bool isStatic)
    {
        auto& entityManager = GetEntityManager();
        auto& componentStore = GetComponentStore();
        
        auto entityId = entityManager.CreateEntity();
        
        componentStore.AddComponent<Nyon::ECS::TransformComponent>(entityId, Nyon::ECS::TransformComponent(position, {1.0f, 1.0f}, 0.0f));
        componentStore.AddComponent<Nyon::ECS::PhysicsBodyComponent>(entityId, Nyon::ECS::PhysicsBodyComponent());
        componentStore.AddComponent<Nyon::ECS::ColliderComponent>(entityId, Nyon::ECS::ColliderComponent());
        componentStore.AddComponent<Nyon::ECS::RenderComponent>(entityId, Nyon::ECS::RenderComponent());
        
        auto& collider = componentStore.GetComponent<Nyon::ECS::ColliderComponent>(entityId);
        collider.type = Nyon::ECS::ColliderComponent::ShapeType::Circle;
        collider.shape = Nyon::ECS::ColliderComponent::CircleShape{{0, 0}, radius};
        collider.density = density;
        
        auto& body = componentStore.GetComponent<Nyon::ECS::PhysicsBodyComponent>(entityId);
        body.isStatic = isStatic;
        float area = collider.CalculateArea();
        body.mass = isStatic ? 0.0f : density * area;
        float unitInertia = collider.CalculateInertiaForUnitDensity();
        if (!isStatic && area > 0.0f)
        {
            body.inertia = unitInertia * (body.mass / area);
        }
        body.UpdateMassProperties();
        
        auto& renderer = componentStore.GetComponent<Nyon::ECS::RenderComponent>(entityId);
        renderer.size = {radius * 2.0f, radius * 2.0f};
        renderer.origin = {radius, radius};
        renderer.visible = true;
        
        return entityId;
    }
    
    Nyon::ECS::EntityID PhysicsDemoGame::CreateCapsule(const Nyon::Math::Vector2& position,
                                                      float height,
                                                      float radius,
                                                      float density,
                                                      bool isStatic)
    {
        auto& entityManager = GetEntityManager();
        auto& componentStore = GetComponentStore();
        
        auto entityId = entityManager.CreateEntity();
        
        componentStore.AddComponent<Nyon::ECS::TransformComponent>(entityId, Nyon::ECS::TransformComponent(position, {1.0f, 1.0f}, 0.0f));
        componentStore.AddComponent<Nyon::ECS::PhysicsBodyComponent>(entityId, Nyon::ECS::PhysicsBodyComponent());
        componentStore.AddComponent<Nyon::ECS::ColliderComponent>(entityId, Nyon::ECS::ColliderComponent());
        componentStore.AddComponent<Nyon::ECS::RenderComponent>(entityId, Nyon::ECS::RenderComponent());
        
        auto& collider = componentStore.GetComponent<Nyon::ECS::ColliderComponent>(entityId);
        // For now approximate capsules with a box polygon so they participate
        // fully in the SAT manifold generation and constraint solver.
        collider.type = Nyon::ECS::ColliderComponent::ShapeType::Polygon;

        Nyon::ECS::ColliderComponent::PolygonShape poly;
        float halfWidth = radius;
        float halfHeight = height * 0.5f;
        poly.vertices = {
            {-halfWidth, -halfHeight},
            { halfWidth, -halfHeight},
            { halfWidth,  halfHeight},
            {-halfWidth,  halfHeight}
        };
        poly.CalculateProperties();
        collider.shape = poly;
        collider.density = density;
        
        auto& body = componentStore.GetComponent<Nyon::ECS::PhysicsBodyComponent>(entityId);
        body.isStatic = isStatic;
        float area = collider.CalculateArea();
        body.mass = isStatic ? 0.0f : density * area;
        float unitInertia = collider.CalculateInertiaForUnitDensity();
        if (!isStatic && area > 0.0f)
        {
            body.inertia = unitInertia * (body.mass / area);
        }
        body.UpdateMassProperties();
        
        auto& renderer = componentStore.GetComponent<Nyon::ECS::RenderComponent>(entityId);
        renderer.size = {radius * 2.0f, height + radius * 2.0f};
        renderer.origin = {radius, (height + radius * 2.0f) * 0.5f};
        renderer.visible = true;
        
        return entityId;
    }
}