#include "SimplePhysicsDemo.h"

#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/SystemManager.h"

#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"

#include <iostream>

using namespace Nyon;

SimplePhysicsDemo::SimplePhysicsDemo()
    : ECSApplication("Simple Physics Demo - Falling Box", 1280, 720)
{
}

void SimplePhysicsDemo::OnECSStart()
{
    // Create physics world with gravity
    auto& entities = GetEntityManager();
    auto& components = GetComponentStore();
    
    ECS::EntityID worldEntity = entities.CreateEntity();
    
    ECS::PhysicsWorldComponent world;
    world.gravity = { 0.0f, -980.0f };          // Y-negative up (standard physics coordinates)
    world.timeStep = 1.0f / 60.0f;
    world.velocityIterations = 8;
    world.positionIterations = 3;
    world.subStepCount = 4;
    world.enableSleep = true;
    world.enableWarmStarting = true;
    world.enableContinuous = false;
    
    // Enable debug visualization
    world.SetDebugDraw(
        /*shapes*/   true,
        /*joints*/   false,
        /*aabbs*/    false,
        /*contacts*/ true,
        /*islands*/  false
    );
    
    components.AddComponent(worldEntity, std::move(world));
    
    // Create platform and falling box
    CreatePlatform();
    CreateFallingBox();
}

void SimplePhysicsDemo::OnECSFixedUpdate(float deltaTime)
{
    // Check for collisions and break loop if detected
    if (!m_CollisionDetected)
    {
        auto& components = GetComponentStore();
        
        // Get physics world to access contact manifolds
        auto& worldEntities = components.GetEntitiesWithComponent<ECS::PhysicsWorldComponent>();
        if (!worldEntities.empty())
        {
            const auto& physicsWorld = components.GetComponent<ECS::PhysicsWorldComponent>(worldEntities[0]);
            
            // Check if any contacts are touching (collision detected)
            if (!physicsWorld.contactManifolds.empty())
            {
                for (const auto& manifold : physicsWorld.contactManifolds)
                {
                    if (manifold.touching)
                    {
                        m_CollisionDetected = true;
                        std::cerr << "[DEMO] Collision detected! Breaking loop..." << std::endl;
                        Close();  // Break the main loop
                        break;
                    }
                }
            }
        }
    }
}

void SimplePhysicsDemo::CreatePlatform()
{
    auto& entities = GetEntityManager();
    auto& components = GetComponentStore();
    
    // Create a large static platform at the bottom of the screen
    ECS::EntityID platform = entities.CreateEntity();
    
    ECS::TransformComponent t;
    t.position = { 640.0f, 70.0f };  // Center horizontally, near bottom (Y-up: small Y = bottom)
    
    ECS::PhysicsBodyComponent body;
    body.isStatic = true;  // Static body - doesn't move
    body.UpdateMassProperties();
    
    // Large rectangular platform (800x50 pixels)
    ECS::ColliderComponent::PolygonShape shape({
        {-400.0f, -25.0f},
        { 400.0f, -25.0f},
        { 400.0f,  25.0f},
        {-400.0f,  25.0f}
    });
    
    ECS::ColliderComponent collider(shape);
    collider.material.friction = 0.6f;
    collider.material.restitution = 0.1f;  // Slight bounce
    
    // Visual representation (dark gray platform)
    ECS::RenderComponent render(
        {800.0f, 50.0f},  // size
        {0.3f, 0.3f, 0.3f}  // dark gray color
    );
    
    components.AddComponent(platform, std::move(t));
    components.AddComponent(platform, std::move(body));
    components.AddComponent(platform, std::move(collider));
    components.AddComponent(platform, std::move(render));
}

void SimplePhysicsDemo::CreateFallingBox()
{
    auto& entities = GetEntityManager();
    auto& components = GetComponentStore();
    
    // Create a small falling box
    ECS::EntityID box = entities.CreateEntity();
    
    ECS::TransformComponent t;
    t.position = { 640.0f, 500.0f };  // Start in the air, above platform (Y-up: large Y = top)
    
    ECS::PhysicsBodyComponent body;
    body.mass = 2.0f;  // Dynamic body with mass
    body.UpdateMassProperties();
    
    // Small square box (50x50 pixels)
    ECS::ColliderComponent::PolygonShape shape({
        {-25.0f, -25.0f},
        { 25.0f, -25.0f},
        { 25.0f,  25.0f},
        {-25.0f,  25.0f}
    });
    
    ECS::ColliderComponent collider(shape);
    collider.material.friction = 0.4f;
    collider.material.restitution = 0.2f;  // Some bounce
    
    // Visual representation (blue box)
    ECS::RenderComponent render(
        {50.0f, 50.0f},  // size
        {0.2f, 0.4f, 1.0f}  // blue color
    );
    
    components.AddComponent(box, std::move(t));
    components.AddComponent(box, std::move(body));
    components.AddComponent(box, std::move(collider));
    components.AddComponent(box, std::move(render));
}
