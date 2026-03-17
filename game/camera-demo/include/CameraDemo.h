#pragma once

#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/components/CameraComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/utils/InputManager.h"

/**
 * @brief Demo application showcasing the unified camera system
 * 
 * Features demonstrated:
 * - Creating a camera entity
 * - Camera follow behavior with smoothing
 * - Zoom and pan controls
 * - Multiple cameras with priority switching
 */
class CameraDemo : public Nyon::ECSApplication
{
public:
    CameraDemo()
        : Nyon::ECSApplication("Camera System Demo", 1280, 720)
    {
    }

protected:
    void OnECSStart() override
    {
        std::cerr << "\n========== CAMERA DEMO STARTING ==========\n";
        
        // Create physics world first
        CreatePhysicsWorld();
        
        // Create ground platform with walls
        CreateGround();
        
        // Create player entity FIRST - before camera so it knows what to follow
        CreatePlayer();
        
        // Create some boxes to look at
        CreateBoxes();
        
        // Create main camera with follow behavior - AFTER player exists
        CreateMainCamera();
        
        // Create second camera for demonstration
        CreateSecondCamera();
        
        std::cerr << "\n[DONE] All entities created. Game starting!\n";
        std::cerr << "==========================================\n\n";
        std::cerr << "Controls:\n"
                  << "  WASD - Move player\n"
                  << "  1 - Switch to main camera (priority 10, follows player)\n"
                  << "  2 - Switch to zoomed camera (priority 20, fixed view)\n"
                  << "  +/- - Zoom in/out (main camera only)\n"
                  << "  Arrow Keys - Pan camera (when not following)\n\n";
    }

    void OnECSFixedUpdate(float deltaTime) override
    {
        HandlePlayerInput(deltaTime);
        HandleCameraInput();
    }

private:
    Nyon::ECS::EntityID m_PlayerEntity;
    Nyon::ECS::EntityID m_MainCameraEntity;
    Nyon::ECS::EntityID m_SecondCameraEntity;
    Nyon::ECS::EntityID m_PhysicsWorldEntity;
    
    float m_MoveForce = 5000.0f;
    float m_JumpForce = -400.0f;
    
    void CreatePhysicsWorld()
    {
        auto& entities = GetEntityManager();
        auto& components = GetComponentStore();
        
        m_PhysicsWorldEntity = entities.CreateEntity();
        
        Nyon::ECS::PhysicsWorldComponent world;
        world.gravity = {0.0f, -500.0f};  // Moderate gravity pointing down
        world.SetDebugDraw(true, false, false, false, false);
        
        components.AddComponent(m_PhysicsWorldEntity, std::move(world));
        
        std::cerr << "[CAMERA DEMO] Physics world created with gravity\n";
    }
    
    void CreateGround()
    {
        auto& entities = GetEntityManager();
        auto& components = GetComponentStore();
        
        // Create a complete rectangular arena with 4 walls
        // Screen is 1280x720, so walls should form a box from (0,0) to (1280,720)
        
        // BOTTOM wall (ground platform) - spans full width at bottom of screen
        auto bottomEntity = entities.CreateEntity();
        
        Nyon::ECS::TransformComponent bottomTransform;
        bottomTransform.position = {640.0f, 20.0f};  // Center X, half-height from bottom
        bottomTransform.previousPosition = bottomTransform.position;
        bottomTransform.rotation = 0.0f;
        bottomTransform.previousRotation = 0.0f;
        
        Nyon::ECS::PhysicsBodyComponent bottomBody;
        bottomBody.isStatic = true;
        bottomBody.UpdateMassProperties();
        
        Nyon::ECS::ColliderComponent::PolygonShape bottomShape({
            {-640.0f, -20.0f},  // Left edge at x=0
            { 640.0f, -20.0f},  // Right edge at x=1280
            { 640.0f,  20.0f},  // Top edge at y=40
            {-640.0f,  20.0f}   // Bottom edge at y=0
        });
        Nyon::ECS::ColliderComponent bottomCollider(bottomShape);
        bottomCollider.material.friction = 0.6f;
        bottomCollider.material.restitution = 0.1f;  // Low bounce for floor
        
        Nyon::ECS::RenderComponent bottomRender({1280.0f, 160.0f}, {0.35f, 0.35f, 0.35f});
        bottomRender.origin = {640.0f, 20.0f};  // Pivot at bottom-left corner
        
        components.AddComponent(bottomEntity, std::move(bottomTransform));
        components.AddComponent(bottomEntity, std::move(bottomBody));
        components.AddComponent(bottomEntity, std::move(bottomCollider));
        components.AddComponent(bottomEntity, std::move(bottomRender));
        
        std::cerr << "[CAMERA DEMO] Bottom wall at y=20 (spans 0-1280)\n";
        
        // TOP wall (ceiling) - spans full width at top of screen
        auto topEntity = entities.CreateEntity();
        
        Nyon::ECS::TransformComponent topTransform;
        topTransform.position = {640.0f, 700.0f};  // Center X, 20px from top (720-20)
        topTransform.previousPosition = topTransform.position;
        topTransform.rotation = 0.0f;
        topTransform.previousRotation = 0.0f;
        
        Nyon::ECS::PhysicsBodyComponent topBody;
        topBody.isStatic = true;
        topBody.UpdateMassProperties();
        
        Nyon::ECS::ColliderComponent::PolygonShape topShape({
            {-640.0f, -20.0f},  // Left edge at x=0
            { 640.0f, -20.0f},  // Right edge at x=1280
            { 640.0f,  20.0f},  // Top edge at y=720
            {-640.0f,  20.0f}   // Bottom edge at y=680
        });
        Nyon::ECS::ColliderComponent topCollider(topShape);
        topCollider.material.friction = 0.6f;
        topCollider.material.restitution = 0.1f;
        
        Nyon::ECS::RenderComponent topRender({1280.0f, 40.0f}, {0.35f, 0.35f, 0.35f});
        topRender.origin = {640.0f, 20.0f};
        
        components.AddComponent(topEntity, std::move(topTransform));
        components.AddComponent(topEntity, std::move(topBody));
        components.AddComponent(topEntity, std::move(topCollider));
        components.AddComponent(topEntity, std::move(topRender));
        
        std::cerr << "[CAMERA DEMO] Top wall at y=700 (spans 0-1280)\n";
        
        // LEFT wall - spans full height at left edge of screen
        auto leftEntity = entities.CreateEntity();
        
        Nyon::ECS::TransformComponent leftTransform;
        leftTransform.position = {20.0f, 360.0f};  // 20px from left, center Y
        leftTransform.previousPosition = leftTransform.position;
        leftTransform.rotation = 0.0f;
        leftTransform.previousRotation = 0.0f;
        
        Nyon::ECS::PhysicsBodyComponent leftBody;
        leftBody.isStatic = true;
        leftBody.UpdateMassProperties();
        
        Nyon::ECS::ColliderComponent::PolygonShape leftShape({
            {-20.0f, -360.0f},  // Left edge at x=0
            { 20.0f, -360.0f},  // Right edge at x=40
            { 20.0f,  360.0f},  // Top edge at y=720
            {-20.0f,  360.0f}   // Bottom edge at y=0
        });
        Nyon::ECS::ColliderComponent leftCollider(leftShape);
        leftCollider.material.friction = 0.6f;
        leftCollider.material.restitution = 0.1f;
        
        Nyon::ECS::RenderComponent leftRender({40.0f, 720.0f}, {0.35f, 0.35f, 0.35f});
        leftRender.origin = {20.0f, 360.0f};
        
        components.AddComponent(leftEntity, std::move(leftTransform));
        components.AddComponent(leftEntity, std::move(leftBody));
        components.AddComponent(leftEntity, std::move(leftCollider));
        components.AddComponent(leftEntity, std::move(leftRender));
        
        std::cerr << "[CAMERA DEMO] Left wall at x=20 (spans 0-720)\n";
        
        // RIGHT wall - spans full height at right edge of screen
        auto rightEntity = entities.CreateEntity();
        
        Nyon::ECS::TransformComponent rightTransform;
        rightTransform.position = {1260.0f, 360.0f};  // 20px from right (1280-20), center Y
        rightTransform.previousPosition = rightTransform.position;
        rightTransform.rotation = 0.0f;
        rightTransform.previousRotation = 0.0f;
        
        Nyon::ECS::PhysicsBodyComponent rightBody;
        rightBody.isStatic = true;
        rightBody.UpdateMassProperties();
        
        Nyon::ECS::ColliderComponent::PolygonShape rightShape({
            {-20.0f, -360.0f},  // Left edge at x=1240
            { 20.0f, -360.0f},  // Right edge at x=1280
            { 20.0f,  360.0f},  // Top edge at y=720
            {-20.0f,  360.0f}   // Bottom edge at y=0
        });
        Nyon::ECS::ColliderComponent rightCollider(rightShape);
        rightCollider.material.friction = 0.6f;
        rightCollider.material.restitution = 0.1f;
        
        Nyon::ECS::RenderComponent rightRender({40.0f, 720.0f}, {0.35f, 0.35f, 0.35f});
        rightRender.origin = {20.0f, 360.0f};
        
        components.AddComponent(rightEntity, std::move(rightTransform));
        components.AddComponent(rightEntity, std::move(rightBody));
        components.AddComponent(rightEntity, std::move(rightCollider));
        components.AddComponent(rightEntity, std::move(rightRender));
        
        std::cerr << "[CAMERA DEMO] Right wall at x=1260 (spans 0-720)\n";
    }
    
    void CreateBoxes()
    {
        auto& entities = GetEntityManager();
        auto& components = GetComponentStore();
        
        // Create a nice pyramid of boxes in the center
        int boxCount = 0;
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col <= row; col++)
            {
                auto boxEntity = entities.CreateEntity();
                
                Nyon::ECS::TransformComponent transform;
                // Stack boxes in center of screen, above ground
                float xOffset = (col - row * 0.5f) * 45.0f;
                float yOffset = 150.0f + row * 45.0f;
                transform.position = {640.0f + xOffset, yOffset};
                transform.previousPosition = transform.position;
                
                Nyon::ECS::PhysicsBodyComponent body;
                body.mass = 1.0f;
                body.UpdateMassProperties();
                
                Nyon::ECS::ColliderComponent::PolygonShape shape({
                    {-20.0f, -20.0f},
                    { 20.0f, -20.0f},
                    { 20.0f,  20.0f},
                    {-20.0f,  20.0f}
                });
                Nyon::ECS::ColliderComponent collider(shape);
                collider.material.friction = 0.4f;
                collider.material.restitution = 0.3f;
                
                // Different colors for each row
                float r = 0.2f + row * 0.15f;
                float g = 0.5f + row * 0.1f;
                float b = 0.9f - row * 0.15f;
                
                Nyon::ECS::RenderComponent render({40.0f, 40.0f}, {r, g, b});
                render.origin = {20.0f, 20.0f};
                
                components.AddComponent(boxEntity, std::move(transform));
                components.AddComponent(boxEntity, std::move(body));
                components.AddComponent(boxEntity, std::move(collider));
                components.AddComponent(boxEntity, std::move(render));
                
                boxCount++;
            }
        }
        
        std::cerr << "[CAMERA DEMO] Created pyramid of " << boxCount << " boxes\n";
    }

    void CreatePlayer()
    {
        auto& entities = GetEntityManager();
        auto& components = GetComponentStore();
        
        m_PlayerEntity = entities.CreateEntity();
        
        // Transform - spawn in CENTER of screen where camera can see it
        Nyon::ECS::TransformComponent transform;
        transform.position = {640.0f, 360.0f};  // Center of screen (matching simple-physics-demo)
        transform.previousPosition = transform.position;  // Prevents interpolation glitching
        transform.rotation = 0.0f;
        transform.previousRotation = 0.0f;
        
        // Physics body
        Nyon::ECS::PhysicsBodyComponent body;
        body.mass = 2.0f;
        
        // Collider - CIRCLE instead of square
        Nyon::ECS::ColliderComponent::CircleShape circleCollider;
        circleCollider.center = {0.0f, 0.0f};  // Centered on transform
        circleCollider.radius = 25.0f;  // Same size as the square was
        
        Nyon::ECS::ColliderComponent collider(circleCollider);
        collider.material.friction = 0.4f;
        collider.material.restitution = 0.1f;  // Low bounce for better control
        collider.material.density = 0.0008f;
        
        // Calculate inertia from collider shape
        {
            float area = collider.CalculateArea();  // π * r²
            float density = (area > 0.0f) ? body.mass / area : 0.0f;
            body.SetInertia(collider.CalculateInertiaPerUnitMass() * body.mass);
        }
        
        // LOCK ROTATION - Circles don't rotate visually anyway, but prevents any issues
        body.motionLocks.lockRotation = true;
        
        // Render component - CIRCLE (orange)
        Nyon::ECS::RenderComponent render({50.0f, 50.0f}, {1.0f, 0.5f, 0.0f});
        render.shapeType = Nyon::ECS::RenderComponent::ShapeType::Circle;
        render.origin = {25.0f, 25.0f};  // Center of circle
        
        components.AddComponent(m_PlayerEntity, std::move(transform));
        components.AddComponent(m_PlayerEntity, std::move(body));
        components.AddComponent(m_PlayerEntity, std::move(collider));
        components.AddComponent(m_PlayerEntity, std::move(render));
        
        std::cerr << "[CAMERA DEMO] Player created at center (640, 360) as CIRCLE\n";
    }
    
    void CreateMainCamera()
    {
        auto& entities = GetEntityManager();
        auto& components = GetComponentStore();
        
        m_MainCameraEntity = entities.CreateEntity();
        
        // Camera doesn't need a transform - it has its own position in CameraComponent
        Nyon::ECS::CameraComponent camera;
        camera.camera.position = {640.0f, 360.0f};  // Center of 1280x720 screen
        camera.camera.zoom = 1.0f;
        camera.camera.rotation = 0.0f;
        camera.priority = 10.0f;
        camera.isActive = true;
        camera.layer = 0;
        
        // Set up smooth follow behavior - follow the player
        camera.followTarget = true;
        camera.targetEntity = m_PlayerEntity;
        camera.followOffset = {0.0f, 50.0f};  // Slightly above player
        camera.followSmoothness = 0.15f;  // Smooth follow (0 = instant, 1 = very smooth)
        
        components.AddComponent(m_MainCameraEntity, std::move(camera));
        
        std::cerr << "[CAMERA DEMO] Main camera created at (640,360) - will follow player at (" 
                  << "640,400)\n";
    }
    
    void CreateSecondCamera()
    {
        auto& entities = GetEntityManager();
        auto& components = GetComponentStore();
        
        m_SecondCameraEntity = entities.CreateEntity();
        
        Nyon::ECS::CameraComponent camera;
        camera.camera.position = {640.0f, 360.0f};
        camera.camera.zoom = 2.0f;  // Zoomed in 2x
        camera.camera.rotation = 0.0f;
        camera.priority = 20.0f;  // Higher priority than main camera
        camera.isActive = false;  // Start inactive
        camera.layer = 0;
        
        // This camera does NOT follow - fixed position
        camera.followTarget = false;
        
        components.AddComponent(m_SecondCameraEntity, std::move(camera));
        
        std::cerr << "[CAMERA DEMO] Second camera created (zoomed, higher priority)\n";
    }
    
    void HandlePlayerInput(float deltaTime)
    {
        auto& components = GetComponentStore();
        
        if (!components.HasComponent<Nyon::ECS::PhysicsBodyComponent>(m_PlayerEntity))
            return;
            
        auto& body = components.GetComponent<Nyon::ECS::PhysicsBodyComponent>(m_PlayerEntity);
        auto& transform = components.GetComponent<Nyon::ECS::TransformComponent>(m_PlayerEntity);
        
        // Debug: Print player position every 60 frames
        static int frameCount = 0;
        if (++frameCount % 60 == 0) {
            std::cerr << "[DEBUG] Player pos: (" << transform.position.x << ", " 
                      << transform.position.y << ") vel: (" << body.velocity.x 
                      << ", " << body.velocity.y << ")\n";
        }
        
        // Read input
        bool left = Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_A);
        bool right = Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_D);
        bool jumpPressed = Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_W);
        
        // Apply forces
        if (left)
        {
            body.ApplyForce({-m_MoveForce * deltaTime, 0.0f});
        }
        if (right)
        {
            body.ApplyForce({m_MoveForce * deltaTime, 0.0f});
        }
        if (jumpPressed && body.isGrounded)
        {
            body.ApplyLinearImpulse({0.0f, m_JumpForce});
        }
    }
    
    void HandleCameraInput()
    {
        auto& components = GetComponentStore();
        
        // Camera switching
        if (Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_1))
        {
            // Activate main camera
            if (components.HasComponent<Nyon::ECS::CameraComponent>(m_MainCameraEntity))
            {
                auto& cam = components.GetComponent<Nyon::ECS::CameraComponent>(m_MainCameraEntity);
                cam.isActive = true;
                cam.priority = 10.0f;
                std::cerr << "[CAMERA DEMO] Switched to main camera (follow mode)\n";
            }
        }
        
        if (Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_2))
        {
            // Activate second camera
            if (components.HasComponent<Nyon::ECS::CameraComponent>(m_SecondCameraEntity))
            {
                auto& cam = components.GetComponent<Nyon::ECS::CameraComponent>(m_SecondCameraEntity);
                cam.isActive = true;
                cam.priority = 20.0f;  // Higher priority
                std::cerr << "[CAMERA DEMO] Switched to zoomed camera (fixed position)\n";
            }
        }
        
        // Zoom controls for main camera
        if (components.HasComponent<Nyon::ECS::CameraComponent>(m_MainCameraEntity))
        {
            auto& cam = components.GetComponent<Nyon::ECS::CameraComponent>(m_MainCameraEntity);
            
            if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_EQUAL))
            {
                cam.camera.zoom += 0.05f;
                std::cerr << "[CAMERA DEMO] Zoom: " << cam.camera.zoom << "\n";
            }
            if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_MINUS))
            {
                cam.camera.zoom = std::max(0.1f, cam.camera.zoom - 0.05f);
                std::cerr << "[CAMERA DEMO] Zoom: " << cam.camera.zoom << "\n";
            }
            
            // Pan controls (when not following)
            if (!cam.followTarget)
            {
                const float panSpeed = 500.0f;
                if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_UP))
                {
                    cam.camera.position.y += panSpeed * 0.016f;
                }
                if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_DOWN))
                {
                    cam.camera.position.y -= panSpeed * 0.016f;
                }
                if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_LEFT))
                {
                    cam.camera.position.x -= panSpeed * 0.016f;
                }
                if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_RIGHT))
                {
                    cam.camera.position.x += panSpeed * 0.016f;
                }
            }
        }
    }
};
