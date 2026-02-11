#include"GameApplication.h"
#include "nyon/graphics/Renderer2D.h"
#include <iostream>
#include <cmath>
#include <vector>

GameApplication::GameApplication()
    : Application("Smash Bros Clone", 1280, 720)
{
    std::cerr << "[DEBUG] GameApplication constructor called" << std::endl;
    // Initialize game-specific systems
    std::cout << "Game application initialized" << std::endl;
}

void GameApplication::OnStart()
{
    std::cerr << "[DEBUG] GameApplication::OnStart() called" << std::endl;

    // Initialize renderer
    Nyon::Graphics::Renderer2D::Init();

    // Initialize input manager with the window from the parent Application class
    std::cerr << "[DEBUG] Initializing InputManager with window: " << GetWindow() << std::endl;
    Nyon::Utils::InputManager::Init(GetWindow());

    // Verify the window isvalid
    if (GetWindow() == nullptr) {
        std::cerr << "[ERROR] GameApplication::OnStart() - window is null!" << std::endl;
    } else {
        std::cerr << "[DEBUG] GameApplication::OnStart() - window is valid" << std::endl;
    }

    // Initialize player size first
    m_PlayerSize = Nyon::Math::Vector2(32.0f, 32.0f);
    m_PlayerColor = Nyon::Math::Vector3(0.0f, 0.8f, 1.0f); // Cyan

    // Initialize player - starting position above the first platform
    m_CurrentPlayerBody.position = Nyon::Math::Vector2(100.0f, 300.0f);
    m_CurrentPlayerBody.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
    m_CurrentPlayerBody.acceleration = Nyon::Math::Vector2(0.0f, 0.0f);
    m_CurrentPlayerBody.mass = 1.0f;
    m_CurrentPlayerBody.isStatic = false;

    // Set previous state equal to current state initially
    m_PreviousPlayerBody = m_CurrentPlayerBody;
    m_PreviousPlayerPosition = m_CurrentPlayerBody.position;

    // Define player as a square polygon for SAT collision
    m_PlayerShape = {
        Nyon::Math::Vector2(0.0f, 0.0f),              // bottom-left
        Nyon::Math::Vector2(m_PlayerSize.x, 0.0f),     // bottom-right
        Nyon::Math::Vector2(m_PlayerSize.x, m_PlayerSize.y), // top-right
        Nyon::Math::Vector2(0.0f, m_PlayerSize.y)      // top-left
    };

    m_IsGrounded = false;

    // Create multiple platforms for a platformer level
    // Platform 1 - Starting platform
    Platform platform1;
    platform1.size = Nyon::Math::Vector2(200.0f, 32.0f);
    platform1.color = Nyon::Math::Vector3(0.6f, 0.4f, 0.2f); // Brown
    platform1.body.position = Nyon::Math::Vector2(50.0f, 400.0f);
    platform1.body.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
    platform1.body.acceleration = Nyon::Math::Vector2(0.0f, 0.0f);
    platform1.body.mass = 1.0f;
    platform1.body.isStatic = true;
    platform1.previousBody = platform1.body;
    platform1.shape = {
        Nyon::Math::Vector2(0.0f, 0.0f),
        Nyon::Math::Vector2(platform1.size.x, 0.0f),
        Nyon::Math::Vector2(platform1.size.x, platform1.size.y),
        Nyon::Math::Vector2(0.0f, platform1.size.y)
    };
    m_Platforms.push_back(platform1);

    // Platform 2 - Floating platform
    Platform platform2;
    platform2.size = Nyon::Math::Vector2(150.0f, 32.0f);
    platform2.color = Nyon::Math::Vector3(0.8f, 0.6f, 0.4f); // Light brown
    platform2.body.position = Nyon::Math::Vector2(350.0f, 350.0f);
    platform2.body.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
    platform2.body.acceleration = Nyon::Math::Vector2(0.0f, 0.0f);
    platform2.body.mass = 1.0f;
    platform2.body.isStatic = true;
    platform2.previousBody = platform2.body;
    platform2.shape = {
        Nyon::Math::Vector2(0.0f, 0.0f),
        Nyon::Math::Vector2(platform2.size.x, 0.0f),
        Nyon::Math::Vector2(platform2.size.x, platform2.size.y),
        Nyon::Math::Vector2(0.0f, platform2.size.y)
    };
    m_Platforms.push_back(platform2);

    // Platform 3 - Higher platform
    Platform platform3;
    platform3.size = Nyon::Math::Vector2(180.0f, 32.0f);
    platform3.color = Nyon::Math::Vector3(0.4f, 0.8f, 0.4f); // Green
    platform3.body.position = Nyon::Math::Vector2(600.0f, 280.0f);
    platform3.body.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
    platform3.body.acceleration = Nyon::Math::Vector2(0.0f, 0.0f);
    platform3.body.mass = 1.0f;
    platform3.body.isStatic = true;
    platform3.previousBody = platform3.body;
    platform3.shape = {
        Nyon::Math::Vector2(0.0f, 0.0f),
        Nyon::Math::Vector2(platform3.size.x, 0.0f),
        Nyon::Math::Vector2(platform3.size.x, platform3.size.y),
        Nyon::Math::Vector2(0.0f, platform3.size.y)
    };
    m_Platforms.push_back(platform3);

    // Platform 4 - Gap platform
    Platform platform4;
    platform4.size = Nyon::Math::Vector2(120.0f, 32.0f);
    platform4.color = Nyon::Math::Vector3(0.8f, 0.2f, 0.2f); // Red
    platform4.body.position = Nyon::Math::Vector2(850.0f, 220.0f);
    platform4.body.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
    platform4.body.acceleration = Nyon::Math::Vector2(0.0f, 0.0f);
    platform4.body.mass = 1.0f;
    platform4.body.isStatic = true;
    platform4.previousBody = platform4.body;
    platform4.shape = {
        Nyon::Math::Vector2(0.0f, 0.0f),
        Nyon::Math::Vector2(platform4.size.x, 0.0f),
        Nyon::Math::Vector2(platform4.size.x, platform4.size.y),
        Nyon::Math::Vector2(0.0f, platform4.size.y)
    };
    m_Platforms.push_back(platform4);

    // Platform 5 - Final platform
    Platform platform5;
    platform5.size = Nyon::Math::Vector2(300.0f, 32.0f);
    platform5.color = Nyon::Math::Vector3(0.2f, 0.2f, 0.8f); // Blue
    platform5.body.position = Nyon::Math::Vector2(1000.0f, 150.0f);
    platform5.body.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
    platform5.body.acceleration = Nyon::Math::Vector2(0.0f, 0.0f);
    platform5.body.mass = 1.0f;
    platform5.body.isStatic = true;
    platform5.previousBody = platform5.body;
    platform5.shape = {
        Nyon::Math::Vector2(0.0f, 0.0f),
        Nyon::Math::Vector2(platform5.size.x, 0.0f),
        Nyon::Math::Vector2(platform5.size.x, platform5.size.y),
        Nyon::Math::Vector2(0.0f, platform5.size.y)
    };
    m_Platforms.push_back(platform5);

    std::cerr << "[DEBUG] GameApplication::OnStart() completed"<< std::endl;
}

void GameApplication::OnUpdate(float deltaTime)
{
    // This is for backward compatibility if someone calls it
    std::cerr << "[DEBUG] GameApplication::OnUpdate() called with delta time: " << deltaTime <<std::endl;
    HandleInput(deltaTime);
    UpdatePhysics(deltaTime);
}

void GameApplication::OnFixedUpdate(float deltaTime)
{
    std::cerr << "[DEBUG] GameApplication::OnFixedUpdate() called with fixed delta time: " << deltaTime << std::endl;

    // Copy current state to previous state before physics update
    m_PreviousPlayerBody = m_CurrentPlayerBody;
    for (auto& platform : m_Platforms) {
        platform.previousBody = platform.body;
    }

    // Update input
    std::cerr << "[DEBUG] Calling InputManager::Update()" << std::endl;
    Nyon::Utils::InputManager::Update();

    // Handle player input
    std::cerr << "[DEBUG] CallingHandleInput()" << std::endl;
    HandleInput(deltaTime);

    // Update physics
    std::cerr << "[DEBUG] Calling UpdatePhysics()" << std::endl;
    UpdatePhysics(deltaTime);

    std::cerr << "[DEBUG] GameApplication::OnFixedUpdate() completed" << std::endl;
}

void GameApplication::OnInterpolateAndRender(float alpha)
{
    std::cerr << "[DEBUG] GameApplication::OnInterpolateAndRender() called with alpha: " <<alpha << std::endl;

    // Clear screen
    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Begin rendering
    Nyon::Graphics::Renderer2D::BeginScene();

    // Draw all platforms
    for (const auto& platform : m_Platforms) {
        Nyon::Math::Vector2 interpolatedPlatformPos = interpolatePosition(platform.previousBody, platform.body, alpha);
        Nyon::Graphics::Renderer2D::DrawQuad(
                interpolatedPlatformPos,
                platform.size,
                Nyon::Math::Vector2(0.0f, 0.0f),
                platform.color
                );
    }

    // Draw player with interpolation
    Nyon::Math::Vector2 interpolatedPlayerPos = interpolatePosition(m_PreviousPlayerBody, m_CurrentPlayerBody, alpha);
    Nyon::Graphics::Renderer2D::DrawQuad(
            interpolatedPlayerPos,
            m_PlayerSize,
            Nyon::Math::Vector2(0.0f, 0.0f),
            m_PlayerColor
            );

    // End rendering
    Nyon::Graphics::Renderer2D::EndScene();

    std::cerr << "[DEBUG] GameApplication::OnInterpolateAndRender()completed" << std::endl;
}

void GameApplication::OnRender()
{
    // This method is kept for backward compatibility, but the interpolation render method is used instead
    std::cerr << "[DEBUG] GameApplication::OnRender() called(compatibility)" << std::endl;
}

Nyon::Math::Vector2 GameApplication::interpolatePosition(const Nyon::Utils::Physics::Body& prev,
        const Nyon::Utils::Physics::Body& curr,
        float alpha) const
{
    // Linear interpolation between previous and currentposition
    return Nyon::Math::Vector2(
            prev.position.x *(1.0f - alpha) + curr.position.x * alpha,
            prev.position.y * (1.0f - alpha) + curr.position.y * alpha
            );
}

void GameApplication::HandleInput(float deltaTime)
{
    std::cerr << "[DEBUG] GameApplication::HandleInput() called" << std::endl;

    // Move based on PLAYER_SPEED when keys arepressed
    if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_A) || Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_LEFT))
    {
        std::cerr << "[DEBUG] Moving left" << std::endl;
        m_CurrentPlayerBody.velocity.x = -PLAYER_SPEED;
    }
    else if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_D) || Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_RIGHT))
    {
        std::cerr << "[DEBUG] Moving right" << std::endl;
        m_CurrentPlayerBody.velocity.x = PLAYER_SPEED;
    }
    else
    {
        // Stop horizontal movement when no keys are pressed
        m_CurrentPlayerBody.velocity.x = 0.0f;
    }

    // Jumping
    if ((Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE) ||
                Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_UP)) && m_IsGrounded)
    {
        std::cerr << "[DEBUG] Jumping" << std::endl;
        //Apply jump impulse - reset vertical velocity to jump force
        m_CurrentPlayerBody.velocity.y =JUMP_FORCE;
        m_IsGrounded = false;
    }
    std::cerr << "[DEBUG] Player velocity: (" << m_CurrentPlayerBody.velocity.x << ", " << m_CurrentPlayerBody.velocity.y << ")"<< std::endl;
    std::cerr << "[DEBUG] Player position: (" << m_CurrentPlayerBody.position.x << ", " << m_CurrentPlayerBody.position.y << ")" << std::endl;

    std::cerr << "[DEBUG] GameApplication::HandleInput() completed" << std::endl;
}

void GameApplication::UpdatePhysics(float deltaTime)
{
    std::cerr << "[DEBUG] GameApplication::UpdatePhysics() called" << std::endl;
    std::cerr << "[DEBUG] Before physics - Player position: (" << m_CurrentPlayerBody.position.x << ", " << m_CurrentPlayerBody.position.y << ")" << std::endl;

    // Store previous grounded state but don't reset it yet
    // We'll determine grounded state based on current collisions
    bool wasGrounded = m_IsGrounded;
    
    // Apply physics with current grounded state
    Nyon::Utils::GravityPhysics::UpdateBody(m_CurrentPlayerBody, deltaTime, m_IsGrounded);

    // Boundary checks to prevent going off-screen
    if (m_CurrentPlayerBody.position.x < 0) {
        m_CurrentPlayerBody.position.x = 0;
    }
    if (m_CurrentPlayerBody.position.x + m_PlayerSize.x > 1280) {// Assuming window width is 1280
        m_CurrentPlayerBody.position.x = 1280 - m_PlayerSize.x;
    }

    // Check for ground collision (floor) FIRST
    if (m_CurrentPlayerBody.position.y + m_PlayerSize.y >= 650.0f)// Ground level
    {
        m_CurrentPlayerBody.position.y = 650.0f - m_PlayerSize.y;
        // Set grounded if moving downward or already grounded
        if (m_CurrentPlayerBody.velocity.y >= 0) {
            m_IsGrounded = true;
            std::cerr << "[DEBUG] Landed on floor - Grounded: " << m_IsGrounded << std::endl;
        }
        // Apply velocity clipping to prevent sinking into floor
        if (m_CurrentPlayerBody.velocity.y > 0) {
            float velocityProjection = m_CurrentPlayerBody.velocity.y;
            m_CurrentPlayerBody.velocity.y -= velocityProjection;  // Stop downward motion
        }
    }

    // Store position before platform collisions for fallback check
    Nyon::Math::Vector2 positionBeforePlatforms = m_CurrentPlayerBody.position;

    // Check collisions with all platforms
    CheckPlatformCollisions();

    // More selective fallback: only trigger when we're clearly above a platform and falling
    if (!m_IsGrounded && m_CurrentPlayerBody.velocity.y > 10.0f) { // Even higher velocity threshold
        for (const auto& platform : m_Platforms) {
            float playerBottom = m_CurrentPlayerBody.position.y + m_PlayerSize.y;
            float platformTop = platform.body.position.y;
            float playerLeft = m_CurrentPlayerBody.position.x;
            float playerRight = m_CurrentPlayerBody.position.x + m_PlayerSize.x;
            float platformLeft = platform.body.position.x;
            float platformRight = platform.body.position.x + platform.size.x;
            
            // Check if player has significant horizontal overlap with platform AND is close vertically
            bool horizontalOverlap = (playerRight > platformLeft + 5.0f && playerLeft < platformRight - 5.0f);
            bool verticallyClose = (playerBottom >= platformTop - 5.0f && playerBottom <= platformTop + 10.0f);
            
            if (horizontalOverlap && verticallyClose && m_CurrentPlayerBody.velocity.y > 10.0f) {
                m_CurrentPlayerBody.position.y = platformTop - m_PlayerSize.y;
                m_CurrentPlayerBody.velocity.y = 0;
                m_IsGrounded = true;
                std::cerr << "[DEBUG] Selective fallback landing on platform - Grounded: " << m_IsGrounded << std::endl;
                break;
            }
        }
    }

    // Check if we should lose grounded state (walking off edges)
    if (m_IsGrounded && wasGrounded && m_CurrentPlayerBody.velocity.y >= 0) {
        bool stillOnPlatform = false;
        for (const auto& platform : m_Platforms) {
            float playerBottom = m_CurrentPlayerBody.position.y + m_PlayerSize.y;
            float platformTop = platform.body.position.y;
            float playerLeft = m_CurrentPlayerBody.position.x;
            float playerRight = m_CurrentPlayerBody.position.x + m_PlayerSize.x;
            float platformLeft = platform.body.position.x;
            float platformRight = platform.body.position.x + platform.size.x;
            
            // Check if player has reasonable horizontal overlap with platform
            bool horizontalOverlap = (playerRight > platformLeft - 3.0f && playerLeft < platformRight + 3.0f);
            bool verticallyClose = (std::abs(playerBottom - platformTop) < 5.0f);
            
            if (horizontalOverlap && verticallyClose) {
                stillOnPlatform = true;
                break;
            }
        }
        
        // If not on any platform and we were grounded, lose grounded state
        if (!stillOnPlatform) {
            m_IsGrounded = false;
            std::cerr << "[DEBUG] Walking off platform edge - Grounded: " << m_IsGrounded << std::endl;
        }
    }

    // Store current position for next frame collision detection
    m_PreviousPlayerPosition = m_CurrentPlayerBody.position;

    std::cerr << "[DEBUG] After physics - Player position: (" << m_CurrentPlayerBody.position.x << ", " << m_CurrentPlayerBody.position.y << ")" << std::endl;
    std::cerr << "[DEBUG]IsGrounded: " << m_IsGrounded << std::endl;
    std::cerr << "[DEBUG] Velocity: (" << m_CurrentPlayerBody.velocity.x << ", " << m_CurrentPlayerBody.velocity.y << ")" << std::endl;
    std::cerr << "[DEBUG]GameApplication::UpdatePhysics() completed" << std::endl;
}

void GameApplication::CheckPlatformCollisions()
{
    bool platformCollisionFound = false;
    
    for (auto& platform : m_Platforms) {
        // Broad-phase collision check first to avoid expensive SAT when objects are far apart
        if (Nyon::Utils::CollisionPhysics::CheckAABBCollision(m_CurrentPlayerBody.position, m_PlayerSize,
                    platform.body.position, platform.size))
        {
            // Use Continuous Collision Detection for fast-moving objects
            auto ccdResult = Nyon::Utils::CollisionPhysics::ContinuousCollisionCheckMovingVsStatic(
                    m_PlayerShape, m_PreviousPlayerPosition, m_CurrentPlayerBody.position,
                    platform.shape, platform.body.position);

            if (ccdResult.collided) {
                std::cerr << "[DEBUG] CCD collision detected with platform! Time of impact: " << ccdResult.timeOfImpact << std::endl;
                
                // Move player to safe position at time of impact
                m_CurrentPlayerBody.position = ccdResult.impactPosition;
                
                // Resolve collision using MTV from CCD result
                const auto& satResult = ccdResult.collision;
                
                // Calculate relative velocity to determine collision direction
                float relVelY = m_CurrentPlayerBody.velocity.y;
                bool isFalling = relVelY > 0.5f; // Low threshold for falling
                bool isRising = relVelY < -0.5f; // Low threshold for rising

                // Simpler approach: if we're falling and there's significant downward overlap, it's a top collision
                bool isTopCollision = isFalling && (satResult.overlapAxis.y < -0.3f);

                std::cerr << "[DEBUG] Platform Collision analysis:" << std::endl;
                std::cerr << "[DEBUG]   Is falling: " << isFalling << " (velocity.y = " << relVelY << ")" << std::endl;
                std::cerr << "[DEBUG]   Is rising: " << isRising << std::endl;
                std::cerr << "[DEBUG]   Overlap axis Y: " << satResult.overlapAxis.y << std::endl;
                std::cerr << "[DEBUG]   Is top collision: " << isTopCollision << std::endl;

                // For top collision, we want the MTV to push the player UP (negative Y)
                // If the MTV is pointing down, flip it for top collisions
                Nyon::Math::Vector2 correctedAxis = satResult.overlapAxis;
                if (isTopCollision && correctedAxis.y > 0) {
                    correctedAxis.y *= -1.0f;
                    std::cerr << "[DEBUG] Flipping MTV for top collision" << std::endl;
                }

                if (isTopCollision && correctedAxis.y < 0) {
                    // Land on top of platform - resolve using MTV
                    float slop = 0.01f;
                    m_CurrentPlayerBody.position.y -= (satResult.overlapAmount + slop) * correctedAxis.y;

                    // Apply velocity clipping
                    float velocityProjection = correctedAxis.x * m_CurrentPlayerBody.velocity.x +
                        correctedAxis.y * m_CurrentPlayerBody.velocity.y;
                    if (velocityProjection < 0) {
                        m_CurrentPlayerBody.velocity.x -= correctedAxis.x * velocityProjection;
                        m_CurrentPlayerBody.velocity.y -= correctedAxis.y * velocityProjection;
                    }

                    m_IsGrounded = true;
                    platformCollisionFound = true;
                    std::cerr << "[DEBUG] Landing on platform from top - Grounded: " << m_IsGrounded << std::endl;
                    break; // Exit loop since we found a collision
                } else {
                    // Other collision types - don't set grounded
                    // Only apply minimal correction to prevent sticking
                    float slop = 0.01f;
                    m_CurrentPlayerBody.position.x -= (satResult.overlapAmount * 0.1f + slop) * correctedAxis.x;
                    m_CurrentPlayerBody.position.y -= (satResult.overlapAmount * 0.1f + slop) * correctedAxis.y;

                    std::cerr << "[DEBUG] Side/bottom collision with platform - minimal correction" << std::endl;
                }
            }
        }
    }
    
    // If no platform collision was found, check discrete SAT as fallback
    if (!platformCollisionFound) {
        for (const auto& platform : m_Platforms) {
            if (Nyon::Utils::CollisionPhysics::CheckAABBCollision(m_CurrentPlayerBody.position, m_PlayerSize,
                        platform.body.position, platform.size))
            {
                auto satResult = Nyon::Utils::CollisionPhysics::CheckPolygonCollision(
                        m_PlayerShape, m_CurrentPlayerBody.position,
                        platform.shape, platform.body.position);
                
                if (satResult.collided) {
                    std::cerr << "[DEBUG] Discrete SAT collision detected with platform!" << std::endl;
                    
                    float relVelY = m_CurrentPlayerBody.velocity.y;
                    bool isFalling = relVelY > 0.5f; // Low threshold for falling
                    bool isRising = relVelY < -0.5f; // Low threshold for rising

                    // Simpler approach: if we're falling and there's significant downward overlap, it's a top collision
                    bool isTopCollision = isFalling && (satResult.overlapAxis.y < -0.3f);

                    std::cerr << "[DEBUG] Discrete SAT analysis:" << std::endl;
                    std::cerr << "[DEBUG]   Is falling: " << isFalling << " (velocity.y = " << relVelY << ")" << std::endl;
                    std::cerr << "[DEBUG]   Is rising: " << isRising << std::endl;
                    std::cerr << "[DEBUG]   Overlap axis Y: " << satResult.overlapAxis.y << std::endl;
                    std::cerr << "[DEBUG]   Is top collision: " << isTopCollision << std::endl;

                    Nyon::Math::Vector2 correctedAxis = satResult.overlapAxis;
                    if (isTopCollision && correctedAxis.y > 0) {
                        correctedAxis.y *= -1.0f;
                        std::cerr << "[DEBUG] Flipping MTV for top collision (discrete)" << std::endl;
                    }

                    if (isTopCollision && correctedAxis.y < 0) {
                        float slop = 0.01f;
                        m_CurrentPlayerBody.position.y -= (satResult.overlapAmount + slop) * correctedAxis.y;

                        float velocityProjection = correctedAxis.x * m_CurrentPlayerBody.velocity.x +
                            correctedAxis.y * m_CurrentPlayerBody.velocity.y;
                        if (velocityProjection < 0) {
                            m_CurrentPlayerBody.velocity.x -= correctedAxis.x * velocityProjection;
                            m_CurrentPlayerBody.velocity.y -= correctedAxis.y * velocityProjection;
                        }

                        m_IsGrounded = true;
                        std::cerr << "[DEBUG] Landing on platform from top (discrete) - Grounded: " << m_IsGrounded << std::endl;
                        break;
                    } else {
                        // Side or bottom collision - don't set grounded
                        // Only apply minimal correction to prevent sticking
                        float slop = 0.01f;
                        m_CurrentPlayerBody.position.x -= (satResult.overlapAmount * 0.1f + slop) * correctedAxis.x;
                        m_CurrentPlayerBody.position.y -= (satResult.overlapAmount * 0.1f + slop) * correctedAxis.y;

                        std::cerr << "[DEBUG] Side/bottom collision with platform (discrete) - minimal correction" << std::endl;
                    }
                }
            }
        }
    }
}
