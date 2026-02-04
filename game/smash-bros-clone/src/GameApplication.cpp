#include"GameApplication.h"
#include "nyon/graphics/Renderer2D.h"
#include <iostream>
#include <cmath>

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

    // Initialize player - starting position above the platform
    m_CurrentPlayerBody.position = Nyon::Math::Vector2(500.0f, 400.0f);
    m_CurrentPlayerBody.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
    m_CurrentPlayerBody.acceleration = Nyon::Math::Vector2(0.0f, 0.0f);
    m_CurrentPlayerBody.mass = 1.0f;
    m_CurrentPlayerBody.isStatic = false;

    // Set previous state equal to current state initiallym_PreviousPlayerBody = m_CurrentPlayerBody;
    m_PreviousPlayerPosition = m_CurrentPlayerBody.position;

    // Define player as a square polygon for SAT collision
    m_PlayerShape = {
        Nyon::Math::Vector2(0.0f, 0.0f),// bottom-left
        Nyon::Math::Vector2(m_PlayerSize.x, 0.0f), //bottom-right
        Nyon::Math::Vector2(m_PlayerSize.x, m_PlayerSize.y), // top-right
        Nyon::Math::Vector2(0.0f, m_PlayerSize.y)  // top-left
    };

    m_PlayerSize = Nyon::Math::Vector2(32.0f, 32.0f);
    m_PlayerColor = Nyon::Math::Vector3(0.0f, 0.8f, 1.0f);// Cyan
    m_IsGrounded = false;

    // Initialize platform
    m_CurrentPlatformBody.position= Nyon::Math::Vector2(400.0f, 500.0f);
    m_CurrentPlatformBody.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
    m_CurrentPlatformBody.acceleration = Nyon::Math::Vector2(0.0f, 0.0f);
    m_CurrentPlatformBody.mass = 1.0f;
    m_CurrentPlatformBody.isStatic = true; // Platform doesn'tmove

    // Set previous state equal to current state initially
    m_PreviousPlatformBody = m_CurrentPlatformBody;

    // Define platform as a rectangle polygon for SAT collision
    m_PlatformShape = {
        Nyon::Math::Vector2(0.0f, 0.0f),// bottom-leftNyon::Math::Vector2(m_PlatformSize.x, 0.0f),          // bottom-rightNyon::Math::Vector2(m_PlatformSize.x, m_PlatformSize.y), // top-right
        Nyon::Math::Vector2(0.0f, m_PlatformSize.y)           // top-left
    };

    m_PlatformSize = Nyon::Math::Vector2(400.0f, 32.0f);
    m_PlatformColor = Nyon::Math::Vector3(0.6f, 0.4f,0.2f); // Brown

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

    // Copy current state to previousstate before physics update
    m_PreviousPlayerBody = m_CurrentPlayerBody;
    m_PreviousPlatformBody = m_CurrentPlatformBody;

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

    // Draw platform with interpolation
    Nyon::Math::Vector2 interpolatedPlatformPos = interpolatePosition(m_PreviousPlatformBody, m_CurrentPlatformBody, alpha);
    Nyon::Graphics::Renderer2D::DrawQuad(
            interpolatedPlatformPos,
            m_PlatformSize,
            Nyon::Math::Vector2(0.0f, 0.0f),
            m_PlatformColor
            );

    // Draw player withinterpolation
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

    //Store previous grounded state to manage persistence
    bool wasGrounded = m_IsGrounded;

    // Apply physics using thecorrected UpdateBody method which handles gravity internally
    // Pass grounded state to prevent gravity accumulation when on surface
    Nyon::Utils::Physics::UpdateBody(m_CurrentPlayerBody, deltaTime,m_IsGrounded);

    // Boundary checks to prevent going off-screen
    if (m_CurrentPlayerBody.position.x < 0) {
        m_CurrentPlayerBody.position.x = 0;
    }
    if (m_CurrentPlayerBody.position.x + m_PlayerSize.x > 1280) {// Assuming window width is 1280
        m_CurrentPlayerBody.position.x = 1280 - m_PlayerSize.x;
    }

    // Check for ground collision (floor)
    if (m_CurrentPlayerBody.position.y + m_PlayerSize.y >= 650.0f)// Ground level
    {
        m_CurrentPlayerBody.position.y = 650.0f - m_PlayerSize.y;
        //Only set grounded if moving downward
        if (m_CurrentPlayerBody.velocity.y >= 0) {
            m_IsGrounded = true;
        }
        // Apply velocity clipping to prevent sinking into floor
        if (m_CurrentPlayerBody.velocity.y > 0) {
            float velocityProjection = m_CurrentPlayerBody.velocity.y;
            m_CurrentPlayerBody.velocity.y -= velocityProjection;  // Stop downward motion
        }
    }

    // Broad-phase collision check first to avoid expensive SAT when objects are far apart
    if (Nyon::Utils::Physics::CheckAABBCollision(m_CurrentPlayerBody.position, m_PlayerSize,
                m_CurrentPlatformBody.position, m_PlatformSize))
    {
        // Only run SAT collision detection if the broad-phase check passes
        auto collisionResult = Nyon::Utils::Physics::CheckPolygonCollision(
                m_PlayerShape, m_CurrentPlayerBody.position,
                m_PlatformShape, m_CurrentPlatformBody.position);

        if (collisionResult.collided)
        {
            std::cerr << "[DEBUG] Platform collision detected with SAT!" << std::endl;

            // Calculate relative velocity to determine collision direction
            float relVelY = m_CurrentPlayerBody.velocity.y;
            bool isFalling = relVelY > 0;
            bool isRising = relVelY <0;

            // Determine if the collision is from above (landing) or below (hitting underside)
            // We only want to land if we're falling onto the platform from above
            bool wasAbove = m_PreviousPlayerPosition.y + m_PlayerSize.y <= m_CurrentPlatformBody.position.y + 0.1f;
            bool isTopCollision = isFalling && wasAbove;

            if (isTopCollision && collisionResult.overlapAxis.y < 0) {
                // Land on top of platform - resolve using MTV with contactskin
                // Add small slop to prevent immediate re-collision
                float slop = 0.01f;
                m_CurrentPlayerBody.position.y -= (collisionResult.overlapAmount + slop) * collisionResult.overlapAxis.y;

                // Apply velocity clipping instead of zeroing
                float velocityProjection = collisionResult.overlapAxis.x * m_CurrentPlayerBody.velocity.x +
                    collisionResult.overlapAxis.y * m_CurrentPlayerBody.velocity.y;
                if (velocityProjection < 0) {
                    // Remove only the component of velocity pointing into the collision
                    m_CurrentPlayerBody.velocity.x -=collisionResult.overlapAxis.x * velocityProjection;
                    m_CurrentPlayerBody.velocity.y -= collisionResult.overlapAxis.y * velocityProjection;
                }

                m_IsGrounded = true;
                std::cerr << "[DEBUG] Landing on platform from top" << std::endl;
            } else {
                // Othercollision types - apply MTV with contact skin and velocity clipping
                float slop = 0.01f;
                m_CurrentPlayerBody.position.x -= (collisionResult.overlapAmount + slop) * collisionResult.overlapAxis.x;
                m_CurrentPlayerBody.position.y -= (collisionResult.overlapAmount +slop) * collisionResult.overlapAxis.y;

                // Apply velocity clipping
                float velocityProjection = collisionResult.overlapAxis.x * m_CurrentPlayerBody.velocity.x +
                    collisionResult.overlapAxis.y * m_CurrentPlayerBody.velocity.y;
                if (velocityProjection < 0) {
                    //Remove only the component of velocity pointing into the collision
                    m_CurrentPlayerBody.velocity.x -= collisionResult.overlapAxis.x * velocityProjection;
                    m_CurrentPlayerBody.velocity.y -= collisionResult.overlapAxis.y * velocityProjection;
                }

                //Don't set grounded for non-top collisions
                if (collisionResult.overlapAxis.y < 0) {
                    // Collision from below - still grounded if we were already
                    m_IsGrounded = wasGrounded;
                } else if (std::abs(collisionResult.overlapAxis.y) < std::abs(collisionResult.overlapAxis.x)) {
                    // Horizontal collision - preserve grounded state
                    m_IsGrounded = wasGrounded;
                }

                std::cerr << "[DEBUG] Other collision type" << std::endl;
            }
        }
    }

    // Check if no longer grounded - only reset if we'removing upward with sufficient velocity
    if (m_IsGrounded && m_CurrentPlayerBody.velocity.y < 0 && std::abs(m_CurrentPlayerBody.velocity.y) > 10.0f) {
        // Player is jumping or being pushed upward with significant force
        m_IsGrounded = false;
    }

    // Store current position for nextframe collision detection
    m_PreviousPlayerPosition = m_CurrentPlayerBody.position;

    std::cerr << "[DEBUG] After physics - Player position: (" << m_CurrentPlayerBody.position.x << ", " << m_CurrentPlayerBody.position.y << ")" << std::endl;
    std::cerr << "[DEBUG]IsGrounded: " << m_IsGrounded << std::endl;
    std::cerr << "[DEBUG] Velocity: (" << m_CurrentPlayerBody.velocity.x << ", " << m_CurrentPlayerBody.velocity.y << ")" << std::endl;
    std::cerr << "[DEBUG]GameApplication::UpdatePhysics() completed" << std::endl;
}
