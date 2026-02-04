#include "GameApplication.h"
#include "nyon/graphics/Renderer2D.h"
#include <iostream>

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
    
    // Verify the window is valid
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
    
    // Set previous state equal to current state initially
    m_PreviousPlayerBody = m_CurrentPlayerBody;
    
    // Define player as a square polygon for SAT collision
    m_PlayerShape = {
        Nyon::Math::Vector2(0.0f, 0.0f),           // bottom-left
        Nyon::Math::Vector2(m_PlayerSize.x, 0.0f), // bottom-right
        Nyon::Math::Vector2(m_PlayerSize.x, m_PlayerSize.y), // top-right
        Nyon::Math::Vector2(0.0f, m_PlayerSize.y)  // top-left
    };
    
    m_PlayerSize = Nyon::Math::Vector2(32.0f, 32.0f);
    m_PlayerColor = Nyon::Math::Vector3(0.0f, 0.8f, 1.0f); // Cyan
    m_IsGrounded = false;
    
    // Initialize platform
    m_CurrentPlatformBody.position = Nyon::Math::Vector2(400.0f, 500.0f);
    m_CurrentPlatformBody.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
    m_CurrentPlatformBody.acceleration = Nyon::Math::Vector2(0.0f, 0.0f);
    m_CurrentPlatformBody.mass = 1.0f;
    m_CurrentPlatformBody.isStatic = true; // Platform doesn't move
    
    // Set previous state equal to current state initially
    m_PreviousPlatformBody = m_CurrentPlatformBody;
    
    // Define platform as a rectangle polygon for SAT collision
    m_PlatformShape = {
        Nyon::Math::Vector2(0.0f, 0.0f),                      // bottom-left
        Nyon::Math::Vector2(m_PlatformSize.x, 0.0f),          // bottom-right
        Nyon::Math::Vector2(m_PlatformSize.x, m_PlatformSize.y), // top-right
        Nyon::Math::Vector2(0.0f, m_PlatformSize.y)           // top-left
    };
    
    m_PlatformSize = Nyon::Math::Vector2(400.0f, 32.0f);
    m_PlatformColor = Nyon::Math::Vector3(0.6f, 0.4f, 0.2f); // Brown
    
    std::cerr << "[DEBUG] GameApplication::OnStart() completed" << std::endl;
}

void GameApplication::OnUpdate(float deltaTime)
{
    // This is for backward compatibility if someone calls it
    std::cerr << "[DEBUG] GameApplication::OnUpdate() called with delta time: " << deltaTime << std::endl;
    HandleInput(deltaTime);
    UpdatePhysics(deltaTime);
}

void GameApplication::OnFixedUpdate(float deltaTime)
{
    std::cerr << "[DEBUG] GameApplication::OnFixedUpdate() called with fixed delta time: " << deltaTime << std::endl;
    
    // Copy current state to previous state before physics update
    m_PreviousPlayerBody = m_CurrentPlayerBody;
    m_PreviousPlatformBody = m_CurrentPlatformBody;
    
    // Update input
    std::cerr << "[DEBUG] Calling InputManager::Update()" << std::endl;
    Nyon::Utils::InputManager::Update();
    
    // Handle player input
    std::cerr << "[DEBUG] Calling HandleInput()" << std::endl;
    HandleInput(deltaTime);
    
    // Update physics
    std::cerr << "[DEBUG] Calling UpdatePhysics()" << std::endl;
    UpdatePhysics(deltaTime);
    
    std::cerr << "[DEBUG] GameApplication::OnFixedUpdate() completed" << std::endl;
}

void GameApplication::OnInterpolateAndRender(float alpha)
{
    std::cerr << "[DEBUG] GameApplication::OnInterpolateAndRender() called with alpha: " << alpha << std::endl;
    
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
    
    std::cerr << "[DEBUG] GameApplication::OnInterpolateAndRender() completed" << std::endl;
}

void GameApplication::OnRender()
{
    // This method is kept for backward compatibility, but the interpolation render method is used instead
    std::cerr << "[DEBUG] GameApplication::OnRender() called (compatibility)" << std::endl;
}

Nyon::Math::Vector2 GameApplication::interpolatePosition(const Nyon::Utils::Physics::Body& prev, 
                                                        const Nyon::Utils::Physics::Body& curr, 
                                                        float alpha) const
{
    // Linear interpolation between previous and current position
    return Nyon::Math::Vector2(
        prev.position.x * (1.0f - alpha) + curr.position.x * alpha,
        prev.position.y * (1.0f - alpha) + curr.position.y * alpha
    );
}

void GameApplication::HandleInput(float deltaTime)
{
    std::cerr << "[DEBUG] GameApplication::HandleInput() called" << std::endl;
    
    // Move based on PLAYER_SPEED when keys are pressed
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
        // Apply jump impulse - reset vertical velocity to jump force
        m_CurrentPlayerBody.velocity.y = JUMP_FORCE;
        m_IsGrounded = false;
    }
    
    std::cerr << "[DEBUG] Player velocity: (" << m_CurrentPlayerBody.velocity.x << ", " << m_CurrentPlayerBody.velocity.y << ")" << std::endl;
    std::cerr << "[DEBUG] Player position: (" << m_CurrentPlayerBody.position.x << ", " << m_CurrentPlayerBody.position.y << ")" << std::endl;
    
    std::cerr << "[DEBUG] GameApplication::HandleInput() completed" << std::endl;
}

void GameApplication::UpdatePhysics(float deltaTime)
{
    std::cerr << "[DEBUG] GameApplication::UpdatePhysics() called" << std::endl;
    std::cerr << "[DEBUG] Before physics - Player position: (" << m_CurrentPlayerBody.position.x << ", " << m_CurrentPlayerBody.position.y << ")" << std::endl;
    
    // Apply physics using the corrected UpdateBody method which handles gravity internally
    // This method properly applies gravity to velocity without accumulating acceleration
    Nyon::Utils::Physics::UpdateBody(m_CurrentPlayerBody, deltaTime);
    
    // Boundary checks to prevent going off-screen
    if (m_CurrentPlayerBody.position.x < 0) {
        m_CurrentPlayerBody.position.x = 0;
    }
    if (m_CurrentPlayerBody.position.x + m_PlayerSize.x > 1280) { // Assuming window width is 1280
        m_CurrentPlayerBody.position.x = 1280 - m_PlayerSize.x;
    }
    
    // Check for ground collision (floor)
    if (m_CurrentPlayerBody.position.y + m_PlayerSize.y >= 650.0f) // Ground level
    {
        m_CurrentPlayerBody.position.y = 650.0f - m_PlayerSize.y;
        m_CurrentPlayerBody.velocity.y = 0.0f;
        m_IsGrounded = true;
    }
    else
    {
        // Check if player is not grounded based on position relative to platform
        // Only set to not grounded if not colliding with platform and falling
        if (m_CurrentPlayerBody.velocity.y > 0) { // Falling
            m_IsGrounded = false;
        }
    }
    
    // Check for platform collision using SAT collision detection
    if (Nyon::Utils::Physics::CheckPolygonCollision(
            m_PlayerShape, m_CurrentPlayerBody.position,
            m_PlatformShape, m_CurrentPlatformBody.position))
    {
        std::cerr << "[DEBUG] Platform collision detected with SAT!" << std::endl;
        
        // Calculate overlap in both directions to determine best response
        float overlapTop = (m_CurrentPlayerBody.position.y + m_PlayerSize.y) - m_CurrentPlatformBody.position.y;
        float overlapBottom = (m_CurrentPlatformBody.position.y + m_PlatformSize.y) - m_CurrentPlayerBody.position.y;
        float overlapLeft = (m_CurrentPlayerBody.position.x + m_PlayerSize.x) - m_CurrentPlatformBody.position.x;
        float overlapRight = (m_CurrentPlatformBody.position.x + m_PlatformSize.x) - m_CurrentPlayerBody.position.x;
        
        // Find smallest overlap to resolve collision properly
        float minOverlapX = std::min(overlapLeft, overlapRight);
        float minOverlapY = std::min(overlapTop, overlapBottom);
        
        // Use a small epsilon to avoid precision issues
        const float epsilon = 0.1f;
        
        if (minOverlapY < minOverlapX) {
            // Vertical collision is smaller - resolve vertically
            if (overlapTop < overlapBottom) {
                // Collision from top - player landed on platform
                m_CurrentPlayerBody.position.y = m_CurrentPlatformBody.position.y - m_PlayerSize.y;
                m_CurrentPlayerBody.velocity.y = 0.0f;
                m_IsGrounded = true;
                std::cerr << "[DEBUG] Landing on platform from top" << std::endl;
            } else {
                // Collision from bottom - player hit platform from below
                m_CurrentPlayerBody.position.y = m_CurrentPlatformBody.position.y + m_PlatformSize.y;
                m_CurrentPlayerBody.velocity.y = 0.0f;
                std::cerr << "[DEBUG] Hitting platform from bottom" << std::endl;
            }
        } else {
            // Horizontal collision is smaller - resolve horizontally
            if (overlapLeft < overlapRight) {
                // Collision from right - player came from right
                m_CurrentPlayerBody.position.x = m_CurrentPlatformBody.position.x - m_PlayerSize.x;
                m_CurrentPlayerBody.velocity.x = 0.0f;
                std::cerr << "[DEBUG] Hitting platform from right" << std::endl;
            } else {
                // Collision from left - player came from left
                m_CurrentPlayerBody.position.x = m_CurrentPlatformBody.position.x + m_PlatformSize.x;
                m_CurrentPlayerBody.velocity.x = 0.0f;
                std::cerr << "[DEBUG] Hitting platform from left" << std::endl;
            }
        }
    }
    
    std::cerr << "[DEBUG] After physics - Player position: (" << m_CurrentPlayerBody.position.x << ", " << m_CurrentPlayerBody.position.y << ")" << std::endl;
    std::cerr << "[DEBUG] GameApplication::UpdatePhysics() completed" << std::endl;
}