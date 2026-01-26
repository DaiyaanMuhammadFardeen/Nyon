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
    m_PlayerBody.position = Nyon::Math::Vector2(500.0f, 400.0f);
    m_PlayerBody.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
    m_PlayerBody.acceleration = Nyon::Math::Vector2(0.0f, 0.0f);
    m_PlayerBody.mass = 1.0f;
    m_PlayerBody.isStatic = false;
    
    m_PlayerSize = Nyon::Math::Vector2(32.0f, 32.0f);
    m_PlayerColor = Nyon::Math::Vector3(0.0f, 0.8f, 1.0f); // Cyan
    m_IsGrounded = false;
    
    // Initialize platform
    m_PlatformBody.position = Nyon::Math::Vector2(400.0f, 500.0f);
    m_PlatformBody.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
    m_PlatformBody.acceleration = Nyon::Math::Vector2(0.0f, 0.0f);
    m_PlatformBody.mass = 1.0f;
    m_PlatformBody.isStatic = true; // Platform doesn't move
    
    m_PlatformSize = Nyon::Math::Vector2(400.0f, 32.0f);
    m_PlatformColor = Nyon::Math::Vector3(0.6f, 0.4f, 0.2f); // Brown
    
    std::cerr << "[DEBUG] GameApplication::OnStart() completed" << std::endl;
}

void GameApplication::OnUpdate(float deltaTime)
{
    std::cerr << "[DEBUG] GameApplication::OnUpdate() called with delta time: " << deltaTime << std::endl;
    
    // Update input
    std::cerr << "[DEBUG] Calling InputManager::Update()" << std::endl;
    Nyon::Utils::InputManager::Update();
    
    // Handle player input
    std::cerr << "[DEBUG] Calling HandleInput()" << std::endl;
    HandleInput(deltaTime);
    
    // Update physics
    std::cerr << "[DEBUG] Calling UpdatePhysics()" << std::endl;
    UpdatePhysics(deltaTime);
    
    std::cerr << "[DEBUG] GameApplication::OnUpdate() completed" << std::endl;
}

void GameApplication::HandleInput(float deltaTime)
{
    std::cerr << "[DEBUG] GameApplication::HandleInput() called" << std::endl;
    
    // Move 1 pixel per frame when keys are pressed (ignoring deltaTime for pixel-perfect movement)
    if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_A) || Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_LEFT))
    {
        std::cerr << "[DEBUG] Moving left 1 pixel" << std::endl;
        m_PlayerBody.position.x -= 1.0f;
    }
    else if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_D) || Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_RIGHT))
    {
        std::cerr << "[DEBUG] Moving right 1 pixel" << std::endl;
        m_PlayerBody.position.x += 1.0f;
    }
    
    // Jumping
    if ((Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE) || 
         Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_UP)) && m_IsGrounded)
    {
        std::cerr << "[DEBUG] Jumping" << std::endl;
        m_PlayerBody.velocity.y = JUMP_FORCE;
        m_IsGrounded = false;
    }
    
    std::cerr << "[DEBUG] Player velocity: (" << m_PlayerBody.velocity.x << ", " << m_PlayerBody.velocity.y << ")" << std::endl;
    std::cerr << "[DEBUG] Player position: (" << m_PlayerBody.position.x << ", " << m_PlayerBody.position.y << ")" << std::endl;
    
    std::cerr << "[DEBUG] GameApplication::HandleInput() completed" << std::endl;
}

void GameApplication::UpdatePhysics(float deltaTime)
{
    std::cerr << "[DEBUG] GameApplication::UpdatePhysics() called" << std::endl;
    std::cerr << "[DEBUG] Before physics - Player position: (" << m_PlayerBody.position.x << ", " << m_PlayerBody.position.y << ")" << std::endl;
    
    // Apply gravity to player (only if not using pixel movement for horizontal)
    if (!(Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_A) || Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_LEFT) ||
          Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_D) || Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_RIGHT))) {
        // Only apply gravity when not doing manual pixel movement (for vertical movement)
        Nyon::Utils::Physics::ApplyGravity(m_PlayerBody);
    }
    
    // Update vertical physics with velocity (for gravity and jumping)
    m_PlayerBody.velocity.y += Nyon::Utils::Physics::Gravity * deltaTime;
    m_PlayerBody.position.y += m_PlayerBody.velocity.y * deltaTime;
    
    // Boundary checks to prevent going off-screen
    if (m_PlayerBody.position.x < 0) {
        m_PlayerBody.position.x = 0;
    }
    if (m_PlayerBody.position.x + m_PlayerSize.x > 1280) { // Assuming window width is 1280
        m_PlayerBody.position.x = 1280 - m_PlayerSize.x;
    }
    
    // Check for ground collision (floor)
    if (m_PlayerBody.position.y + m_PlayerSize.y >= 650.0f) // Ground level
    {
        m_PlayerBody.position.y = 650.0f - m_PlayerSize.y;
        m_PlayerBody.velocity.y = 0.0f;
        m_IsGrounded = true;
    }
    else
    {
        m_IsGrounded = false; // Reset grounded if not on ground
    }
    
    // Check for platform collision
    if (Nyon::Utils::Physics::CheckCollision(
            m_PlayerBody, m_PlayerSize,
            m_PlatformBody, m_PlatformSize))
    {
        std::cerr << "[DEBUG] Platform collision detected!" << std::endl;
        
        // Calculate overlap in both directions to determine best response
        float overlapTop = (m_PlayerBody.position.y + m_PlayerSize.y) - m_PlatformBody.position.y;
        float overlapBottom = (m_PlatformBody.position.y + m_PlatformSize.y) - m_PlayerBody.position.y;
        float overlapLeft = (m_PlayerBody.position.x + m_PlayerSize.x) - m_PlatformBody.position.x;
        float overlapRight = (m_PlatformBody.position.x + m_PlatformSize.x) - m_PlayerBody.position.x;
        
        // Find smallest overlap to resolve collision properly
        float minOverlapX = std::min(overlapLeft, overlapRight);
        float minOverlapY = std::min(overlapTop, overlapBottom);
        
        if (minOverlapY < minOverlapX) {
            // Vertical collision is smaller - resolve vertically
            if (overlapTop < overlapBottom) {
                // Collision from top - player landed on platform
                m_PlayerBody.position.y = m_PlatformBody.position.y - m_PlayerSize.y;
                m_PlayerBody.velocity.y = 0.0f;
                m_IsGrounded = true;
                std::cerr << "[DEBUG] Landing on platform from top" << std::endl;
            } else {
                // Collision from bottom - player hit platform from below
                m_PlayerBody.position.y = m_PlatformBody.position.y + m_PlatformSize.y;
                m_PlayerBody.velocity.y = 0.0f;
                std::cerr << "[DEBUG] Hitting platform from bottom" << std::endl;
            }
        } else {
            // Horizontal collision is smaller - resolve horizontally
            if (overlapLeft < overlapRight) {
                // Collision from right - player came from right
                m_PlayerBody.position.x = m_PlatformBody.position.x - m_PlayerSize.x;
                std::cerr << "[DEBUG] Hitting platform from right" << std::endl;
            } else {
                // Collision from left - player came from left
                m_PlayerBody.position.x = m_PlatformBody.position.x + m_PlatformSize.x;
                std::cerr << "[DEBUG] Hitting platform from left" << std::endl;
            }
        }
    }
    
    std::cerr << "[DEBUG] After physics - Player position: (" << m_PlayerBody.position.x << ", " << m_PlayerBody.position.y << ")" << std::endl;
    std::cerr << "[DEBUG] GameApplication::UpdatePhysics() completed" << std::endl;
}

void GameApplication::OnRender()
{
    std::cerr << "[DEBUG] GameApplication::OnRender() called" << std::endl;
    
    // Clear screen
    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    
    // Begin rendering
    Nyon::Graphics::Renderer2D::BeginScene();
    
    // Draw platform
    Nyon::Graphics::Renderer2D::DrawQuad(
        m_PlatformBody.position,
        m_PlatformSize,
        Nyon::Math::Vector2(0.0f, 0.0f),
        m_PlatformColor
    );
    
    // Draw player
    Nyon::Graphics::Renderer2D::DrawQuad(
        m_PlayerBody.position,
        m_PlayerSize,
        Nyon::Math::Vector2(0.0f, 0.0f),
        m_PlayerColor
    );
    
    // End rendering
    Nyon::Graphics::Renderer2D::EndScene();
    
    std::cerr << "[DEBUG] GameApplication::OnRender() completed" << std::endl;
}