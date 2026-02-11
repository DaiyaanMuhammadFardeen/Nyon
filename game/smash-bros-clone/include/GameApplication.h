#pragma once

#include "nyon/core/Application.h"
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include "nyon/utils/InputManager.h"
#include "nyon/utils/GravityPhysics.h"
#include "nyon/utils/CollisionPhysics.h"
#include <vector>

class GameApplication : public Nyon::Application
{
public:
    GameApplication();

protected:
    virtual void OnStart() override;
    virtual void OnUpdate(float deltaTime) override;  // For backward compatibility
    virtual void OnFixedUpdate(float deltaTime) override;
    virtual void OnRender() override;  // For backward compatibility
    virtual void OnInterpolateAndRender(float alpha) override;

private:
    void HandleInput(float deltaTime);
    void UpdatePhysics(float deltaTime);
    void CheckPlatformCollisions();
    
    // Player properties - need both current and previous states for interpolation
    Nyon::Utils::Physics::Body m_CurrentPlayerBody;
    Nyon::Utils::Physics::Body m_PreviousPlayerBody;
    Nyon::Utils::CollisionPhysics::Polygon m_PlayerShape;  // Polygon representation for SAT collision
    Nyon::Math::Vector2 m_PlayerSize;
    Nyon::Math::Vector3 m_PlayerColor;
    bool m_IsGrounded;
    
    // Platform and obstacle structures
    struct Platform {
        Nyon::Utils::Physics::Body body;
        Nyon::Utils::Physics::Body previousBody;
        Nyon::Utils::CollisionPhysics::Polygon shape;
        Nyon::Math::Vector2 size;
        Nyon::Math::Vector3 color;
    };
    
    std::vector<Platform> m_Platforms;
    
    // Previous frame position for collision response
    Nyon::Math::Vector2 m_PreviousPlayerPosition;
    
    // Game constants
    static constexpr float PLAYER_SPEED = 300.0f;
    static constexpr float JUMP_FORCE = -400.0f;
    
    // Interpolation helper
    Nyon::Math::Vector2 interpolatePosition(const Nyon::Utils::Physics::Body& prev, 
                                           const Nyon::Utils::Physics::Body& curr, 
                                           float alpha) const;
};