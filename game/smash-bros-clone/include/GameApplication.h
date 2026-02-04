#pragma once

#include "nyon/core/Application.h"
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include "nyon/utils/InputManager.h"
#include "nyon/utils/Physics.h"
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
    
    // Player properties - need both current and previous states for interpolation
    Nyon::Utils::Physics::Body m_CurrentPlayerBody;
    Nyon::Utils::Physics::Body m_PreviousPlayerBody;
    Nyon::Utils::Physics::Polygon m_PlayerShape;  // Polygon representation for SAT collision
    Nyon::Math::Vector2 m_PlayerSize;
    Nyon::Math::Vector3 m_PlayerColor;
    bool m_IsGrounded;
    
    // Platform properties
    Nyon::Utils::Physics::Body m_CurrentPlatformBody;
    Nyon::Utils::Physics::Body m_PreviousPlatformBody;
    Nyon::Utils::Physics::Polygon m_PlatformShape;  // Polygon representation for SAT collision
    Nyon::Math::Vector2 m_PlatformSize;
    Nyon::Math::Vector3 m_PlatformColor;
    
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