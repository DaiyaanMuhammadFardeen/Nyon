#pragma once
#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/EntityManager.h"
class SimplePhysicsDemo : public Nyon::ECSApplication {
public:
    SimplePhysicsDemo();
protected:
    void OnECSStart() override;
    void OnECSFixedUpdate(float deltaTime) override;
private:
    void CreateWorld();
    void CreatePlatform();
    void CreateFallingBox();
    Nyon::ECS::EntityID m_PlatformEntity { 0 };
    Nyon::ECS::EntityID m_BoxEntity      { 0 };
    bool  m_DebugConfigured    { false };
    bool  m_CollisionDetected  { false };
    float m_FirstCollisionTime { 0.0f  };
    float m_SimTime            { 0.0f  };
    static constexpr float DEMO_DURATION_S { 6.0f };
    static constexpr float LOG_INTERVAL_S  { 0.5f  };
    float m_NextLogTime                    { 0.0f  }; };