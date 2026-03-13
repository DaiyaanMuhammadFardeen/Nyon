#pragma once

#include "nyon/core/ECSApplication.h"

class SimplePhysicsDemo : public Nyon::ECSApplication
{
public:
    SimplePhysicsDemo();
    
protected:
    void OnECSStart() override;
    void OnECSFixedUpdate(float deltaTime) override;
    
private:
    void CreatePlatform();
    void CreateFallingBox();
    
    bool m_CollisionDetected = false;  // Track if collision has occurred
};
