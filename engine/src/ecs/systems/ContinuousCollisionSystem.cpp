#include "nyon/ecs/systems/ContinuousCollisionSystem.h"
#include "nyon/ecs/components/BehaviorComponent.h"
#include <iostream>

namespace Nyon::ECS {

void ContinuousCollisionSystem::PerformCCD(
    TransformComponent& transformA, TransformComponent& transformB,
    PhysicsBodyComponent& bodyA, PhysicsBodyComponent& bodyB,
    ColliderComponent& colliderA, ColliderComponent& colliderB,
    EntityID entityA, EntityID entityB, float deltaTime)
{
    // Early exit for static-static pairs
    if (bodyA.isStatic && bodyB.isStatic)
        return;
    
    // Predict future positions first for swept broad phase
    Math::Vector2 predictedPosA = transformA.position + bodyA.velocity * deltaTime;
    Math::Vector2 predictedPosB = transformB.position + bodyB.velocity * deltaTime;
    
    // Swept broad phase: Expand AABB by motion path for the frame
    Math::Vector2 minA, maxA, minB, maxB;
    colliderA.CalculateAABB(transformA.position, minA, maxA);
    colliderB.CalculateAABB(transformB.position, minB, maxB);
    
    // Calculate swept AABB endpoints
    Math::Vector2 minA_end, maxA_end, minB_end, maxB_end;
    colliderA.CalculateAABB(predictedPosA, minA_end, maxA_end);
    colliderB.CalculateAABB(predictedPosB, minB_end, maxB_end);
    
    // Expand AABBs by swept path
    Math::Vector2 sweptMinA = {
        std::min(minA.x, minA_end.x),
        std::min(minA.y, minA_end.y)
    };
    Math::Vector2 sweptMaxA = {
        std::max(maxA.x, maxA_end.x),
        std::max(maxA.y, maxA_end.y)
    };
    
    Math::Vector2 sweptMinB = {
        std::min(minB.x, minB_end.x),
        std::min(minB.y, minB_end.y)
    };
    Math::Vector2 sweptMaxB = {
        std::max(maxB.x, maxB_end.x),
        std::max(maxB.y, maxB_end.y)
    };
    
    // Check for swept AABB overlap
    if (sweptMinA.x >= sweptMaxB.x || sweptMaxA.x <= sweptMinB.x || 
        sweptMinA.y >= sweptMaxB.y || sweptMaxA.y <= sweptMinB.y)
    {
        return; // No overlap in swept path, skip CCD
    }
    
    // Use already calculated predicted positions
    
    // Calculate time of impact using swept collision detection
    float toi = 1.0f; // Default to end of frame
    
    // Simple sphere-sphere TOI calculation for now
    Math::Vector2 startPosA = transformA.position;
    Math::Vector2 startPosB = transformB.position;
    Math::Vector2 endPosA = predictedPosA;
    Math::Vector2 endPosB = predictedPosB;
    
    // Calculate actual radii from collider extents
    float radiusA, radiusB;
    if (m_ComponentStore && 
        m_ComponentStore->HasComponent<ColliderComponent>(entityA) &&
        m_ComponentStore->HasComponent<ColliderComponent>(entityB))
    {
        const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityA);
        const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(entityB);
        
        Math::Vector2 minA, maxA, minB, maxB;
        colliderA.CalculateAABB(startPosA, minA, maxA);
        colliderB.CalculateAABB(startPosB, minB, maxB);
        
        Math::Vector2 extentA = (maxA - minA) * 0.5f;
        Math::Vector2 extentB = (maxB - minB) * 0.5f;
        
        // Use minimum extent as sphere radius approximation
        radiusA = std::min(extentA.x, extentA.y);
        radiusB = std::min(extentB.x, extentB.y);
    }
    else
    {
        radiusA = 16.0f;
        radiusB = 16.0f;
    }
    
    toi = Physics::CollisionDetectionStrategy::CalculateTimeOfImpact(
        startPosA, endPosA, startPosB, endPosB, radiusA, radiusB);
    
    // If collision occurs within this timestep
    if (toi >= 0.0f && toi <= 1.0f)
    {
        // Single impulse application followed by iterative positional correction
        // Apply impulse exactly once on first contact
        ResolveCCDImpulse(transformA, transformB, bodyA, bodyB, entityA, entityB);
        
        // Then iterative positional correction without re-applying impulses
        const int ITERATIONS = 3;
        for (int iter = 0; iter < ITERATIONS; ++iter)
        {
            ResolveCCDPositionalCorrection(transformA, transformB, bodyA, bodyB, entityA, entityB);
        }
    }
}

void ContinuousCollisionSystem::ResolveCCDImpulse(
    TransformComponent& transformA, TransformComponent& transformB,
    PhysicsBodyComponent& bodyA, PhysicsBodyComponent& bodyB,
    EntityID entityA, EntityID entityB)
{
    // Calculate collision normal and penetration using current positions
    Math::Vector2 separation = transformA.position - transformB.position;
    float distance = separation.Length();
    
    // Safety check for zero distance
    if (distance < 1e-6f)
    {
        separation = Math::Vector2{1.0f, 0.0f};
        distance = 1.0f;
    }
    
    // Get actual collider dimensions
    Math::Vector2 halfSizeA, halfSizeB;
    if (m_ComponentStore && 
        m_ComponentStore->HasComponent<ColliderComponent>(entityA) &&
        m_ComponentStore->HasComponent<ColliderComponent>(entityB))
    {
        const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityA);
        const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(entityB);
        
        Math::Vector2 minA, maxA, minB, maxB;
        colliderA.CalculateAABB(transformA.position, minA, maxA);
        colliderB.CalculateAABB(transformB.position, minB, maxB);
        
        halfSizeA = (maxA - minA) * 0.5f;
        halfSizeB = (maxB - minB) * 0.5f;
    }
    else
    {
        halfSizeA = {16.0f, 16.0f};
        halfSizeB = {16.0f, 16.0f};
    }
    
    Math::Vector2 delta = transformB.position - transformA.position;
    float overlapX = (halfSizeA.x + halfSizeB.x) - std::abs(delta.x);
    float overlapY = (halfSizeA.y + halfSizeB.y) - std::abs(delta.y);
    
    Math::Vector2 normal;
    if (overlapX < overlapY)
    {
        normal = Math::Vector2(delta.x < 0 ? -1.0f : 1.0f, 0.0f);
    }
    else
    {
        normal = Math::Vector2(0.0f, delta.y < 0 ? -1.0f : 1.0f);
    }
    
    // Calculate proper penetration depth
    float penetrationDepth = (overlapX < overlapY) ? -overlapX : -overlapY;
    
    // Only proceed if there's significant penetration
    if (penetrationDepth < -0.01f)
    {
        // Calculate combined material properties
        float restitution = std::sqrt(bodyA.restitution * bodyB.restitution);
        float friction = std::sqrt(bodyA.friction * bodyB.friction);
        
        // Apply restitution threshold for resting contacts
        const float RESTITUTION_THRESHOLD = 1.0f; // pixels per second
        Math::Vector2 relVel = bodyA.velocity - bodyB.velocity;
        float velAlongNormal = Math::Vector2::Dot(relVel, normal);
        
        if (std::abs(velAlongNormal) < RESTITUTION_THRESHOLD)
        {
            restitution = 0.0f; // No bounce for slow contacts
        }
        
        // Apply impulse if objects are approaching
        if (velAlongNormal < 0)
        {
            float impulseScalar = -(1.0f + restitution) * velAlongNormal;
            impulseScalar /= (bodyA.inverseMass + bodyB.inverseMass);
            
            Math::Vector2 impulse = normal * impulseScalar;
            
            // Apply impulses
            if (!bodyA.isStatic)
            {
                bodyA.velocity = bodyA.velocity + impulse * bodyA.inverseMass;
                // Clamp velocity magnitude
                float velMagnitude = bodyA.velocity.Length();
                const float MAX_VELOCITY = 2000.0f; // Increased from 100
                if (velMagnitude > MAX_VELOCITY)
                {
                    bodyA.velocity = bodyA.velocity * (MAX_VELOCITY / velMagnitude);
                }
            }
            if (!bodyB.isStatic)
            {
                bodyB.velocity = bodyB.velocity - impulse * bodyB.inverseMass;
                // Clamp velocity magnitude
                float velMagnitude = bodyB.velocity.Length();
                const float MAX_VELOCITY = 2000.0f; // Increased from 100
                if (velMagnitude > MAX_VELOCITY)
                {
                    bodyB.velocity = bodyB.velocity * (MAX_VELOCITY / velMagnitude);
                }
            }
            
            // Apply friction
            Math::Vector2 tangent = relVel - normal * velAlongNormal;
            float tangentLength = tangent.Length();
            if (tangentLength > 1e-6f)
            {
                tangent = tangent * (1.0f / tangentLength);
                float frictionImpulse = -Math::Vector2::Dot(relVel, tangent) * friction;
                float maxFriction = std::abs(impulseScalar) * friction;
                frictionImpulse = std::clamp(frictionImpulse, -maxFriction, maxFriction);
                
                Math::Vector2 frictionVector = tangent * frictionImpulse;
                
                if (!bodyA.isStatic)
                {
                    bodyA.velocity = bodyA.velocity + frictionVector * bodyA.inverseMass;
                }
                if (!bodyB.isStatic)
                {
                    bodyB.velocity = bodyB.velocity - frictionVector * bodyB.inverseMass;
                }
            }
            
            // Special ground contact handling
            bool isGroundContact = (normal.y < -0.7f && bodyB.isStatic); // Normal points up, B is static
            if (isGroundContact && bodyA.velocity.y > 0) // Moving downward
            {
                bodyA.velocity.y = 0.0f; // Stop downward motion
            }
        }
    }
    
    // Wake up sleeping bodies
    bodyA.isAwake = true;
    bodyB.isAwake = true;
    bodyA.sleepTimer = 0.0f;
    bodyB.sleepTimer = 0.0f;
}

void ContinuousCollisionSystem::ResolveCCDPositionalCorrection(
    TransformComponent& transformA, TransformComponent& transformB,
    PhysicsBodyComponent& bodyA, PhysicsBodyComponent& bodyB,
    EntityID entityA, EntityID entityB)
{
    // Recalculate contact using current positions
    Math::Vector2 separation = transformA.position - transformB.position;
    float distance = separation.Length();
    
    if (distance < 1e-6f)
    {
        separation = Math::Vector2{1.0f, 0.0f};
        distance = 1.0f;
    }
    
    // Get actual collider dimensions
    Math::Vector2 halfSizeA, halfSizeB;
    if (m_ComponentStore && 
        m_ComponentStore->HasComponent<ColliderComponent>(entityA) &&
        m_ComponentStore->HasComponent<ColliderComponent>(entityB))
    {
        const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityA);
        const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(entityB);
        
        Math::Vector2 minA, maxA, minB, maxB;
        colliderA.CalculateAABB(transformA.position, minA, maxA);
        colliderB.CalculateAABB(transformB.position, minB, maxB);
        
        halfSizeA = (maxA - minA) * 0.5f;
        halfSizeB = (maxB - minB) * 0.5f;
    }
    else
    {
        halfSizeA = {16.0f, 16.0f};
        halfSizeB = {16.0f, 16.0f};
    }
    
    Math::Vector2 delta = transformB.position - transformA.position;
    float overlapX = (halfSizeA.x + halfSizeB.x) - std::abs(delta.x);
    float overlapY = (halfSizeA.y + halfSizeB.y) - std::abs(delta.y);
    
    Math::Vector2 normal;
    if (overlapX < overlapY)
    {
        normal = Math::Vector2(delta.x < 0 ? -1.0f : 1.0f, 0.0f);
    }
    else
    {
        normal = Math::Vector2(0.0f, delta.y < 0 ? -1.0f : 1.0f);
    }
    
    float penetrationDepth = (overlapX < overlapY) ? -overlapX : -overlapY;
    
    // Positional correction with improved Baumgarte
    if (penetrationDepth < -0.01f)
    {
        const float BAUMGARTE_FACTOR = 0.8f; // Increased from 0.6 for faster convergence
        const float ALLOWABLE_SLOP = 0.01f;
        
        if (penetrationDepth < -ALLOWABLE_SLOP)
        {
            Math::Vector2 correction = normal * (std::abs(penetrationDepth) * BAUMGARTE_FACTOR);
            
            float totalInvMass = bodyA.inverseMass + bodyB.inverseMass;
            if (totalInvMass > 0.0f)
            {
                // Limit correction magnitude
                float correctionMagnitude = correction.Length();
                const float MAX_CORRECTION = 50.0f;
                if (correctionMagnitude > MAX_CORRECTION)
                {
                    correction = correction * (MAX_CORRECTION / correctionMagnitude);
                }
                
                if (!bodyA.isStatic)
                {
                    transformA.position = transformA.position + 
                        correction * (bodyA.inverseMass / totalInvMass);
                }
                if (!bodyB.isStatic)
                {
                    transformB.position = transformB.position - 
                        correction * (bodyB.inverseMass / totalInvMass);
                }
            }
        }
    }
}

void ContinuousCollisionSystem::ResolveCCDCollision(
    TransformComponent& transformA, TransformComponent& transformB,
    PhysicsBodyComponent& bodyA, PhysicsBodyComponent& bodyB,
    const Math::Vector2& startPosA, const Math::Vector2& startPosB,
    const Math::Vector2& endPosA, const Math::Vector2& endPosB,
    float timeOfImpact,
    EntityID entityA, EntityID entityB)
{
    // === PHASE A: POSITIONAL CORRECTION (Un-sticking) ===
    // Handle TOI=0 case separately - objects are already overlapping
    if (timeOfImpact == 0.0f)
    {
        // Already overlapping - skip position snap and apply full depenetration
        // Objects are already at their start positions, so we don't move them
        std::cout << "[DEBUG] Handling already-overlapping case for entities " 
                  << entityA << " and " << entityB << std::endl;
    }
    else
    {
        // Move to collision point to prevent tunneling
        transformA.position = startPosA + (endPosA - startPosA) * timeOfImpact;
        transformB.position = startPosB + (endPosB - startPosB) * timeOfImpact;
    }
    
    // Calculate collision normal and penetration
    Math::Vector2 separation = transformA.position - transformB.position;
    float distance = separation.Length();
            
    // Safety check for zero distance
    if (distance < 1e-6f)
    {
        // Objects are at same position - use arbitrary normal
        separation = Math::Vector2{1.0f, 0.0f};
        distance = 1.0f;
    }
            
    // CORRECT AABB COLLISION NORMAL COMPUTATION
    // Following Box2D-style minimum penetration axis determination
    // Get actual collider dimensions instead of hardcoded values
    Math::Vector2 halfSizeA, halfSizeB;
    
    if (m_ComponentStore && 
        m_ComponentStore->HasComponent<ColliderComponent>(entityA) &&
        m_ComponentStore->HasComponent<ColliderComponent>(entityB))
    {
        const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityA);
        const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(entityB);
        
        // Calculate actual half-sizes from colliders
        Math::Vector2 minA, maxA, minB, maxB;
        colliderA.CalculateAABB(transformA.position, minA, maxA);
        colliderB.CalculateAABB(transformB.position, minB, maxB);
        
        halfSizeA = (maxA - minA) * 0.5f;
        halfSizeB = (maxB - minB) * 0.5f;
    }
    else
    {
        // Fallback to reasonable defaults if colliders not available
        halfSizeA = {16.0f, 16.0f};
        halfSizeB = {16.0f, 16.0f};
    }
    Math::Vector2 delta = transformB.position - transformA.position;
    
    // Calculate overlap on each axis
    float overlapX = (halfSizeA.x + halfSizeB.x) - std::abs(delta.x);
    float overlapY = (halfSizeA.y + halfSizeB.y) - std::abs(delta.y);
    
    Math::Vector2 normal;
    if (overlapX < overlapY)
    {
        // X-axis has minimum penetration - collision normal points along X-axis
        // Normal should point FROM A TO B (away from A's surface)
        normal = Math::Vector2(delta.x < 0 ? -1.0f : 1.0f, 0.0f);
    }
    else
    {
        // Y-axis has minimum penetration - collision normal points along Y-axis
        // Normal should point FROM A TO B (away from A's surface)
        normal = Math::Vector2(0.0f, delta.y < 0 ? -1.0f : 1.0f);
    }
    
    // DEBUG: Print normal computation details
    std::cout << "[DEBUG] Normal computation for entities " << entityA << " and " << entityB
              << " | Delta: (" << delta.x << ", " << delta.y << ")"
              << " | Overlap X: " << overlapX << " Y: " << overlapY
              << " | Chosen normal: (" << normal.x << ", " << normal.y << ")" << std::endl;
    
    // Calculate proper penetration depth based on chosen axis
    float penetrationDepth;
    if (overlapX < overlapY)
    {
        // Using X-axis penetration
        penetrationDepth = -overlapX; // Negative = penetration
    }
    else
    {
        // Using Y-axis penetration
        penetrationDepth = -overlapY; // Negative = penetration
    }
            
    // For non-circle shapes, we'd need proper shape-specific calculations
    // This is a simplified version for now
            
    // Safety bounds checking
    if (std::isnan(penetrationDepth) || std::isinf(penetrationDepth))
    {
        std::cout << "[WARNING] Invalid penetration depth: " << penetrationDepth << std::endl;
        return; // Skip this collision resolution
    }
            
    // Limit maximum penetration correction to prevent instability
    const float MAX_PENETRATION_CORRECTION = 50.0f;
    if (penetrationDepth < -MAX_PENETRATION_CORRECTION)
    {
        penetrationDepth = -MAX_PENETRATION_CORRECTION;
    }
    
    // Only proceed if there's significant penetration
    if (penetrationDepth < -0.01f)
    {
        
        // Industry-standard slop tolerance to prevent jitter
        const float ALLOWABLE_SLOP = 0.01f;
        if (penetrationDepth < -ALLOWABLE_SLOP)
        {
            // Positional correction with improved Baumgarte stabilization
            // Increased from 0.2f to 0.6f for better convergence on large penetrations
            const float BAUMGARTE_FACTOR = 0.6f;
            // Correction must follow collision normal exactly
            Math::Vector2 correction = normal * (std::abs(penetrationDepth) * BAUMGARTE_FACTOR);
                            
            // DEBUG: Print positional correction
            std::cout << "[DEBUG] Positional correction for entities " << entityA << " and " << entityB
                      << " | Penetration: " << penetrationDepth
                      << " | Correction: (" << correction.x << ", " << correction.y << ")" << std::endl;
                            
            float totalInvMass = bodyA.inverseMass + bodyB.inverseMass;
            if (totalInvMass > 0.0f)
            {
                // Limit correction magnitude
                float correctionMagnitude = correction.Length();
                const float MAX_CORRECTION = 50.0f;
                if (correctionMagnitude > MAX_CORRECTION)
                {
                    correction = correction * (MAX_CORRECTION / correctionMagnitude);
                }
                                
                if (!bodyA.isStatic)
                {
                    Math::Vector2 oldPosA = transformA.position;
                    transformA.position = transformA.position + 
                        correction * (bodyA.inverseMass / totalInvMass);
                                    
                    // Bounds checking
                    transformA.position.x = std::clamp(transformA.position.x, -1000.0f, 2000.0f);
                    transformA.position.y = std::clamp(transformA.position.y, -1000.0f, 2000.0f);
                                    
                    std::cout << "[DEBUG] BodyA position changed from (" 
                              << oldPosA.x << ", " << oldPosA.y << ") to (" 
                              << transformA.position.x << ", " << transformA.position.y << ")" << std::endl;
                }
                if (!bodyB.isStatic)
                {
                    Math::Vector2 oldPosB = transformB.position;
                    transformB.position = transformB.position - 
                        correction * (bodyB.inverseMass / totalInvMass);
                                    
                    // Bounds checking
                    transformB.position.x = std::clamp(transformB.position.x, -1000.0f, 2000.0f);
                    transformB.position.y = std::clamp(transformB.position.y, -1000.0f, 2000.0f);
                                    
                    std::cout << "[DEBUG] BodyB position changed from (" 
                              << oldPosB.x << ", " << oldPosB.y << ") to (" 
                              << transformB.position.x << ", " << transformB.position.y << ")" << std::endl;
                }
            }
        }
        
        // === PHASE B: IMPULSE RESOLUTION (Bounce) ===
        // Calculate combined material properties (industry standard geometric mean)
        float restitution = std::sqrt(bodyA.restitution * bodyB.restitution);
        float friction = std::sqrt(bodyA.friction * bodyB.friction);
        
        // Relative velocity calculation
        Math::Vector2 relVel = bodyA.velocity - bodyB.velocity;
        float velAlongNormal = Math::Vector2::Dot(relVel, normal);
        
        // Don't resolve if objects are separating (industry standard check)
        // But allow resolution for ground contacts to maintain stability
        // Assuming Y increases downward (common in 2D graphics)
        bool isGroundContact = (normal.y > 0.8f && bodyB.isStatic); // Downward-facing normal against static body (ground contact)
        if (velAlongNormal > 0 && !isGroundContact)
            return;
        
        // === INDUSTRY-STANDARD IMPULSE FORMULA ===
        // j = -(1 + e)(va - vb)·n / (1/ma + 1/mb)
        float impulseScalar = -(1.0f + restitution) * velAlongNormal;
        impulseScalar /= (bodyA.inverseMass + bodyB.inverseMass);
        
        Math::Vector2 impulse = normal * impulseScalar;
        
        // DEBUG: Print collision details
        std::cout << "[DEBUG] Collision between " << entityA << " and " << entityB 
                  << " | Restitution: " << restitution 
                  << " | Vel along normal: " << velAlongNormal
                  << " | Impulse scalar: " << impulseScalar
                  << " | Normal: (" << normal.x << ", " << normal.y << ")"
                  << " | BodyA vel: (" << bodyA.velocity.x << ", " << bodyA.velocity.y << ")"
                  << " | BodyB vel: (" << bodyB.velocity.x << ", " << bodyB.velocity.y << ")"
                  << std::endl;
        
        // Apply impulses to linear velocities with proper constraint handling
        const float MAX_VELOCITY = 100.0f; // Maximum reasonable velocity
        
        if (!bodyA.isStatic)
        {
            Math::Vector2 oldVelA = bodyA.velocity;
            bodyA.velocity = bodyA.velocity + impulse * bodyA.inverseMass;
            
            // Special handling for ground contacts - prevent downward velocity
            // In Y-down coordinate system, positive Y velocity = moving downward
            if (isGroundContact && bodyA.velocity.y > 0)
            {
                bodyA.velocity.y = 0.0f; // Stop downward motion against ground
            }
            
            // Limit velocity magnitude
            float velMagnitude = bodyA.velocity.Length();
            if (velMagnitude > MAX_VELOCITY)
            {
                bodyA.velocity = bodyA.velocity * (MAX_VELOCITY / velMagnitude);
            }
            
            std::cout << "[DEBUG] BodyA velocity changed from (" 
                      << oldVelA.x << ", " << oldVelA.y << ") to (" 
                      << bodyA.velocity.x << ", " << bodyA.velocity.y << ")" << std::endl;
        }
        if (!bodyB.isStatic)
        {
            Math::Vector2 oldVelB = bodyB.velocity;
            bodyB.velocity = bodyB.velocity - impulse * bodyB.inverseMass;
            
            // Special handling for ground contacts - prevent upward velocity on static bodies
            // Static bodies shouldn't move, but this ensures robustness
            if (isGroundContact && bodyB.velocity.y < 0)
            {
                bodyB.velocity.y = 0.0f; // Prevent upward motion of static body
            }
            
            // Limit velocity magnitude
            float velMagnitude = bodyB.velocity.Length();
            if (velMagnitude > MAX_VELOCITY)
            {
                bodyB.velocity = bodyB.velocity * (MAX_VELOCITY / velMagnitude);
            }
            
            std::cout << "[DEBUG] BodyB velocity changed from (" 
                      << oldVelB.x << ", " << oldVelB.y << ") to (" 
                      << bodyB.velocity.x << ", " << bodyB.velocity.y << ")" << std::endl;
        }
        
        // === FRICTION HANDLING (Coulomb friction model) ===
        Math::Vector2 tangent = relVel - normal * velAlongNormal;
        float tangentLength = tangent.Length();
        if (tangentLength > 1e-6f)
        {
            tangent = tangent * (1.0f / tangentLength);
            float frictionImpulse = -Math::Vector2::Dot(relVel, tangent) * friction;
            float maxFriction = std::abs(impulseScalar) * friction;
            frictionImpulse = std::clamp(frictionImpulse, -maxFriction, maxFriction);
            
            Math::Vector2 frictionVector = tangent * frictionImpulse;
            
            if (!bodyA.isStatic)
            {
                bodyA.velocity = bodyA.velocity + frictionVector * bodyA.inverseMass;
            }
            if (!bodyB.isStatic)
            {
                bodyB.velocity = bodyB.velocity - frictionVector * bodyB.inverseMass;
            }
        }
    }
    
    // Wake up sleeping bodies
    bodyA.isAwake = true;
    bodyB.isAwake = true;
    
    // Reset sleep timers since bodies are now active
    bodyA.sleepTimer = 0.0f;
    bodyB.sleepTimer = 0.0f;
    
    // Verify depenetration - log final separation
    Math::Vector2 finalSeparation = transformA.position - transformB.position;
    float finalDistance = finalSeparation.Length();
    std::cout << "[DEBUG] Post-resolution separation for " << entityA << " and " << entityB 
              << ": distance = " << finalDistance << std::endl;
    
    // === NOTIFY GAME LOGIC ===
    // Fire collision events for game-specific logic
    NotifyCollisionEvent(entityA, entityB);
    NotifyCollisionEvent(entityB, entityA);
    
    std::cout << "[CCD] Collision resolved between entities " 
             << entityA << " and " << entityB 
             << " at time " << timeOfImpact 
             << " | Final positions A:(" << transformA.position.x << "," << transformA.position.y 
             << ") B:(" << transformB.position.x << "," << transformB.position.y << ")"
             << " | Final velocities A:(" << bodyA.velocity.x << "," << bodyA.velocity.y 
             << ") B:(" << bodyB.velocity.x << "," << bodyB.velocity.y << ")" << std::endl;
}

void ContinuousCollisionSystem::NotifyCollisionEvent(EntityID entityA, EntityID entityB)
{
    // Check if entity has behavior component for game-specific logic
    if (m_ComponentStore && m_ComponentStore->HasComponent<BehaviorComponent>(entityA))
    {
        auto& behavior = m_ComponentStore->GetComponent<BehaviorComponent>(entityA);
        behavior.OnCollision(entityA, entityB);
    }
    
    // TODO: Add event system for global collision notifications
    // This would allow game systems to listen for specific collision types
    // Example: PhysicsWorldComponent could have callback registry
}

} // namespace Nyon::ECS