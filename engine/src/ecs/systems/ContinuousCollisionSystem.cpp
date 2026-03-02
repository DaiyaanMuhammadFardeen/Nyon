#include "nyon/ecs/systems/ContinuousCollisionSystem.h"
#include "nyon/ecs/components/BehaviorComponent.h"
#include <iostream>

// Debug logging macro - only output in debug builds
#ifdef _DEBUG
#define NYON_DEBUG_LOG(x) std::cout << x << std::endl
#else
#define NYON_DEBUG_LOG(x)
#endif

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
    
    // Calculate time of impact using swept AABB collision detection
    float toi = 1.0f; // Default to end of frame
    
    Math::Vector2 startPosA = transformA.position;
    Math::Vector2 startPosB = transformB.position;
    Math::Vector2 endPosA = predictedPosA;
    Math::Vector2 endPosB = predictedPosB;
    
    // Calculate actual half-extents from collider extents for AABB-based TOI
    float halfWidthA, halfHeightA, halfWidthB, halfHeightB;
    Math::Vector2 minA_start, maxA_start, minB_start, maxB_start;
    Math::Vector2 minA_end, maxA_end, minB_end, maxB_end;
    
    if (m_ComponentStore && 
        m_ComponentStore->HasComponent<ColliderComponent>(entityA) &&
        m_ComponentStore->HasComponent<ColliderComponent>(entityB))
    {
        const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityA);
        const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(entityB);
        
        // Calculate AABBs at start and end positions for swept calculation
        colliderA.CalculateAABB(startPosA, minA_start, maxA_start);
        colliderB.CalculateAABB(startPosB, minB_start, maxB_start);
        colliderA.CalculateAABB(endPosA, minA_end, maxA_end);
        colliderB.CalculateAABB(endPosB, minB_end, maxB_end);
        
        Math::Vector2 extentA_start = (maxA_start - minA_start) * 0.5f;
        Math::Vector2 extentB_start = (maxB_start - minB_start) * 0.5f;
        
        halfWidthA = extentA_start.x;
        halfHeightA = extentA_start.y;
        halfWidthB = extentB_start.x;
        halfHeightB = extentB_start.y;
    }
    else
    {
        // Default values for when colliders are not available
        halfWidthA = 16.0f;
        halfHeightA = 16.0f;
        halfWidthB = 16.0f;
        halfHeightB = 16.0f;
        
        // Calculate default AABBs
        minA_start = {startPosA.x - halfWidthA, startPosA.y - halfHeightA};
        maxA_start = {startPosA.x + halfWidthA, startPosA.y + halfHeightA};
        minB_start = {startPosB.x - halfWidthB, startPosB.y - halfHeightB};
        maxB_start = {startPosB.x + halfWidthB, startPosB.y + halfHeightB};
        minA_end = {endPosA.x - halfWidthA, endPosA.y - halfHeightA};
        maxA_end = {endPosA.x + halfWidthA, endPosA.y + halfHeightA};
        minB_end = {endPosB.x - halfWidthB, endPosB.y - halfHeightB};
        maxB_end = {endPosB.x + halfWidthB, endPosB.y + halfHeightB};
    }
    
    toi = Physics::CollisionDetectionStrategy::CalculateTimeOfImpact(
        startPosA, endPosA, startPosB, endPosB, 
        halfWidthA, halfHeightA, halfWidthB, halfHeightB);
    
    // If collision occurs within this timestep
    if (toi >= 0.0f && toi <= 1.0f)
    {
        // Use the unified collision resolution that maintains consistency between TOI and resolution
        ResolveCCDCollision(transformA, transformB, bodyA, bodyB,
                          startPosA, startPosB, endPosA, endPosB, toi, entityA, entityB);
    }
}

// Deprecated methods - functionality consolidated into ResolveCCDCollision
// These methods are kept for backward compatibility but should not be called
void ContinuousCollisionSystem::ResolveCCDImpulse(
    TransformComponent& transformA, TransformComponent& transformB,
    PhysicsBodyComponent& bodyA, PhysicsBodyComponent& bodyB,
    EntityID entityA, EntityID entityB)
{
    // This method is deprecated - use ResolveCCDCollision instead
    NYON_DEBUG_LOG("[WARNING] ResolveCCDImpulse called - this method is deprecated");
}

void ContinuousCollisionSystem::ResolveCCDPositionalCorrection(
    TransformComponent& transformA, TransformComponent& transformB,
    PhysicsBodyComponent& bodyA, PhysicsBodyComponent& bodyB,
    EntityID entityA, EntityID entityB)
{
    // This method is deprecated - use ResolveCCDCollision instead
    NYON_DEBUG_LOG("[WARNING] ResolveCCDPositionalCorrection called - this method is deprecated");
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
        // Already overlapping - use start positions for collision resolution
        transformA.position = startPosA;
        transformB.position = startPosB;
        std::cout << "[DEBUG] Handling already-overlapping case for entities " 
                  << entityA << " and " << entityB << std::endl;
    }
    else
    {
        // Move to exact collision point to prevent tunneling
        transformA.position = startPosA + (endPosA - startPosA) * timeOfImpact;
        transformB.position = startPosB + (endPosB - startPosB) * timeOfImpact;
    }
    
    // Calculate collision normal and penetration at the exact TOI moment
    // This ensures consistency between TOI calculation and resolution geometry
    Math::Vector2 collisionNormal;
    float penetrationDepth;
    
    // Get collider information at collision moment
    Math::Vector2 minA, maxA, minB, maxB;
    if (m_ComponentStore && 
        m_ComponentStore->HasComponent<ColliderComponent>(entityA) &&
        m_ComponentStore->HasComponent<ColliderComponent>(entityB))
    {
        const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityA);
        const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(entityB);
        
        colliderA.CalculateAABB(transformA.position, minA, maxA);
        colliderB.CalculateAABB(transformB.position, minB, maxB);
        
        // Calculate collision normal using minimum penetration axis at TOI moment
        Math::Vector2 delta = transformB.position - transformA.position;
        Math::Vector2 halfSizeA = (maxA - minA) * 0.5f;
        Math::Vector2 halfSizeB = (maxB - minB) * 0.5f;
        
        float overlapX = (halfSizeA.x + halfSizeB.x) - std::abs(delta.x);
        float overlapY = (halfSizeA.y + halfSizeB.y) - std::abs(delta.y);
        
        Math::Vector2 normal;
        if (overlapX < overlapY)
        {
            // X-axis has minimum penetration - collision normal points along X-axis
            // Standardized convention: normal points FROM B TO A (direction A should be pushed)
            normal = Math::Vector2(delta.x > 0 ? -1.0f : 1.0f, 0.0f);
        }
        else
        {
            // Y-axis has minimum penetration - collision normal points along Y-axis
            // Standardized convention: normal points FROM B TO A (direction A should be pushed)
            normal = Math::Vector2(0.0f, delta.y > 0 ? -1.0f : 1.0f);
        }
        
        penetrationDepth = -std::min(overlapX, overlapY);
    }
    else
    {
        // Fallback calculation using positions only
        Math::Vector2 separation = transformA.position - transformB.position;
        float distance = separation.Length();
        
        if (distance < 1e-6f)
        {
            separation = Math::Vector2{1.0f, 0.0f};
            distance = 1.0f;
        }
        
        collisionNormal = separation.Normalize();
        penetrationDepth = 0.0f; // No penetration in fallback case
    }
    
    // Use the pre-calculated collision normal and penetration depth
    Math::Vector2 normal = collisionNormal;
    // penetrationDepth is already calculated above
    
    // DEBUG: Print normal computation details
    NYON_DEBUG_LOG("[DEBUG] Consistent normal computation for entities " << entityA << " and " << entityB
              << " | TOI: " << timeOfImpact
              << " | Normal: (" << normal.x << ", " << normal.y << ")"
              << " | Penetration: " << penetrationDepth);
            
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
            NYON_DEBUG_LOG("[DEBUG] Positional correction for entities " << entityA << " and " << entityB
                      << " | Penetration: " << penetrationDepth
                      << " | Correction: (" << correction.x << ", " << correction.y << ")");
                            
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
                                    
                    NYON_DEBUG_LOG("[DEBUG] BodyA position changed from (" 
                              << oldPosA.x << ", " << oldPosA.y << ") to (" 
                              << transformA.position.x << ", " << transformA.position.y << ")");
                }
                if (!bodyB.isStatic)
                {
                    Math::Vector2 oldPosB = transformB.position;
                    transformB.position = transformB.position - 
                        correction * (bodyB.inverseMass / totalInvMass);
                                    
                    // Bounds checking
                    transformB.position.x = std::clamp(transformB.position.x, -1000.0f, 2000.0f);
                    transformB.position.y = std::clamp(transformB.position.y, -1000.0f, 2000.0f);
                                    
                    NYON_DEBUG_LOG("[DEBUG] BodyB position changed from (" 
                              << oldPosB.x << ", " << oldPosB.y << ") to (" 
                              << transformB.position.x << ", " << transformB.position.y << ")");
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
        // In Y-down coordinate system, ground normal points upward (negative Y)
        bool isGroundContact = (normal.y < -0.7f && bodyB.isStatic); // Upward-facing normal against static body (ground contact)
        if (velAlongNormal > 0 && !isGroundContact)
            return;
        
        // === INDUSTRY-STANDARD IMPULSE FORMULA ===
        // j = -(1 + e)(va - vb)·n / (1/ma + 1/mb)
        float impulseScalar = -(1.0f + restitution) * velAlongNormal;
        impulseScalar /= (bodyA.inverseMass + bodyB.inverseMass);
        
        Math::Vector2 impulse = normal * impulseScalar;
        
        // DEBUG: Print collision details
        NYON_DEBUG_LOG("[DEBUG] Collision between " << entityA << " and " << entityB 
                  << " | Restitution: " << restitution 
                  << " | Vel along normal: " << velAlongNormal
                  << " | Impulse scalar: " << impulseScalar
                  << " | Normal: (" << normal.x << ", " << normal.y << ")"
                  << " | BodyA vel: (" << bodyA.velocity.x << ", " << bodyA.velocity.y << ")"
                  << " | BodyB vel: (" << bodyB.velocity.x << ", " << bodyB.velocity.y << ")");
        
        // Apply impulses to linear velocities with proper constraint handling
        
        if (!bodyA.isStatic)
        {
            Math::Vector2 oldVelA = bodyA.velocity;
            bodyA.velocity = bodyA.velocity + impulse * bodyA.inverseMass;
            
            // Special handling for ground contacts - prevent downward velocity
            // In Y-down coordinate system, positive Y velocity = moving downward
            // Ground normal points upward (negative Y), so we stop positive Y velocity
            if (isGroundContact && bodyA.velocity.y > 0)
            {
                bodyA.velocity.y = 0.0f; // Stop downward motion against ground
            }
            
            NYON_DEBUG_LOG("[DEBUG] BodyA velocity changed from (" 
                      << oldVelA.x << ", " << oldVelA.y << ") to (" 
                      << bodyA.velocity.x << ", " << bodyA.velocity.y << ")");
        }
        if (!bodyB.isStatic)
        {
            Math::Vector2 oldVelB = bodyB.velocity;
            bodyB.velocity = bodyB.velocity - impulse * bodyB.inverseMass;
            
            // Special handling for ground contacts - prevent upward velocity on static bodies
            // Static bodies shouldn't move, but this ensures robustness
            // Ground normal points upward (negative Y), so we stop negative Y velocity
            if (isGroundContact && bodyB.velocity.y < 0)
            {
                bodyB.velocity.y = 0.0f; // Prevent upward motion of static body
            }
            
            NYON_DEBUG_LOG("[DEBUG] BodyB velocity changed from (" 
                      << oldVelB.x << ", " << oldVelB.y << ") to (" 
                      << bodyB.velocity.x << ", " << bodyB.velocity.y << ")");
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
    NYON_DEBUG_LOG("[DEBUG] Post-resolution separation for " << entityA << " and " << entityB 
              << ": distance = " << finalDistance);
    
    // === NOTIFY GAME LOGIC ===
    // Fire collision events for game-specific logic
    NotifyCollisionEvent(entityA, entityB);
    NotifyCollisionEvent(entityB, entityA);
    
    NYON_DEBUG_LOG("[CCD] Collision resolved between entities " 
             << entityA << " and " << entityB 
             << " at time " << timeOfImpact 
             << " | Final positions A:(" << transformA.position.x << "," << transformA.position.y 
             << ") B:(" << transformB.position.x << "," << transformB.position.y << ")"
             << " | Final velocities A:(" << bodyA.velocity.x << "," << bodyA.velocity.y 
             << ") B:(" << bodyB.velocity.x << "," << bodyB.velocity.y << ")");
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