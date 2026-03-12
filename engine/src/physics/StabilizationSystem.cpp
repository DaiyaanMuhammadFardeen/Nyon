#include <nyon/physics/StabilizationSystem.h>
#include <nyon/physics/SATCollisionDetector.h>
#include <nyon/physics/ConstraintSolver.h>
#include <algorithm>
#include <cmath>

namespace Nyon {

// ============================================================================
// Persistent Contact Matching
// ============================================================================

std::vector<Nyon::Physics::PersistentContact> Nyon::Physics::StabilizationSystem::UpdatePersistentContacts(
    const std::vector<Nyon::Physics::ContactManifold>& currentContacts,
    const std::vector<Nyon::ECS::TransformComponent>& transforms)
{
    std::vector<PersistentContact> newPersistentContacts;
    newPersistentContacts.reserve(currentContacts.size());
    
    const float localPointTolerance = 0.01f; // Small tolerance for floating point errors
    
    for (const auto& manifold : currentContacts)
    {
        if (!manifold.touching || manifold.points.empty())
            continue;
        
        for (const auto& contactPoint : manifold.points)
        {
            // Compute feature ID based on contact normal and positions
            uint32_t featureId = ComputeFeatureId(contactPoint, manifold.normal);
            
            // Transform world space points to local space for persistence
            const TransformComponent& transformA = transforms[manifold.entityIdA];
            const TransformComponent& transformB = transforms[manifold.entityIdB];
            
            // Convert rotation angle to inverse rotation (negative angle)
            float invRotA = -transformA.rotation;
            float invRotB = -transformB.rotation;
            float cosInvA = std::cos(invRotA), sinInvA = std::sin(invRotA);
            float cosInvB = std::cos(invRotB), sinInvB = std::sin(invRotB);
            
            Math::Vector2 dA = contactPoint.position - transformA.position;
            Math::Vector2 dB = contactPoint.position - transformB.position;
            
            // Apply inverse rotation to get local space points
            Math::Vector2 localPointA = { dA.x * cosInvA - dA.y * sinInvA, dA.x * sinInvA + dA.y * cosInvA };
            Math::Vector2 localPointB = { dB.x * cosInvB - dB.y * sinInvB, dB.x * sinInvB + dB.y * cosInvB };
            
            // Try to find matching cached contact
            PersistentContact* cachedContact = FindMatchingContact(
                manifold.entityIdA,
                manifold.entityIdB,
                manifold.shapeIdA,
                manifold.shapeIdB,
                featureId,
                localPointA,
                localPointB
            );
            
            if (cachedContact && cachedContact->active)
            {
                // Reuse cached contact with accumulated frame count
                PersistentContact persistentContact;
                persistentContact.entityIdA = manifold.entityIdA;
                persistentContact.entityIdB = manifold.entityIdB;
                persistentContact.shapeIdA = manifold.shapeIdA;
                persistentContact.shapeIdB = manifold.shapeIdB;
                persistentContact.featureId = featureId;
                persistentContact.localPointA = localPointA;
                persistentContact.localPointB = localPointB;
                persistentContact.frameCount = cachedContact->frameCount + 1;
                persistentContact.active = true;
                
                // Transfer cached impulses for warm starting
                persistentContact.normalImpulse = cachedContact->normalImpulse;
                persistentContact.tangentImpulse = cachedContact->tangentImpulse;
                
                newPersistentContacts.push_back(persistentContact);
            }
            else
            {
                // New contact - start with zero impulses
                PersistentContact persistentContact;
                persistentContact.entityIdA = manifold.entityIdA;
                persistentContact.entityIdB = manifold.entityIdB;
                persistentContact.shapeIdA = manifold.shapeIdA;
                persistentContact.shapeIdB = manifold.shapeIdB;
                persistentContact.featureId = featureId;
                persistentContact.localPointA = localPointA;
                persistentContact.localPointB = localPointB;
                persistentContact.frameCount = 1;
                persistentContact.active = true;
                persistentContact.normalImpulse = 0.0f;
                persistentContact.tangentImpulse = 0.0f;
                
                newPersistentContacts.push_back(persistentContact);
            }
        }
    }
    
    // Update cached contacts for next frame
    m_PersistentContacts = newPersistentContacts;
    
    return newPersistentContacts;
}

Nyon::Physics::PersistentContact* Nyon::Physics::StabilizationSystem::FindMatchingContact(
    uint32_t entityIdA,
    uint32_t entityIdB,
    uint32_t shapeIdA,
    uint32_t shapeIdB,
    uint32_t featureId,
    const Math::Vector2& localPointA,
    const Math::Vector2& localPointB)
{
    const float tolerance = 0.01f; // Small tolerance for floating point errors
    
    for (auto& contact : m_PersistentContacts)
    {
        if (!contact.active)
            continue;
        
        // Check entity and shape match
        if (contact.entityIdA != entityIdA || 
            contact.entityIdB != entityIdB ||
            contact.shapeIdA != shapeIdA || 
            contact.shapeIdB != shapeIdB)
            continue;
        
        // Check feature ID match (primary matching criterion)
        if (contact.featureId != featureId)
            continue;
        
        // Check local point proximity (secondary matching criterion)
        float distSqA = Math::Vector2::DistanceSquared(contact.localPointA, localPointA);
        float distSqB = Math::Vector2::DistanceSquared(contact.localPointB, localPointB);
        
        if (distSqA <= tolerance * tolerance && distSqB <= tolerance * tolerance)
        {
            return &contact;
        }
    }
    
    return nullptr;
}

uint32_t Nyon::Physics::StabilizationSystem::ComputeFeatureId(
    const Nyon::Physics::ContactPoint& contactPoint,
    const Math::Vector2& normal)
{
    // Quantize normal and position to reduce sensitivity to small changes
    const float quantizationFactor = 10.0f;
    auto qi = [quantizationFactor](float v) { 
        return static_cast<int32_t>(std::round(v * quantizationFactor)); 
    };
    
    // Combine normal and position components into hash using MurmurHash-style mixing
    uint32_t hash = 0;
    auto mix = [&](uint32_t v) {
        hash ^= v + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    };
    
    // Mix in quantized normal components
    mix(static_cast<uint32_t>(qi(normal.x)));
    mix(static_cast<uint32_t>(qi(normal.y)));
    
    // Mix in quantized contact position components
    mix(static_cast<uint32_t>(qi(contactPoint.position.x)));
    mix(static_cast<uint32_t>(qi(contactPoint.position.y)));
    
    return hash;
}

// ============================================================================
// Warm Starting
// ============================================================================

void Nyon::Physics::StabilizationSystem::ApplyWarmStarting(
    const std::vector<Nyon::Physics::PersistentContact>& persistentContacts,
    std::vector<Nyon::Physics::SolverBody>& solverBodies,  // Non-const to allow mutation
    std::vector<Nyon::Physics::VelocityConstraint>& velocityConstraints)
{
    // Scale only by dtRatio (which is 1.0 for constant timestep)
    // Do NOT apply constant decay factor - that would exponentially decay impulses to zero
    
    for (auto& vc : velocityConstraints)
    {
        // Find matching persistent contacts by entity ID (not index!)
        for (const auto& pc : persistentContacts)
        {
            if ((pc.entityIdA == vc.entityIdA && pc.entityIdB == vc.entityIdB) ||
                (pc.entityIdA == vc.entityIdB && pc.entityIdB == vc.entityIdA))
            {
                bool swapped = (pc.entityIdA == vc.entityIdB);
                
                for (auto& vcp : vc.points)
                {
                    // Apply cached normal impulse - scale only by dtRatio
                    // Note: dtRatio parameter removed, assuming constant timestep (dtRatio=1.0)
                    vcp.normalImpulse = pc.normalImpulse; // No scaling
                    
                    // Apply cached tangent impulse (friction) - no scaling
                    vcp.tangentImpulse = pc.tangentImpulse; // No scaling
                    
                    // Apply impulses to bodies for immediate effect
                    if (vc.indexA != UINT32_MAX)
                    {
                        const float invMassA = solverBodies[vc.indexA].invMass;
                        const float invInertiaA = solverBodies[vc.indexA].invInertia;
                        
                        Math::Vector2 P = vc.normal * vcp.normalImpulse + vc.tangent * vcp.tangentImpulse;
                        solverBodies[vc.indexA].linearVelocity += P * invMassA;
                        
                        if (swapped)
                            solverBodies[vc.indexA].angularVelocity -= Math::Vector2::Cross(vcp.rA, P) * invInertiaA;
                        else
                            solverBodies[vc.indexA].angularVelocity += Math::Vector2::Cross(vcp.rA, P) * invInertiaA;
                    }
                    
                    if (vc.indexB != UINT32_MAX)
                    {
                        const float invMassB = solverBodies[vc.indexB].invMass;
                        const float invInertiaB = solverBodies[vc.indexB].invInertia;
                        
                        Math::Vector2 P = -(vc.normal * vcp.normalImpulse + vc.tangent * vcp.tangentImpulse);
                        solverBodies[vc.indexB].linearVelocity += P * invMassB;
                        
                        if (swapped)
                            solverBodies[vc.indexB].angularVelocity += Math::Vector2::Cross(vcp.rB, P) * invInertiaB;
                        else
                            solverBodies[vc.indexB].angularVelocity -= Math::Vector2::Cross(vcp.rB, P) * invInertiaB;
                    }
                }
                
                break; // Found matching contact
            }
        }
    }
}

// ============================================================================
// Speculative Contacts
// ============================================================================

float Nyon::Physics::StabilizationSystem::ComputeSpeculativeDistance(
    const Math::Vector2& relativeVelocity,
    float minExtent,
    float baseSpeculativeDistance)
{
    // Add extra margin proportional to relative speed to prevent tunneling
    const float velocityMarginFactor = 0.002f; // Tunable parameter
    const float maxVelocityMargin = 0.1f; // Cap to prevent excessive margins
    
    float velocityMagnitude = relativeVelocity.Length();
    float velocityMargin = std::min(velocityMagnitude * velocityMarginFactor, maxVelocityMargin);
    
    // Add extent-based margin for larger objects
    const float extentMarginFactor = 0.05f;
    float extentMargin = minExtent * extentMarginFactor;
    
    return baseSpeculativeDistance + velocityMargin + extentMargin;
}

float Nyon::Physics::StabilizationSystem::ComputeSpeculativeDistance(
    const Math::Vector2& velocityA,
    const Math::Vector2& velocityB,
    float angularVelocityA,
    float angularVelocityB,
    float minExtentA,
    float minExtentB,
    float dt,
    float baseSpeculativeDistance)
{
    // Compute worst-case relative velocity at perimeter
    Math::Vector2 linearRelativeVelocity = velocityB - velocityA;
    
    // Add angular velocity contribution (approximate)
    float angularContributionA = std::abs(angularVelocityA) * minExtentA;
    float angularContributionB = std::abs(angularVelocityB) * minExtentB;
    
    float totalRelativeVelocity = linearRelativeVelocity.Length() + angularContributionA + angularContributionB;
    
    // Scale by time step to get distance traveled
    float travelDistance = totalRelativeVelocity * dt;
    
    // Add speculative margin
    const float safetyFactor = 1.2f; // 20% safety margin
    return baseSpeculativeDistance + travelDistance * safetyFactor;
}

// ============================================================================
// Sleep Management
// ============================================================================

void Nyon::Physics::SleepManager::UpdateSleepTime(
    uint32_t bodyIndex,
    float linearVelocity,
    float angularVelocity,
    float dt)
{
    if (bodyIndex >= bodySleepTime.size())
    {
        // Resize if needed
        if (bodyIndex >= bodySleepTime.capacity())
        {
            bodySleepTime.resize(std::max(bodySleepTime.size() * 2, bodyIndex + 1), 0.0f);
        }
        else
        {
            bodySleepTime.resize(bodyIndex + 1, 0.0f);
        }
    }
    
    // Compute kinetic energy metric
    float kineticMetric = linearVelocity * linearVelocity + 
                         0.1f * angularVelocity * angularVelocity; // Weight angular less
    
    if (kineticMetric < sleepThreshold * sleepThreshold)
    {
        // Body is slowing down - accumulate sleep time
        bodySleepTime[bodyIndex] += dt;
    }
    else
    {
        // Body is active - reset sleep timer
        bodySleepTime[bodyIndex] = 0.0f;
    }
}

bool Nyon::Physics::SleepManager::ShouldSleep(uint32_t bodyIndex) const
{
    if (bodyIndex >= bodySleepTime.size())
        return false;
    
    return bodySleepTime[bodyIndex] >= timeToSleep;
}

void Nyon::Physics::SleepManager::WakeBody(uint32_t bodyIndex)
{
    if (bodyIndex < bodySleepTime.size())
    {
        bodySleepTime[bodyIndex] = 0.0f;
    }
}

void Nyon::Physics::SleepManager::Clear()
{
    std::fill(bodySleepTime.begin(), bodySleepTime.end(), 0.0f);
}

// ============================================================================
// Impulse Caching
// ============================================================================

void Nyon::Physics::StabilizationSystem::CacheImpulses(
    const std::vector<Nyon::Physics::VelocityConstraint>& velocityConstraints,
    std::vector<Nyon::Physics::PersistentContact>& persistentContacts)
{
    // Update cached impulses in persistent contacts from solved velocity constraints
    for (const auto& vc : velocityConstraints)
    {
        for (const auto& vcp : vc.points)
        {
            // Find matching persistent contact
            for (auto& pc : persistentContacts)
            {
                if ((pc.entityIdA == vc.indexA && pc.entityIdB == vc.indexB) ||
                    (pc.entityIdA == vc.indexB && pc.entityIdB == vc.indexA))
                {
                    // Cache the solved impulses
                    pc.normalImpulse = vcp.normalImpulse;
                    pc.tangentImpulse = vcp.tangentImpulse;
                    break;
                }
            }
        }
    }
}

// ============================================================================
// Split Impulse Application
// ============================================================================

void Nyon::Physics::StabilizationSystem::ApplySplitImpulses(
    const std::vector<Nyon::Physics::PositionConstraint>& positionConstraints,
    std::vector<Nyon::Physics::SolverBody>& solverBodies,
    float baumgarteBeta,
    float baumgarteSlop)
{
    // Apply position corrections separately from velocity corrections
    // This helps reduce jitter by decoupling position and velocity solving
    
    const float maxPositionCorrection = 0.2f; // Limit to prevent instability
    
    for (const auto& pc : positionConstraints)
    {
        // separation < 0 means penetration. Correct when separation < -slop.
        float C = std::min(pc.separation + baumgarteSlop, 0.0f); // <= 0 when penetrating
        float positionImpulse = -baumgarteBeta * C; // now positive = push apart
        
        // Clamp position correction to positive range
        positionImpulse = std::clamp(positionImpulse, 0.0f, maxPositionCorrection);
        
        Math::Vector2 P = pc.localNormal * positionImpulse;
        
        if (pc.indexA != UINT32_MAX)
        {
            float invMassA = solverBodies[pc.indexA].invMass;
            float invInertiaA = solverBodies[pc.indexA].invInertia;
            
            // Body A moves in -normal direction (pushed back)
            solverBodies[pc.indexA].position -= P * invMassA;
            solverBodies[pc.indexA].angle -= Math::Vector2::Cross(pc.localPointA, P) * invInertiaA;
            
            // Update transform immediately
            // Removed: solverBodies[pc.indexA].rotation = Math::Rotation2D(solverBodies[pc.indexA].angle);
        }
        
        if (pc.indexB != UINT32_MAX)
        {
            float invMassB = solverBodies[pc.indexB].invMass;
            float invInertiaB = solverBodies[pc.indexB].invInertia;
            
            // Body B moves in +normal direction (pushed forward)
            solverBodies[pc.indexB].position += P * invMassB;
            solverBodies[pc.indexB].angle += Math::Vector2::Cross(pc.localPointB, P) * invInertiaB;
            
            // Update transform immediately
            // Removed: solverBodies[pc.indexB].rotation = Math::Rotation2D(solverBodies[pc.indexB].angle);
        }
    }
}

} // namespace Nyon
