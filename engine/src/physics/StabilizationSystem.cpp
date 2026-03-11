#include <nyon/physics/StabilizationSystem.h>
#include <nyon/physics/SATCollisionDetector.h>
#include <nyon/physics/ConstraintSolver.h>
#include <algorithm>
#include <cmath>

namespace Nyon {

// ============================================================================
// Persistent Contact Matching
// ============================================================================

std::vector<PersistentContact> StabilizationSystem::UpdatePersistentContacts(
    const std::vector<ContactManifold>& currentContacts,
    const std::vector<TransformComponent>& transforms)
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
            
            Math::Vector2 localPointA = transformA.rotation.Inverse() * (contactPoint.position - transformA.position);
            Math::Vector2 localPointB = transformB.rotation.Inverse() * (contactPoint.position - transformB.position);
            
            // Try to find matching cached contact
            PersistentContact* cachedContact = FindMatchingContact(
                m_PersistentContacts,
                manifold.entityIdA,
                manifold.entityIdB,
                manifold.shapeIdA,
                manifold.shapeIdB,
                featureId,
                localPointA,
                localPointB,
                localPointTolerance
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

PersistentContact* StabilizationSystem::FindMatchingContact(
    std::vector<PersistentContact>& contacts,
    uint32_t entityIdA,
    uint32_t entityIdB,
    uint32_t shapeIdA,
    uint32_t shapeIdB,
    uint32_t featureId,
    const Math::Vector2& localPointA,
    const Math::Vector2& localPointB,
    float tolerance)
{
    for (auto& contact : contacts)
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

uint32_t StabilizationSystem::ComputeFeatureId(
    const ContactPoint& contactPoint,
    const Math::Vector2& normal)
{
    // Quantize normal to reduce sensitivity to small angle changes
    const float quantizationFactor = 10.0f;
    int32_t quantizedX = static_cast<int32_t>(std::round(normal.x * quantizationFactor));
    int32_t quantizedY = static_cast<int32_t>(std::round(normal.y * quantizationFactor));
    
    // Combine with contact position for unique feature identification
    uint32_t hash = 0;
    hash ^= static_cast<uint32_t>(quantizedX) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= static_cast<uint32_t>(quantizedY) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    
    return hash;
}

// ============================================================================
// Warm Starting
// ============================================================================

void StabilizationSystem::ApplyWarmStarting(
    const std::vector<PersistentContact>& persistentContacts,
    const std::vector<SolverBody>& solverBodies,
    std::vector<VelocityConstraint>& velocityConstraints,
    float dtRatio)
{
    // dtRatio accounts for changes in time step between frames
    const float warmStartScaling = 0.9f; // Slightly reduce cached impulses for stability
    
    for (auto& vc : velocityConstraints)
    {
        // Find matching persistent contacts
        for (const auto& pc : persistentContacts)
        {
            if ((pc.entityIdA == vc.indexA && pc.entityIdB == vc.indexB) ||
                (pc.entityIdA == vc.indexB && pc.entityIdB == vc.indexA))
            {
                bool swapped = (pc.entityIdA == vc.indexB);
                
                for (auto& vcp : vc.points)
                {
                    // Apply cached normal impulse
                    float scaledNormalImpulse = pc.normalImpulse * warmStartScaling * dtRatio;
                    vcp.normalImpulse = scaledNormalImpulse;
                    
                    // Apply cached tangent impulse (friction)
                    float scaledTangentImpulse = pc.tangentImpulse * warmStartScaling * dtRatio;
                    vcp.tangentImpulse = scaledTangentImpulse;
                    
                    // Apply impulses to bodies for immediate effect
                    if (vc.indexA != kInvalidBodyIndex)
                    {
                        const float invMassA = solverBodies[vc.indexA].invMass;
                        const float invInertiaA = solverBodies[vc.indexA].invInertia;
                        
                        Math::Vector2 P = vc.normal * scaledNormalImpulse + vc.tangent * scaledTangentImpulse;
                        solverBodies[vc.indexA].linearVelocity += P * invMassA;
                        
                        if (swapped)
                            solverBodies[vc.indexA].angularVelocity -= Math::Vector2::Cross(vcp.rA, P) * invInertiaA;
                        else
                            solverBodies[vc.indexA].angularVelocity += Math::Vector2::Cross(vcp.rA, P) * invInertiaA;
                    }
                    
                    if (vc.indexB != kInvalidBodyIndex)
                    {
                        const float invMassB = solverBodies[vc.indexB].invMass;
                        const float invInertiaB = solverBodies[vc.indexB].invInertia;
                        
                        Math::Vector2 P = -(vc.normal * scaledNormalImpulse + vc.tangent * scaledTangentImpulse);
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

float StabilizationSystem::ComputeSpeculativeDistance(
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

float StabilizationSystem::ComputeSpeculativeDistance(
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

void SleepManager::UpdateSleepTime(
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

bool SleepManager::ShouldSleep(uint32_t bodyIndex) const
{
    if (bodyIndex >= bodySleepTime.size())
        return false;
    
    return bodySleepTime[bodyIndex] >= timeToSleep;
}

void SleepManager::WakeBody(uint32_t bodyIndex)
{
    if (bodyIndex < bodySleepTime.size())
    {
        bodySleepTime[bodyIndex] = 0.0f;
    }
}

void SleepManager::Clear()
{
    std::fill(bodySleepTime.begin(), bodySleepTime.end(), 0.0f);
}

// ============================================================================
// Impulse Caching
// ============================================================================

void StabilizationSystem::CacheImpulses(
    const std::vector<VelocityConstraint>& velocityConstraints,
    std::vector<PersistentContact>& persistentContacts)
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

void StabilizationSystem::ApplySplitImpulses(
    const std::vector<PositionConstraint>& positionConstraints,
    std::vector<SolverBody>& solverBodies,
    float baumgarteBeta,
    float baumgarteSlop)
{
    // Apply position corrections separately from velocity corrections
    // This helps reduce jitter by decoupling position and velocity solving
    
    const float maxPositionCorrection = 0.2f; // Limit to prevent instability
    
    for (const auto& pc : positionConstraints)
    {
        float C = std::max(pc.separation + baumgarteSlop, 0.0f);
        float positionImpulse = -baumgarteBeta * C;
        
        // Clamp position correction
        positionImpulse = std::clamp(positionImpulse, -maxPositionCorrection, maxPositionCorrection);
        
        Math::Vector2 P = pc.normal * positionImpulse;
        
        if (pc.indexA != kInvalidBodyIndex)
        {
            float invMassA = solverBodies[pc.indexA].invMass;
            float invInertiaA = solverBodies[pc.indexA].invInertia;
            
            solverBodies[pc.indexA].position += P * invMassA;
            solverBodies[pc.indexA].angle -= Math::Vector2::Cross(pc.rA, P) * invInertiaA;
            
            // Update transform immediately
            solverBodies[pc.indexA].rotation = Math::Rotation2D(solverBodies[pc.indexA].angle);
        }
        
        if (pc.indexB != kInvalidBodyIndex)
        {
            float invMassB = solverBodies[pc.indexB].invMass;
            float invInertiaB = solverBodies[pc.indexB].invInertia;
            
            solverBodies[pc.indexB].position -= P * invMassB;
            solverBodies[pc.indexB].angle += Math::Vector2::Cross(pc.rB, P) * invInertiaB;
            
            // Update transform immediately
            solverBodies[pc.indexB].rotation = Math::Rotation2D(solverBodies[pc.indexB].angle);
        }
    }
}

} // namespace Nyon
