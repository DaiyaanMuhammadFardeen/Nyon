#pragma once

#include "../math/Vector2.h"
#include "../ecs/components/PhysicsBodyComponent.h"
#include "../ecs/components/TransformComponent.h"
#include <vector>

namespace Nyon::Physics {

/**
 * @brief Contact constraint solver following Box2D's approach
 * 
 * This system properly resolves collisions by:
 * 1. Calculating contact points and normals
 * 2. Applying impulses iteratively for stable resolution
 * 3. Handling both linear and angular effects
 * 4. Managing friction and restitution properly
 * 5. Preventing penetration with positional correction
 */
class ContactSolver
{
public:
    struct ContactPoint
    {
        Math::Vector2 position;     // World contact point
        Math::Vector2 normal;       // Contact normal (points from A to B)
        float separation;          // Penetration depth (negative = penetration)
        float normalImpulse;       // Accumulated normal impulse
        float tangentImpulse;      // Accumulated tangent impulse
        float normalMass;          // Normal effective mass
        float tangentMass;         // Tangent effective mass
        float bias;                // Baumgarte stabilization bias
    };
    
    struct ContactManifold
    {
        uint32_t entityIdA;
        uint32_t entityIdB;
        std::vector<ContactPoint> points;
        float friction;
        float restitution;
        float invMassA, invMassB;
        float invInertiaA, invInertiaB;
    };
    
    /**
     * @brief Initialize contact constraints from collision data
     */
    static void InitializeContacts(std::vector<ContactManifold>& contacts,
                                 ECS::ComponentStore& componentStore);
    
    /**
     * @brief Solve velocity constraints iteratively
     * This applies impulses to resolve collisions properly
     */
    static void SolveVelocityConstraints(std::vector<ContactManifold>& contacts,
                                       ECS::ComponentStore& componentStore,
                                       int iterations = 8);
    
    /**
     * @brief Solve position constraints to prevent penetration
     * This corrects positions to eliminate overlap
     */
    static void SolvePositionConstraints(std::vector<ContactManifold>& contacts,
                                       ECS::ComponentStore& componentStore,
                                       int iterations = 3);

private:
    /**
     * @brief Calculate effective mass for impulse calculation
     */
    static float CalculateEffectiveMass(const Math::Vector2& normal,
                                      const Math::Vector2& rA,
                                      const Math::Vector2& rB,
                                      float invMassA, float invMassB,
                                      float invInertiaA, float invInertiaB);
    
    /**
     * @brief Apply normal impulse with restitution
     */
    static void ApplyNormalImpulse(ContactPoint& point,
                                 ECS::PhysicsBodyComponent& bodyA,
                                 ECS::PhysicsBodyComponent& bodyB,
                                 const Math::Vector2& normal,
                                 const Math::Vector2& rA,
                                 const Math::Vector2& rB,
                                 float desiredDeltaVelocity);
    
    /**
     * @brief Apply friction impulse
     */
    static void ApplyFrictionImpulse(ContactPoint& point,
                                   ECS::PhysicsBodyComponent& bodyA,
                                   ECS::PhysicsBodyComponent& bodyB,
                                   const Math::Vector2& tangent,
                                   const Math::Vector2& rA,
                                   const Math::Vector2& rB,
                                   float maxFrictionImpulse);
};

} // namespace Nyon::Physics