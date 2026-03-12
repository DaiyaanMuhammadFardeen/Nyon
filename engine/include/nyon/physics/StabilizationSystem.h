// SPDX-FileCopyrightText: 2026 Nyon Engine
// SPDX-License-Identifier: MIT

#pragma once

#include "nyon/physics/SATCollisionDetector.h"
#include "nyon/physics/ConstraintSolver.h"
#include "nyon/ecs/components/TransformComponent.h"
#include <vector>
#include <cstdint>

namespace Nyon::Physics
{
    /**
     * @brief Contact persistence data for stability across frames
     */
    struct PersistentContact
    {
        uint32_t entityIdA;
        uint32_t entityIdB;
        uint32_t shapeIdA;
        uint32_t shapeIdB;
        uint32_t featureId;      // Feature identifier for matching
        Math::Vector2 localPointA;  // Contact point in local space of A
        Math::Vector2 localPointB;  // Contact point in local space of B
        float normalImpulse;     // Cached normal impulse from last frame
        float tangentImpulse;    // Cached tangent impulse from last frame
        int frameCount;          // Number of frames this contact persisted
        bool active;             // Is this contact currently active
        
        PersistentContact()
            : entityIdA(0), entityIdB(0), shapeIdA(0), shapeIdB(0)
            , featureId(0), localPointA{0.0f, 0.0f}, localPointB{0.0f, 0.0f}
            , normalImpulse(0.0f), tangentImpulse(0.0f), frameCount(0), active(false)
        {}
    };
    
    /**
     * @brief Sleep management for bodies at rest
     */
    struct SleepManager
    {
        float sleepThreshold = 0.01f;       // Velocity threshold for sleep
        float wakeThreshold = 0.1f;         // Velocity to wake up
        float timeToSleep = 0.5f;           // Time before sleeping (seconds)
        
        std::vector<float> bodySleepTime;   // Accumulated sleep time per body
        
        void Initialize(size_t bodyCount)
        {
            bodySleepTime.resize(bodyCount, 0.0f);
        }
        
        bool ShouldSleep(uint32_t bodyIndex) const;
        
        void UpdateSleepTime(uint32_t bodyIndex, float linearVelocity, float angularVelocity, float dt);
        
        bool CanSleep(uint32_t bodyIndex) const
        {
            return bodySleepTime[bodyIndex] >= timeToSleep;
        }
        
        void WakeBody(uint32_t bodyIndex);
        
        void Clear();
    };
    
    /**
     * @brief Advanced stabilization system to eliminate flickering and jittering
     * 
     * Techniques used:
     * - Contact persistence across frames
     * - Impulse caching and warm starting
     * - Split impulses for position correction
     * - TOI event buffering
     * - Sleep management
     * - Speculative contacts with proper bias
     */
    class StabilizationSystem
    {
    public:
        StabilizationSystem();
        ~StabilizationSystem();
        
        /**
         * @brief Initialize system with expected capacity
         */
        void Initialize(size_t maxContacts, size_t bodyCount);
        
        /**
         * @brief Update persistent contacts from current frame
         * 
         * Matches new contacts with cached contacts using feature IDs.
         * 
         * @param currentContacts Current frame contact manifolds
         * @param transforms Entity transforms
         * @return Updated persistent contacts
         */
        std::vector<PersistentContact> UpdatePersistentContacts(
            const std::vector<ContactManifold>& currentContacts,
            const std::vector<struct TransformComponent>& transforms);
        
        /**
         * @brief Apply warm starting impulses from persistent contacts
         * 
         * Uses cached impulses to improve solver convergence.
         * 
         * @param persistentContacts Persistent contact data
         * @param solverBodies Non-const reference to solver bodies (will be mutated)
         * @param velocityConstraints Velocity constraints to initialize
         */
        void ApplyWarmStarting(
            const std::vector<PersistentContact>& persistentContacts,
            std::vector<SolverBody>& solverBodies,  // Non-const to allow mutation
            std::vector<VelocityConstraint>& velocityConstraints);
        
        /**
         * @brief Compute speculative contact distance
         * 
         * Adds extra margin based on relative velocity to prevent tunneling.
         * 
         * @param relativeVelocity Relative velocity at contact
         * @param minExtent Minimum shape extent
         * @param baseSpeculativeDistance Base speculative distance
         * @return Computed speculative distance
         */
        static float ComputeSpeculativeDistance(
            const Math::Vector2& relativeVelocity,
            float minExtent,
            float baseSpeculativeDistance = 0.02f);
        
        /**
         * @brief Get sleep manager for body sleep control
         */
        SleepManager& GetSleepManager() { return m_SleepManager; }
        const SleepManager& GetSleepManager() const { return m_SleepManager; }
        
        /**
         * @brief Enable/disable contact persistence
         */
        void SetContactPersistence(bool enabled) { m_UseContactPersistence = enabled; }
        
        /**
         * @brief Enable/disable sleep
         */
        void SetSleepEnabled(bool enabled) { m_SleepEnabled = enabled; }
        
        /**
         * @brief Get statistics
         */
        size_t GetActivePersistentContactCount() const;
        size_t GetSleepingBodyCount() const;
        
    private:
        std::vector<PersistentContact> m_PersistentContacts;
        SleepManager m_SleepManager;
        bool m_UseContactPersistence;
        bool m_SleepEnabled;
        size_t m_MaxContacts;
        
        /**
         * @brief Match current contact with persistent contact
         * 
         * Uses feature IDs and local points for matching.
         */
        PersistentContact* FindMatchingContact(
            uint32_t entityIdA,
            uint32_t entityIdB,
            uint32_t shapeIdA,
            uint32_t shapeIdB,
            uint32_t featureId,
            const Math::Vector2& localPointA,
            const Math::Vector2& localPointB);
        
        /**
         * @brief Compute feature ID from contact point and normal
         */
        static uint32_t ComputeFeatureId(const ContactPoint& cp, const Math::Vector2& normal);
        
        /**
         * @brief Create new persistent contact from manifold
         */
        PersistentContact CreatePersistentContact(
            const ContactManifold& manifold,
            const ContactPoint& point,
            const struct TransformComponent& transformA,
            const struct TransformComponent& transformB);
    };
}
