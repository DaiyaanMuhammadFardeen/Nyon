#pragma once

#include <nyon/physics/DynamicTree.h>
#include <nyon/physics/SATCollisionDetector.h>
#include <nyon/physics/ContinuousCollisionDetection.h>
#include <nyon/physics/BoundarySystem.h>
#include <nyon/physics/ConstraintSolver.h>
#include <nyon/physics/StabilizationSystem.h>
#include <nyon/physics/CollisionDetectionStrategy.h>
#include <nyon/ecs/components/TransformComponent.h>
#include <nyon/ecs/components/PhysicsBodyComponent.h>
#include <nyon/ecs/components/ColliderComponent.h>
#include <vector>
#include <unordered_map>

namespace Nyon {

/**
 * @brief Broad-phase collision pair from dynamic tree
 */
struct BroadPhasePair
{
    uint32_t entityIdA;
    uint32_t entityIdB;
    uint32_t shapeIdA;
    uint32_t shapeIdB;
    AABB aabbA;
    AABB aabbB;
};

/**
 * @brief Narrow-phase contact with TOI information
 */
struct NarrowPhaseContact
{
    ContactManifold manifold;
    float toi = 1.0f; // Time of impact [0,1]
    bool needsCCD = false;
};

/**
 * @brief Complete Physics Pipeline integrating all collision systems
 * 
 * Orchestrates:
 * 1. Broad-phase using Dynamic Tree (AABB hierarchy)
 * 2. Collision strategy selection (CCD vs discrete)
 * 3. Narrow-phase SAT collision detection
 * 4. Continuous Collision Detection for fast objects
 * 5. Boundary collision detection
 * 6. Constraint solving with warm starting
 * 7. Stabilization for anti-flickering
 */
class PhysicsPipeline
{
public:
    PhysicsPipeline();
    ~PhysicsPipeline();
    
    /**
     * @brief Initialize pipeline with expected capacities
     */
    void Initialize(size_t maxEntities, size_t maxContacts);
    
    /**
     * @brief Complete physics update for one time step
     * 
     * @param componentStore ECS component store
     * @param deltaTime Time step
     * @param gravity Gravity vector
     * @param bounds World boundaries
     */
    void Update(
        class ECS::ComponentStore& componentStore,
        float deltaTime,
        const Math::Vector2& gravity,
        const Boundary& bounds);
    
    /**
     * @brief Step 1: Broad-phase pair detection using dynamic tree
     */
    void BroadPhase(
        ECS::ComponentStore& componentStore,
        const std::vector<ECS::EntityID>& activeEntities);
    
    /**
     * @brief Step 2: Narrow-phase SAT collision detection
     */
    void NarrowPhase(
        ECS::ComponentStore& componentStore,
        float deltaTime);
    
    /**
     * @brief Step 3: Continuous collision detection for fast objects
     */
    void ContinuousCollisionDetection(
        ECS::ComponentStore& componentStore,
        float deltaTime);
    
    /**
     * @brief Step 4: Boundary collision detection
     */
    void BoundaryCollision(
        ECS::ComponentStore& componentStore,
        const Boundary& bounds,
        float deltaTime);
    
    /**
     * @brief Step 5: Solve velocity and position constraints
     */
    void SolveConstraints(
        ECS::ComponentStore& componentStore,
        const Math::Vector2& gravity,
        float deltaTime,
        int velocityIterations = 8,
        int positionIterations = 3);
    
    /**
     * @brief Step 6: Apply stabilization and sleep management
     */
    void ApplyStabilization(
        ECS::ComponentStore& componentStore,
        float deltaTime);
    
    /**
     * @brief Get statistics about pipeline performance
     */
    struct Statistics
    {
        size_t broadPhasePairs = 0;
        size_t narrowPhaseContacts = 0;
        size_t CCDQueries = 0;
        size_t boundaryContacts = 0;
        size_t activeConstraints = 0;
        size_t sleepingBodies = 0;
        size_t persistentContacts = 0;
    };
    
    Statistics GetStatistics() const;
    
private:
    DynamicTree m_DynamicTree;
    SATCollisionDetector m_SATDetector;
    BoundarySystem m_BoundarySystem;
    ConstraintSolver m_ConstraintSolver;
    StabilizationSystem m_StabilizationSystem;
    
    std::vector<BroadPhasePair> m_BroadPhasePairs;
    std::vector<NarrowPhaseContact> m_Contacts;
    std::vector<ContactManifold> m_FinalContacts;
    
    std::unordered_map<uint32_t, uint32_t> m_EntityProxyMap; // entityId -> proxyId
    
    size_t m_MaxContacts;
    bool m_Initialized;
    
    /**
     * @brief Update dynamic tree with current entity AABBs
     */
    void UpdateDynamicTree(
        ECS::ComponentStore& componentStore,
        const std::vector<ECS::EntityID>& activeEntities);
    
    /**
     * @brief Query dynamic tree for overlapping pairs
     */
    void QueryOverlappingPairs();
    
    /**
     * @brief Determine if CCD is needed for a collision pair
     */
    static bool NeedsCCDForPair(
        const ECS::PhysicsBodyComponent& bodyA,
        const ECS::PhysicsBodyComponent& bodyB,
        float deltaTime,
        float minExtentA,
        float minExtentB);
    
    /**
     * @brief Create solver bodies from physics components
     */
    std::vector<SolverBody> CreateSolverBodies(
        ECS::ComponentStore& componentStore,
        const std::vector<ContactManifold>& contacts,
        const Math::Vector2& gravity,
        float deltaTime);
    
    /**
     * @brief Write solved velocities back to components
     */
    void WriteBackToComponents(
        ECS::ComponentStore& componentStore,
        const std::vector<SolverBody>& solverBodies);
};

} // namespace Nyon
