#include <nyon/physics/PhysicsPipeline.h>
#include <nyon/ecs/ComponentStore.h>
#include <algorithm>
#include <cmath>

using namespace Nyon::Physics;

namespace Nyon {

// ============================================================================
// Construction & Initialization
// ============================================================================

PhysicsPipeline::PhysicsPipeline()
    : m_MaxContacts(0)
    , m_Initialized(false)
{
}

PhysicsPipeline::~PhysicsPipeline()
{
}

void PhysicsPipeline::Initialize(size_t maxEntities, size_t maxContacts)
{
    m_MaxContacts = maxContacts;
    
    // Reserve memory for contacts and pairs
    m_BroadPhasePairs.reserve(maxEntities * 4); // Estimate 4 collisions per entity
    m_Contacts.reserve(maxContacts);
    m_FinalContacts.reserve(maxContacts);
    
    // Initialize subsystems
    m_StabilizationSystem.Initialize(maxContacts, maxEntities);
    
    m_Initialized = true;
}

// ============================================================================
// Main Update Loop
// ============================================================================

void PhysicsPipeline::Update(
    ECS::ComponentStore& componentStore,
    float deltaTime,
    const Math::Vector2& gravity,
    const Boundary& bounds)
{
    if (!m_Initialized)
        return;
    
    // Get active (non-sleeping) physics entities
    std::vector<ECS::EntityID> activeEntities;
    componentStore.ForEachComponent<ECS::PhysicsBodyComponent>(
        [&activeEntities](ECS::EntityID entityId, const ECS::PhysicsBodyComponent& body)
        {
            if (!body.isStatic && !body.isSleeping)
            {
                activeEntities.push_back(entityId);
            }
        });
    
    // Step 1: Broad-phase collision detection
    BroadPhase(componentStore, activeEntities);
    
    // Step 2: Narrow-phase SAT collision detection
    NarrowPhase(componentStore, deltaTime);
    
    // Step 3: Continuous collision detection for fast objects
    ContinuousCollisionDetection(componentStore, deltaTime);
    
    // Step 4: Boundary collision detection
    BoundaryCollision(componentStore, bounds, deltaTime);
    
    // Step 5: Solve velocity and position constraints
    SolveConstraints(componentStore, gravity, deltaTime);
    
    // Step 6: Apply stabilization and sleep management
    ApplyStabilization(componentStore, deltaTime);
}

// ============================================================================
// Step 1: Broad-Phase
// ============================================================================

class BroadPhaseQueryCallback
{
public:
    BroadPhaseQueryCallback(std::vector<BroadPhasePair>& pairs, const DynamicTree& tree, 
                           uint32_t currentIdA, uint32_t nodeIdA)
        : m_Pairs(pairs), m_Tree(tree), m_CurrentIdA(currentIdA), m_NodeIdA(nodeIdA) {}
    
    bool QueryCallback(uint32_t nodeId, uint32_t userData)
    {
        // Add overlap pair for each overlapping node
        if (userData <= m_CurrentIdA)
            return true; // Dedup - only add each pair once
        
        BroadPhasePair pair;
        pair.entityIdA = m_CurrentIdA;
        pair.entityIdB = userData;
        pair.shapeIdA = 0; // Default shape for now
        pair.shapeIdB = 0;
        pair.aabbA = m_Tree.GetFatAABB(m_NodeIdA);
        pair.aabbB = m_Tree.GetFatAABB(nodeId);
        
        m_Pairs.push_back(pair);
        return true; // Continue query
    }
    
    void AddOverlap(uint32_t nodeIdB, uint32_t userDataB)
    {
        // Avoid duplicate pairs and self-collision
        if (userDataB == m_CurrentIdA || userDataB < m_CurrentIdA)
            return;
        
        BroadPhasePair pair;
        pair.entityIdA = m_CurrentIdA;
        pair.entityIdB = userDataB;
        pair.shapeIdA = 0; // Default shape for now
        pair.shapeIdB = 0;
        pair.aabbA = m_Tree.GetFatAABB(m_NodeIdA);
        pair.aabbB = m_Tree.GetFatAABB(nodeIdB);
        
        m_Pairs.push_back(pair);
    }
    
private:
    std::vector<BroadPhasePair>& m_Pairs;
    const DynamicTree& m_Tree;
    uint32_t m_CurrentIdA = 0;
    uint32_t m_NodeIdA = 0;
};

void PhysicsPipeline::BroadPhase(
    ECS::ComponentStore& componentStore,
    const std::vector<ECS::EntityID>& activeEntities)
{
    m_BroadPhasePairs.clear();
    
    // Update dynamic tree with current AABBs
    UpdateDynamicTree(componentStore, activeEntities);
    
    // Rebuild tree if necessary (for better balancing)
    m_DynamicTree.Rebuild(false);
    
    // Query for overlapping pairs
    QueryOverlappingPairs();
}

void PhysicsPipeline::UpdateDynamicTree(
    ECS::ComponentStore& componentStore,
    const std::vector<ECS::EntityID>& activeEntities)
{
    // Update or create proxies for all active entities
    componentStore.ForEachComponent<ECS::ColliderComponent>(
        [this, &componentStore](ECS::EntityID entityId, ECS::ColliderComponent& collider)
        {
            if (!componentStore.HasComponent<ECS::TransformComponent>(entityId))
                return;
            
            auto& transform = componentStore.GetComponent<ECS::TransformComponent>(entityId);
            
            // Compute AABB for this collider
            AABB aabb;
            float margin = 0.1f;
            
            if (std::holds_alternative<ECS::ColliderComponent::CircleShape>(collider.shape))
            {
                const auto& circle = std::get<ECS::ColliderComponent::CircleShape>(collider.shape);
                float radius = circle.radius * transform.scale.x;
                
                aabb.lowerBound = transform.position - Math::Vector2{radius, radius};
                aabb.upperBound = transform.position + Math::Vector2{radius, radius};
            }
            else if (std::holds_alternative<ECS::ColliderComponent::PolygonShape>(collider.shape))
            {
                const auto& polygon = std::get<ECS::ColliderComponent::PolygonShape>(collider.shape);
                
                Math::Vector2 minVertex{std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
                Math::Vector2 maxVertex{std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};
                
                float cosA = std::cos(transform.rotation);
                float sinA = std::sin(transform.rotation);
                
                for (const auto& vertex : polygon.vertices)
                {
                    Math::Vector2 scaled = vertex * transform.scale;
                    Math::Vector2 rotated{
                        scaled.x * cosA - scaled.y * sinA,
                        scaled.x * sinA + scaled.y * cosA
                    };
                    Math::Vector2 worldVertex = transform.position + rotated;
                    
                    minVertex.x = std::min(minVertex.x, worldVertex.x);
                    minVertex.y = std::min(minVertex.y, worldVertex.y);
                    maxVertex.x = std::max(maxVertex.x, worldVertex.x);
                    maxVertex.y = std::max(maxVertex.y, worldVertex.y);
                }
                
                aabb.lowerBound = minVertex - Math::Vector2{margin, margin};
                aabb.upperBound = maxVertex + Math::Vector2{margin, margin};
            }
            
            // Update or create proxy in dynamic tree
            auto it = m_EntityProxyMap.find(entityId);
            if (it != m_EntityProxyMap.end())
            {
                Math::Vector2 displacement = transform.position - transform.previousPosition;
                m_DynamicTree.MoveProxy(it->second, aabb, displacement);
            }
            else
            {
                uint32_t proxyId = m_DynamicTree.CreateProxy(aabb, entityId);
                m_EntityProxyMap[entityId] = proxyId;
            }
        });
}

void PhysicsPipeline::QueryOverlappingPairs()
{
    // For each proxy, query the tree for overlaps
    for (const auto& [entityId, proxyId] : m_EntityProxyMap)
    {
        AABB proxyAABB = m_DynamicTree.GetFatAABB(proxyId);
        
        BroadPhaseQueryCallback callback(m_BroadPhasePairs, m_DynamicTree, entityId, proxyId);
        
        // Query the tree - this will find all overlapping AABBs
        m_DynamicTree.Query(proxyAABB, &callback);
    }
}

// ============================================================================
// Step 2: Narrow-Phase
// ============================================================================

void PhysicsPipeline::NarrowPhase(
    ECS::ComponentStore& componentStore,
    float deltaTime)
{
    m_Contacts.clear();
    
    for (auto& pair : m_BroadPhasePairs)
    {
        auto* bodyA = componentStore.GetComponent<ECS::PhysicsBodyComponent>(pair.entityIdA);
        auto* bodyB = componentStore.GetComponent<ECS::PhysicsBodyComponent>(pair.entityIdB);
        auto* colliderA = componentStore.GetComponent<ECS::ColliderComponent>(pair.entityIdA);
        auto* colliderB = componentStore.GetComponent<ECS::ColliderComponent>(pair.entityIdB);
        auto* transformA = componentStore.GetComponent<ECS::TransformComponent>(pair.entityIdA);
        auto* transformB = componentStore.GetComponent<ECS::TransformComponent>(pair.entityIdB);
        
        if (!bodyA || !bodyB || !colliderA || !colliderB || !transformA || !transformB)
            continue;
        
        // Determine collision detection strategy
        float minExtentA = 1.0f; // Approximate
        float minExtentB = 1.0f;
        
        bool needsCCD = NeedsCCDForPair(*bodyA, *bodyB, deltaTime, minExtentA, minExtentB);
        
        // Perform SAT collision detection based on shape types
        ContactManifold manifold;
        
        // Handle different shape combinations
        if (std::holds_alternative<ECS::ColliderComponent::CircleShape>(colliderA->shape) &&
            std::holds_alternative<ECS::ColliderComponent::CircleShape>(colliderB->shape))
        {
            const auto& circleA = std::get<ECS::ColliderComponent::CircleShape>(colliderA->shape);
            const auto& circleB = std::get<ECS::ColliderComponent::CircleShape>(colliderB->shape);
            
            manifold = m_SATDetector.DetectCircleCircle(
                pair.entityIdA, pair.entityIdB,
                pair.shapeIdA, pair.shapeIdB,
                circleA, circleB,
                *transformA, *transformB);
        }
        else if (std::holds_alternative<ECS::ColliderComponent::CircleShape>(colliderA->shape) &&
                 std::holds_alternative<ECS::ColliderComponent::PolygonShape>(colliderB->shape))
        {
            const auto& circle = std::get<ECS::ColliderComponent::CircleShape>(colliderA->shape);
            const auto& polygon = std::get<ECS::ColliderComponent::PolygonShape>(colliderB->shape);
            
            manifold = m_SATDetector.DetectCirclePolygon(
                pair.entityIdA, pair.entityIdB,
                pair.shapeIdA, pair.shapeIdB,
                circle, polygon,
                *transformA, *transformB);
        }
        else if (std::holds_alternative<ECS::ColliderComponent::PolygonShape>(colliderA->shape) &&
                 std::holds_alternative<ECS::ColliderComponent::PolygonShape>(colliderB->shape))
        {
            const auto& polygonA = std::get<ECS::ColliderComponent::PolygonShape>(colliderA->shape);
            const auto& polygonB = std::get<ECS::ColliderComponent::PolygonShape>(colliderB->shape);
            
            manifold = m_SATDetector.DetectPolygonPolygon(
                pair.entityIdA, pair.entityIdB,
                pair.shapeIdA, pair.shapeIdB,
                polygonA, polygonB,
                *transformA, *transformB);
        }
        // Add more shape combinations as needed
        
        if (manifold.touching && !manifold.points.empty())
        {
            NarrowPhaseContact contact;
            contact.manifold = manifold;
            contact.needsCCD = needsCCD;
            contact.toi = 1.0f; // Will be updated in CCD step if needed
            
            m_Contacts.push_back(contact);
        }
    }
}

bool PhysicsPipeline::NeedsCCDForPair(
    const ECS::PhysicsBodyComponent& bodyA,
    const ECS::PhysicsBodyComponent& bodyB,
    float deltaTime,
    float minExtentA,
    float minExtentB)
{
    // Check if either body is moving fast enough to require CCD
    float speedA = bodyA.velocity.Length();
    float speedB = bodyB.velocity.Length();
    
    float thresholdA = minExtentA / deltaTime * 0.5f;
    float thresholdB = minExtentB / deltaTime * 0.5f;
    
    return (speedA > thresholdA || speedB > thresholdB);
}

// ============================================================================
// Step 3: Continuous Collision Detection
// ============================================================================

void PhysicsPipeline::ContinuousCollisionDetection(
    ECS::ComponentStore& componentStore,
    float deltaTime)
{
    size_t ccdQueries = 0;
    
    for (auto& contact : m_Contacts)
    {
        if (!contact.needsCCD)
            continue;
        
        ccdQueries++;
        
        auto* bodyA = componentStore.GetComponent<ECS::PhysicsBodyComponent>(contact.manifold.entityIdA);
        auto* bodyB = componentStore.GetComponent<ECS::PhysicsBodyComponent>(contact.manifold.entityIdB);
        
        if (!bodyA || !bodyB)
            continue;
        
        // Use SAT's built-in CCD
        Math::Vector2 point, normal;
        float fraction;
        
        bool toiFound = SATCollisionDetector::CCD::ComputeTOI(
            contact.manifold,
            bodyA->velocity,
            bodyB->velocity,
            bodyA->angularVelocity,
            bodyB->angularVelocity,
            deltaTime,
            fraction,
            point,
            normal);
        
        if (toiFound && fraction < contact.toi)
        {
            contact.toi = fraction;
        }
    }
}

// ============================================================================
// Step 4: Boundary Collision
// ============================================================================

void PhysicsPipeline::BoundaryCollision(
    ECS::ComponentStore& componentStore,
    const Boundary& bounds,
    float deltaTime)
{
    if (!bounds.enabled)
        return;
    
    componentStore.ForEachComponent<ECS::ColliderComponent>(
        [this, &componentStore, &bounds, deltaTime](ECS::EntityID entityId, ECS::ColliderComponent& collider)
        {
            auto* body = componentStore.GetComponent<ECS::PhysicsBodyComponent>(entityId);
            auto* transform = componentStore.GetComponent<ECS::TransformComponent>(entityId);
            
            if (!body || !transform || body->isStatic)
                return;
            
            BoundaryCollisionResult boundaryCollision;
            
            // Detect boundary collision based on shape type
            if (std::holds_alternative<ECS::ColliderComponent::CircleShape>(collider.shape))
            {
                const auto& circle = std::get<ECS::ColliderComponent::CircleShape>(collider.shape);
                float radius = circle.radius * transform->scale.x;
                
                boundaryCollision = m_BoundarySystem.DetectCircleCollision(
                    transform->position,
                    radius,
                    body->velocity,
                    deltaTime);
            }
            // Add polygon and capsule boundary detection
            
            if (boundaryCollision.colliding && !boundaryCollision.contacts.empty())
            {
                // Create contact manifold from boundary collision
                ContactManifold manifold;
                manifold.entityIdA = entityId;
                manifold.entityIdB = UINT32_MAX; // Boundary has no entity
                manifold.shapeIdA = 0;
                manifold.shapeIdB = UINT32_MAX;
                manifold.touching = true;
                manifold.normal = boundaryCollision.contacts[0].normal;
                
                for (const auto& contact : boundaryCollision.contacts)
                {
                    ContactPoint cp;
                    cp.position = contact.contactPoint;
                    cp.penetrationDepth = contact.penetrationDepth;
                    manifold.points.push_back(cp);
                }
                
                m_FinalContacts.push_back(manifold);
            }
        });
}

// ============================================================================
// Step 5: Constraint Solving
// ============================================================================

std::vector<SolverBody> PhysicsPipeline::CreateSolverBodies(
    ECS::ComponentStore& componentStore,
    const std::vector<ContactManifold>& contacts,
    const Math::Vector2& gravity,
    float deltaTime)
{
    // Collect all unique bodies in contacts
    std::unordered_set<uint32_t> uniqueBodies;
    for (const auto& contact : contacts)
    {
        uniqueBodies.insert(contact.entityIdA);
        uniqueBodies.insert(contact.entityIdB);
    }
    
    std::vector<SolverBody> solverBodies;
    solverBodies.reserve(uniqueBodies.size());
    
    for (uint32_t entityId : uniqueBodies)
    {
        auto* body = componentStore.GetComponent<ECS::PhysicsBodyComponent>(entityId);
        auto* transform = componentStore.GetComponent<ECS::TransformComponent>(entityId);
        
        if (!body || !transform)
            continue;
        
        SolverBody solverBody;
        solverBody.index = static_cast<uint32_t>(solverBodies.size());
        solverBody.entityId = entityId;
        solverBody.invMass = body->inverseMass;
        solverBody.invInertia = body->inverseInertia;
        solverBody.linearVelocity = body->velocity;
        solverBody.angularVelocity = body->angularVelocity;
        solverBody.position = transform->position;
        solverBody.angle = transform->rotation;
        solverBody.rotation = Math::Rotation2D(solverBody.angle);
        solverBody.gravity = gravity;
        
        solverBodies.push_back(solverBody);
    }
    
    return solverBodies;
}

void PhysicsPipeline::SolveConstraints(
    ECS::ComponentStore& componentStore,
    const Math::Vector2& gravity,
    float deltaTime,
    int velocityIterations,
    int positionIterations)
{
    // Filter contacts by TOI (only process contacts that happened this frame)
    std::vector<ContactManifold> validContacts;
    for (const auto& contact : m_Contacts)
    {
        if (contact.toi <= 1.0f && contact.manifold.touching)
        {
            validContacts.push_back(contact.manifold);
        }
    }
    
    if (validContacts.empty())
        return;
    
    // Only insert valid contacts into m_FinalContacts (not all contacts)
    m_FinalContacts.insert(m_FinalContacts.end(), validContacts.begin(), validContacts.end());
    
    // Create solver bodies
    std::vector<SolverBody> solverBodies = CreateSolverBodies(
        componentStore, validContacts, gravity, deltaTime);
    
    // Initialize constraint solver
    m_ConstraintSolver.Initialize(
        validContacts,
        solverBodies,
        deltaTime,
        velocityIterations,
        positionIterations);
    
    m_ConstraintSolver.SetGravity(gravity);
    m_ConstraintSolver.SetWarmStarting(true);
    m_ConstraintSolver.SetSplitImpulses(true);
    
    // Apply warm starting from persistent contacts
    // Note: UpdatePersistentContacts now takes ComponentStore reference directly
    auto persistentContacts = m_StabilizationSystem.UpdatePersistentContacts(
        validContacts,
        componentStore);
    
    // Solve velocity constraints
    m_ConstraintSolver.SolveVelocityConstraints();
    
    // Integrate velocities
    m_ConstraintSolver.IntegrateVelocities(deltaTime);
    
    // Solve position constraints
    m_ConstraintSolver.SolvePositionConstraints();
    
    // Write back to components
    WriteBackToComponents(componentStore, solverBodies);
}

void PhysicsPipeline::WriteBackToComponents(
    ECS::ComponentStore& componentStore,
    const std::vector<SolverBody>& solverBodies)
{
    for (const auto& solverBody : solverBodies)
    {
        auto* body = componentStore.GetComponent<ECS::PhysicsBodyComponent>(solverBody.entityId);
        auto* transform = componentStore.GetComponent<ECS::TransformComponent>(solverBody.entityId);
        
        if (!body || !transform)
            continue;
        
        body->velocity = solverBody.linearVelocity;
        body->angularVelocity = solverBody.angularVelocity;
        transform->position = solverBody.position;
        transform->rotation = solverBody.angle;  // Just assign the float value directly
    }
}

// ============================================================================
// Step 6: Stabilization
// ============================================================================

void PhysicsPipeline::ApplyStabilization(
    ECS::ComponentStore& componentStore,
    float deltaTime)
{
    // Update sleep management
    auto& sleepManager = m_StabilizationSystem.GetSleepManager();
    
    componentStore.ForEachComponent<ECS::PhysicsBodyComponent>(
        [&sleepManager, deltaTime](ECS::EntityID entityId, ECS::PhysicsBodyComponent& body)
        {
            if (body.isStatic)
                return;
            
            float linearSpeed = body.velocity.Length();
            float angularSpeed = std::abs(body.angularVelocity);
            
            sleepManager.UpdateSleepTime(entityId, linearSpeed, angularSpeed, deltaTime);
            
            // Put body to sleep if conditions are met
            if (sleepManager.ShouldSleep(entityId))
            {
                body.isSleeping = true;
                body.velocity = Math::Vector2{0.0f, 0.0f};
                body.angularVelocity = 0.0f;
            }
        });
    
    // Wake up bodies on contact
    for (const auto& contact : m_FinalContacts)
    {
        auto* bodyA = componentStore.GetComponent<ECS::PhysicsBodyComponent>(contact.entityIdA);
        auto* bodyB = componentStore.GetComponent<ECS::PhysicsBodyComponent>(contact.entityIdB);
        
        if (bodyA && !bodyA->isStatic && bodyA->isSleeping)
        {
            sleepManager.WakeBody(contact.entityIdA);
            bodyA->isSleeping = false;
        }
        
        if (bodyB && contact.entityIdB != UINT32_MAX && !bodyB->isStatic && bodyB->isSleeping)
        {
            sleepManager.WakeBody(contact.entityIdB);
            bodyB->isSleeping = false;
        }
    }
}

// ============================================================================
// Statistics
// ============================================================================

PhysicsPipeline::Statistics PhysicsPipeline::GetStatistics() const
{
    Statistics stats;
    stats.broadPhasePairs = m_BroadPhasePairs.size();
    stats.narrowPhaseContacts = m_Contacts.size();
    stats.boundaryContacts = m_FinalContacts.size();
    stats.persistentContacts = m_StabilizationSystem.GetActivePersistentContactCount();
    stats.sleepingBodies = m_StabilizationSystem.GetSleepingBodyCount();
    
    return stats;
}

} // namespace Nyon
