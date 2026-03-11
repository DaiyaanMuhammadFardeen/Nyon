#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/TransformComponent.h"

namespace Nyon::Physics
{
    /**
     * @brief Utility responsible for generating contact manifolds between pairs of shapes.
     * 
     * Supports circle–circle, circle–polygon, and polygon–polygon using SAT-style tests,
     * leveraging the polygon face normals stored on ColliderComponent::PolygonShape.
     */
    class ManifoldGenerator
    {
    public:
        static ECS::ContactManifold GenerateManifold(uint32_t entityIdA,
                                                     uint32_t entityIdB,
                                                     uint32_t shapeIdA,
                                                     uint32_t shapeIdB,
                                                     const ECS::ColliderComponent& colliderA,
                                                     const ECS::ColliderComponent& colliderB,
                                                     const ECS::TransformComponent& transformA,
                                                     const ECS::TransformComponent& transformB);

    private:
        static ECS::ContactManifold CircleCircle(uint32_t entityIdA,
                                                 uint32_t entityIdB,
                                                 uint32_t shapeIdA,
                                                 uint32_t shapeIdB,
                                                 const ECS::ColliderComponent::CircleShape& circleA,
                                                 const ECS::ColliderComponent::CircleShape& circleB,
                                                 const ECS::TransformComponent& transformA,
                                                 const ECS::TransformComponent& transformB,
                                                 ECS::ContactManifold& manifold);

        static ECS::ContactManifold CirclePolygon(uint32_t entityIdA,
                                                  uint32_t entityIdB,
                                                  uint32_t shapeIdA,
                                                  uint32_t shapeIdB,
                                                  const ECS::ColliderComponent::CircleShape& circle,
                                                  const ECS::ColliderComponent::PolygonShape& polygon,
                                                  const ECS::TransformComponent& circleTransform,
                                                  const ECS::TransformComponent& polyTransform,
                                                  bool flipNormal,
                                                  ECS::ContactManifold& manifold);

        static ECS::ContactManifold PolygonPolygon(uint32_t entityIdA,
                                                   uint32_t entityIdB,
                                                   uint32_t shapeIdA,
                                                   uint32_t shapeIdB,
                                                   const ECS::ColliderComponent::PolygonShape& polyA,
                                                   const ECS::ColliderComponent::PolygonShape& polyB,
                                                   const ECS::TransformComponent& transformA,
                                                   const ECS::TransformComponent& transformB,
                                                   ECS::ContactManifold& manifold);
        
        static ECS::ContactManifold CapsuleCollision(uint32_t entityIdA,
                                                     uint32_t entityIdB,
                                                     uint32_t shapeIdA,
                                                     uint32_t shapeIdB,
                                                     const ECS::ColliderComponent& colliderA,
                                                     const ECS::ColliderComponent& colliderB,
                                                     const ECS::TransformComponent& transformA,
                                                     const ECS::TransformComponent& transformB,
                                                     ECS::ContactManifold& manifold);
        
        static ECS::ContactManifold SegmentCollision(uint32_t entityIdA,
                                                     uint32_t entityIdB,
                                                     uint32_t shapeIdA,
                                                     uint32_t shapeIdB,
                                                     const ECS::ColliderComponent& colliderA,
                                                     const ECS::ColliderComponent& colliderB,
                                                     const ECS::TransformComponent& transformA,
                                                     const ECS::TransformComponent& transformB,
                                                     ECS::ContactManifold& manifold);
    };
}

