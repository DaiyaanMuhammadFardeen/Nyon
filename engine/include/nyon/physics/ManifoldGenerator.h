#pragma once
#include "nyon/math/Vector2.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
namespace Nyon::Physics {
    class ManifoldGenerator {
    public:
        static ECS::ContactManifold GenerateManifold(uint32_t entityIdA,
                                                     uint32_t entityIdB,
                                                     uint32_t shapeIdA,
                                                     uint32_t shapeIdB,
                                                     const Nyon::ECS::ColliderComponent& colliderA,
                                                     const Nyon::ECS::ColliderComponent& colliderB,
                                                     const Nyon::ECS::TransformComponent& transformA,
                                                     const Nyon::ECS::TransformComponent& transformB);
    private:
        static ECS::ContactManifold CircleCircle(uint32_t entityIdA,
                                                 uint32_t entityIdB,
                                                 uint32_t shapeIdA,
                                                 uint32_t shapeIdB,
                                                 const Nyon::ECS::ColliderComponent::CircleShape& circleA,
                                                 const Nyon::ECS::ColliderComponent::CircleShape& circleB,
                                                 const Nyon::ECS::TransformComponent& transformA,
                                                 const Nyon::ECS::TransformComponent& transformB,
                                                 ECS::ContactManifold& manifold);
        static ECS::ContactManifold CirclePolygon(uint32_t entityIdA,
                                                  uint32_t entityIdB,
                                                  uint32_t shapeIdA,
                                                  uint32_t shapeIdB,
                                                  const Nyon::ECS::ColliderComponent::CircleShape& circle,
                                                  const Nyon::ECS::ColliderComponent::PolygonShape& polygon,
                                                  const Nyon::ECS::TransformComponent& circleTransform,
                                                  const Nyon::ECS::TransformComponent& polyTransform,
                                                  bool flipNormal,
                                                  ECS::ContactManifold& manifold);
        static ECS::ContactManifold PolygonPolygon(uint32_t entityIdA,
                                                   uint32_t entityIdB,
                                                   uint32_t shapeIdA,
                                                   uint32_t shapeIdB,
                                                   const Nyon::ECS::ColliderComponent::PolygonShape& polyA,
                                                   const Nyon::ECS::ColliderComponent::PolygonShape& polyB,
                                                   const Nyon::ECS::TransformComponent& transformA,
                                                   const Nyon::ECS::TransformComponent& transformB,
                                                   ECS::ContactManifold& manifold);
        static ECS::ContactManifold CircleCapsule(uint32_t entityIdA,
                                                  uint32_t entityIdB,
                                                  uint32_t shapeIdA,
                                                  uint32_t shapeIdB,
                                                  const Nyon::ECS::ColliderComponent& circleCollider,
                                                  const Nyon::ECS::ColliderComponent& capsuleCollider,
                                                  const Nyon::ECS::TransformComponent& transformA,
                                                  const Nyon::ECS::TransformComponent& transformB,
                                                  ECS::ContactManifold& manifold);
        static ECS::ContactManifold PolygonCapsule(uint32_t entityIdA,
                                                   uint32_t entityIdB,
                                                   uint32_t shapeIdA,
                                                   uint32_t shapeIdB,
                                                   const Nyon::ECS::ColliderComponent& polyCollider,
                                                   const Nyon::ECS::ColliderComponent& capsuleCollider,
                                                   const Nyon::ECS::TransformComponent& transformA,
                                                   const Nyon::ECS::TransformComponent& transformB,
                                                   ECS::ContactManifold& manifold);
        static ECS::ContactManifold CapsuleCapsule(uint32_t entityIdA,
                                                   uint32_t entityIdB,
                                                   uint32_t shapeIdA,
                                                   uint32_t shapeIdB,
                                                   const Nyon::ECS::ColliderComponent& colliderA,
                                                   const Nyon::ECS::ColliderComponent& colliderB,
                                                   const Nyon::ECS::TransformComponent& transformA,
                                                   const Nyon::ECS::TransformComponent& transformB,
                                                   ECS::ContactManifold& manifold);
        static ECS::ContactManifold CapsuleCollision(uint32_t entityIdA,
                                                     uint32_t entityIdB,
                                                     uint32_t shapeIdA,
                                                     uint32_t shapeIdB,
                                                     const Nyon::ECS::ColliderComponent& colliderA,
                                                     const Nyon::ECS::ColliderComponent& colliderB,
                                                     const Nyon::ECS::TransformComponent& transformA,
                                                     const Nyon::ECS::TransformComponent& transformB,
                                                     ECS::ContactManifold& manifold);
        static ECS::ContactManifold SegmentCollision(uint32_t entityIdA,
                                                     uint32_t entityIdB,
                                                     uint32_t shapeIdA,
                                                     uint32_t shapeIdB,
                                                     const Nyon::ECS::ColliderComponent& colliderA,
                                                     const Nyon::ECS::ColliderComponent& colliderB,
                                                     const Nyon::ECS::TransformComponent& transformA,
                                                     const Nyon::ECS::TransformComponent& transformB,
                                                     ECS::ContactManifold& manifold); }; }
