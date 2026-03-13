// SPDX-FileCopyrightText: 2026 Nyon Engine
// SPDX-License-Identifier: MIT

#pragma once

#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"

using namespace Nyon;
using namespace Nyon::ECS;

/**
 * @brief Minimal physics test - one box falling on one wall
 */
class SimplePhysicsTest : public Nyon::ECSApplication
{
public:
    SimplePhysicsTest()
        : ECSApplication("Simple Physics Test - One Box, One Wall", 1280, 720)
    {
    }

    void OnECSStart() override
    {
        CreatePhysicsWorld();
        CreateFloor();
        CreateFallingBox();
    }

private:
    void CreatePhysicsWorld()
    {
        auto& entities = GetEntityManager();
        auto& components = GetComponentStore();
        
        Nyon::ECS::EntityID physicsWorld = entities.CreateEntity();
        
        Nyon::ECS::PhysicsWorldComponent world;
        world.gravity = {0.0f, 981.0f};  // Earth gravity (pixels/s^2) - POSITIVE because Y+ is DOWN in screen coords
        world.timeStep = 1.0f / 60.0f;     // 60 Hz
        
        components.AddComponent(physicsWorld, std::move(world));
    }

    void CreateFloor()
    {
        auto& entities = GetEntityManager();
        auto& components = GetComponentStore();

        // Static floor at bottom of screen
        Nyon::ECS::EntityID floor = entities.CreateEntity();

        Nyon::ECS::TransformComponent t;
        t.position = {640.0f, 680.0f};  // Center-bottom

        Nyon::ECS::PhysicsBodyComponent body;
        body.isStatic = true;
        body.UpdateMassProperties();

        // Floor: 800 wide, 40 thick
        Nyon::ECS::ColliderComponent::PolygonShape shape({
            {-400.0f, -20.0f},
            { 400.0f, -20.0f},
            { 400.0f,  20.0f},
            {-400.0f,  20.0f}
        });

        Nyon::ECS::ColliderComponent collider(shape);
        collider.material.friction = 0.5f;
        collider.material.restitution = 0.1f;

        Nyon::ECS::RenderComponent render(
            {800.0f, 40.0f},
            {0.3f, 0.3f, 0.3f}  // Dark gray
        );

        components.AddComponent(floor, std::move(t));
        components.AddComponent(floor, std::move(body));
        components.AddComponent(floor, std::move(collider));
        components.AddComponent(floor, std::move(render));
    }

    void CreateFallingBox()
    {
        auto& entities = GetEntityManager();
        auto& components = GetComponentStore();

        // Dynamic box that falls from top
        Nyon::ECS::EntityID box = entities.CreateEntity();

        Nyon::ECS::TransformComponent t;
        t.position = {640.0f, 200.0f};  // Start above the floor

        Nyon::ECS::PhysicsBodyComponent body;
        body.mass = 1.0f;
        body.UpdateMassProperties();

        // Box: 50x50
        Nyon::ECS::ColliderComponent::PolygonShape shape({
            {-25.0f, -25.0f},
            { 25.0f, -25.0f},
            { 25.0f,  25.0f},
            {-25.0f,  25.0f}
        });

        Nyon::ECS::ColliderComponent collider(shape);
        collider.material.friction = 0.3f;
        collider.material.restitution = 0.5f;  // Bouncy

        Nyon::ECS::RenderComponent render(
            {50.0f, 50.0f},
            {1.0f, 0.5f, 0.0f},  // Orange
            Nyon::ECS::RenderComponent::ShapeType::Rectangle
        );

        components.AddComponent(box, std::move(t));
        components.AddComponent(box, std::move(body));
        components.AddComponent(box, std::move(collider));
        components.AddComponent(box, std::move(render));
    }
};
