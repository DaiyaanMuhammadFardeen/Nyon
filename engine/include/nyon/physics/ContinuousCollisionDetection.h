// SPDX-FileCopyrightText: 2026 Nyon Engine
// SPDX-License-Identifier: MIT

#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/physics/SATCollisionDetector.h"
#include <vector>

namespace Nyon::Physics
{
    /**
     * @brief Time of Impact (TOI) result for continuous collision detection
     */
    struct TOIResult
    {
        bool hit = false;
        float fraction;           // Time of impact as fraction of dt [0,1]
        Math::Vector2 point;      // Contact point at TOI
        Math::Vector2 normal;     // Contact normal at TOI
        uint32_t entityIdA;
        uint32_t entityIdB;
    };
    
    /**
     * @brief Advanced Continuous Collision Detection system
     * 
     * Prevents tunneling of fast-moving objects using:
     * - Conservative advancement
     * - Swept shape tests
     * - Sub-stepping when necessary
     * 
     * Supports all shape types: circles, polygons, capsules
     */
    class ContinuousCollisionDetection
    {
    public:
        ContinuousCollisionDetection() = default;
        ~ContinuousCollisionDetection() = default;
        
        /**
         * @brief Perform CCD between two moving circles
         * 
         * Analytical solution for circle-circle TOI.
         * 
         * @param centerA Start position of circle A
         * @param radiusA Radius of circle A
         * @param velocityA Velocity of circle A
         * @param centerB Start position of circle B
         * @param radiusB Radius of circle B
         * @param velocityB Velocity of circle B
         * @param dt Time step
         * @return TOIResult with time of impact
         */
        static TOIResult CircleCircleCCD(
            const Math::Vector2& centerA,
            float radiusA,
            const Math::Vector2& velocityA,
            const Math::Vector2& centerB,
            float radiusB,
            const Math::Vector2& velocityB,
            float dt);
        
        /**
         * @brief Perform CCD between circle and polygon
         * 
         * Uses conservative advancement with swept circle test.
         * 
         * @param circleCenter Start position of circle
         * @param circleRadius Radius of circle
         * @param circleVelocity Velocity of circle
         * @param polygonVertices Polygon vertices in world space
         * @param polygonVelocity Velocity of polygon
         * @param polygonAngularVelocity Angular velocity of polygon
         * @param dt Time step
         * @return TOIResult with time of impact
         */
        static TOIResult CirclePolygonCCD(
            const Math::Vector2& circleCenter,
            float circleRadius,
            const Math::Vector2& circleVelocity,
            const std::vector<Math::Vector2>& polygonVertices,
            const Math::Vector2& polygonVelocity,
            float polygonAngularVelocity,
            float dt);
        
        /**
         * @brief Perform CCD between two polygons
         * 
         * Uses GJK-based conservative advancement.
         * 
         * @param vertsA Polygon A vertices in world space
         * @param velocityA Velocity of polygon A
         * @param angularVelA Angular velocity of polygon A
         * @param vertsB Polygon B vertices in world space
         * @param velocityB Velocity of polygon B
         * @param angularVelB Angular velocity of polygon B
         * @param dt Time step
         * @return TOIResult with time of impact
         */
        static TOIResult PolygonPolygonCCD(
            const std::vector<Math::Vector2>& vertsA,
            const Math::Vector2& velocityA,
            float angularVelA,
            const std::vector<Math::Vector2>& vertsB,
            const Math::Vector2& velocityB,
            float angularVelB,
            float dt);
        
        /**
         * @brief Perform CCD between capsule and other shape
         * 
         * Treats capsule as swept circle segment.
         * 
         * @param capStart Start of capsule segment
         * @param capEnd End of capsule segment
         * @param capRadius Radius of capsule
         * @param velocity Capsule velocity
         * @param otherType Type of other shape (0=circle, 1=polygon, 2=capsule)
         * @param otherData Other shape data
         * @param otherVelocity Other shape velocity
         * @param dt Time step
         * @return TOIResult with time of impact
         */
        static TOIResult CapsuleCCD(
            const Math::Vector2& capStart,
            const Math::Vector2& capEnd,
            float capRadius,
            const Math::Vector2& velocity,
            int otherType,
            const void* otherData,
            const Math::Vector2& otherVelocity,
            float dt);
        
        /**
         * @brief Compute conservative advancement distance
         * 
         * Safe upper bound on motion before collision.
         * 
         * @param distance Current distance between shapes
         * @param relativeSpeed Relative approach speed
         * @param maxMotion Maximum allowed motion per step
         * @return Safe time fraction to advance
         */
        static float ConservativeAdvancement(
            float distance,
            float relativeSpeed,
            float maxMotion = 0.2f);
        
        /**
         * @brief Check if CCD is needed for a body
         * 
         * Based on velocity and size ratio.
         * 
         * @param velocity Linear velocity
         * @param angularVelocity Angular velocity
         * @param minExtent Minimum shape extent
         * @param dt Time step
         * @param threshold Threshold ratio (default 0.5)
         * @return true if CCD should be used
         */
        static bool NeedsCCD(
            const Math::Vector2& velocity,
            float angularVelocity,
            float minExtent,
            float dt,
            float threshold = 0.5f);
        
    private:
        /**
         * @brief Solve quadratic equation for circle-circle TOI
         */
        static bool SolveQuadratic(float a, float b, float c, float& t);
        
        /**
         * @brief Find closest points between two polygons
         */
        static Math::Vector2 ClosestPointOnPolygon(
            const Math::Vector2& point,
            const std::vector<Math::Vector2>& vertices);
    };
}
