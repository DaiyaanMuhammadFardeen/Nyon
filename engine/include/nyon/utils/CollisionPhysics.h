#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/utils/Physics.h"
#include <vector>
#include <utility>

namespace Nyon::Utils
{
    /**
     * @brief CollisionPhysics class providing collision detection utilities.
     *
     * Uses a y-positive-down coordinate system with pixels as units. Polygons must be convex.
     *
     * Supports three collision detection methods:
     * - Discrete SAT: Fast, for normal-speed objects
     * - Continuous CCD: Prevents tunneling for fast-moving objects
     * - Raycast: Perfect for projectiles and line-of-sight
     */
    class CollisionPhysics
    {
    public:
        /// Define a polygon as a collection of vertices in local space
        using Polygon = std::vector<Math::Vector2>;

        /**
         * @brief Result of collision detection with MTV (Minimum Translation Vector).
         *
         * Contains information needed to resolve a collision.
         */
        struct CollisionResult {
            bool collided;              ///< True if collision occurred
            Math::Vector2 overlapAxis;  ///< Direction vector to push object to resolve collision
            float overlapAmount;        ///< Distance to push object to resolve collision

            CollisionResult() : collided(false), overlapAxis(0.0f, 0.0f), overlapAmount(0.0f) {}
            CollisionResult(bool coll, const Math::Vector2& axis, float amount)
                : collided(coll), overlapAxis(axis), overlapAmount(amount) {}
        };

        /**
         * @brief Result of Continuous Collision Detection (CCD).
         *
         * Contains the time of impact and collision information along a movement path.
         */
        struct CCDResult
        {
            bool collided;                  ///< Did a collision occur along the path?
            float timeOfImpact;             ///< Time of impact (0.0 to 1.0, where 1.0 is end of movement)
            Math::Vector2 impactPosition;   ///< Position at time of impact (safe position before collision)
            CollisionResult collision;      ///< Collision data at impact time

            CCDResult() : collided(false), timeOfImpact(1.0f), impactPosition(0.0f, 0.0f),
                         collision(false, Math::Vector2(0.0f, 0.0f), 0.0f) {}
            CCDResult(bool c, float toi, const Math::Vector2& pos, const CollisionResult& col)
                : collided(c), timeOfImpact(toi), impactPosition(pos), collision(col) {}
        };

        /**
         * @brief Result of a raycast operation.
         *
         * Contains information about where and if a ray hit a polygon.
         */
        struct RaycastResult
        {
            bool hit;                       ///< Did the ray hit the polygon?
            Math::Vector2 hitPoint;         ///< Point of intersection in world space
            Math::Vector2 hitNormal;        ///< Surface normal at hit point
            float hitDistance;              ///< Distance along ray (0.0 to 1.0)

            RaycastResult() : hit(false), hitPoint(0.0f, 0.0f), hitNormal(0.0f, 0.0f), hitDistance(1.0f) {}
            RaycastResult(bool h, const Math::Vector2& point, const Math::Vector2& normal, float dist)
                : hit(h), hitPoint(point), hitNormal(normal), hitDistance(dist) {}
        };

        // ============================================================================
        // LEGACY API - Maintains backward compatibility
        // ============================================================================

        /**
         * @brief Checks collision between two rectangular bodies using AABB collision detection.
         *
         * @param body1 First body
         * @param size1 Size of first body
         * @param body2 Second body
         * @param size2 Size of second body
         * @return True if bodies are colliding
         */
        static bool CheckCollision(const Physics::Body& body1, const Math::Vector2& size1,
                                  const Physics::Body& body2, const Math::Vector2& size2);

        /**
         * @brief Broad-phase collision check using Axis-Aligned Bounding Boxes (AABB).
         *
         * @param pos1 Position of first box
         * @param size1 Size of first box
         * @param pos2 Position of second box
         * @param size2 Size of second box
         * @return True if AABBs are overlapping
         */
        static bool CheckAABBCollision(const Math::Vector2& pos1, const Math::Vector2& size1,
                                      const Math::Vector2& pos2, const Math::Vector2& size2);

        // ============================================================================
        // DISCRETE COLLISION DETECTION (SAT-based)
        // ============================================================================

        /**
         * @brief SAT-based collision detection between two convex polygons.
         *
         * Uses Separating Axis Theorem (SAT) to detect collisions.
         * Returns the Minimum Translation Vector (MTV) to resolve collision.
         * Polygons must be convex and wound consistently.
         *
         * Use this for normal-speed objects (moving less than 50% of their size per frame).
         * For fast-moving objects, use ContinuousCollisionCheck to prevent tunneling.
         *
         * @param poly1 First polygon (vertices in local space)
         * @param pos1 World position of first polygon
         * @param poly2 Second polygon (vertices in local space)
         * @param pos2 World position of second polygon
         * @return CollisionResult with collision information and MTV
         */
        static CollisionResult CheckPolygonCollision(const Polygon& poly1, const Math::Vector2& pos1,
                                                     const Polygon& poly2, const Math::Vector2& pos2);

        // ============================================================================
        // CONTINUOUS COLLISION DETECTION (CCD) - For fast-moving objects
        // ============================================================================

        /**
         * @brief Continuous collision detection between two moving polygons.
         *
         * Uses conservative advancement with binary search to find the Time of Impact (TOI).
         * Prevents tunneling for fast-moving objects by checking the entire movement path.
         *
         * Use this when objects move more than 50% of their size per frame, such as:
         * - Player dashing
         * - Falling at high speed
         * - Fast-moving enemies
         *
         * @param poly1 First polygon (vertices in local space)
         * @param startPos1 Starting world position of first polygon
         * @param endPos1 Ending world position of first polygon
         * @param poly2 Second polygon (vertices in local space)
         * @param startPos2 Starting world position of second polygon
         * @param endPos2 Ending world position of second polygon
         * @param maxIterations Number of binary search iterations (default: 16, higher = more accurate)
         * @return CCDResult containing time of impact and collision data
         */
        static CCDResult ContinuousCollisionCheck(
            const Polygon& poly1, const Math::Vector2& startPos1, const Math::Vector2& endPos1,
            const Polygon& poly2, const Math::Vector2& startPos2, const Math::Vector2& endPos2,
            int maxIterations = 16);

        /**
         * @brief Optimized CCD for moving object vs static object.
         *
         * This is the most common case in platformers (player vs walls, enemies vs terrain).
         * More efficient than the general two-moving-objects version.
         *
         * @param movingPoly Polygon of the moving object (vertices in local space)
         * @param startPos Starting world position of moving object
         * @param endPos Ending world position of moving object
         * @param staticPoly Polygon of the static object (vertices in local space)
         * @param staticPos World position of static object
         * @param maxIterations Number of binary search iterations (default: 16)
         * @return CCDResult containing time of impact and collision data
         */
        static CCDResult ContinuousCollisionCheckMovingVsStatic(
            const Polygon& movingPoly, const Math::Vector2& startPos, const Math::Vector2& endPos,
            const Polygon& staticPoly, const Math::Vector2& staticPos,
            int maxIterations = 16);

        // ============================================================================
        // RAYCAST - For projectiles and line-of-sight
        // ============================================================================

        /**
         * @brief Cast a ray and find the first collision with a polygon.
         *
         * Perfect for:
         * - Bullets and projectiles
         * - Laser beams
         * - Line-of-sight checks
         * - Ground detection
         *
         * More efficient than CCD for point-like or extremely fast objects.
         *
         * @param rayStart Starting point of the ray in world space
         * @param rayEnd Ending point of the ray in world space
         * @param polygon Target polygon (vertices in local space)
         * @param polyPos World position of the polygon
         * @return RaycastResult containing hit information
         */
        static RaycastResult RaycastPolygon(
            const Math::Vector2& rayStart, const Math::Vector2& rayEnd,
            const Polygon& polygon, const Math::Vector2& polyPos);

        // ============================================================================
        // COLLISION RESOLUTION
        // ============================================================================

        /**
         * @brief Resolve collision between two bodies using the MTV from collision detection.
         *
         * Moves bodies apart according to their masses. Non-static bodies are moved, static
         * bodies remain stationary. The resolution accounts for mass ratios.
         *
         * Use this with discrete collision detection results.
         *
         * @param body1 First body involved in collision
         * @param body2 Second body involved in collision
         * @param result Collision result containing MTV from collision detection
         */
        static void ResolveCollision(Physics::Body& body1, Physics::Body& body2,
                                    const CollisionResult& result);

        /**
         * @brief Resolve CCD collision for a single body.
         *
         * Moves the body to its safe impact position and adjusts velocity to prevent
         * further penetration. Use this when resolving collisions against static geometry.
         *
         * @param body The body to resolve
         * @param ccdResult Result from CCD containing impact position and collision data
         * @param deltaTime Time step used in physics simulation
         */
        static void ResolveCCDCollision(Physics::Body& body, const CCDResult& ccdResult,
                                       float deltaTime);

        /**
         * @brief Determines if a body is grounded based on collision with surfaces below it.
         *
         * A body is considered grounded if there's a collision with a surface whose normal
         * has a significant y-component (pointing up). Threshold controls sensitivity.
         *
         * Works with both discrete and CCD collision results.
         *
         * @param body The body to check for grounded state
         * @param collisionResult Result from collision detection
         * @param threshold Minimum y-component of normal to consider as ground (default 0.7)
         * @return True if the body is in contact with a surface below it
         */
        static bool IsBodyGrounded(const Physics::Body& body, const CollisionResult& collisionResult,
                                  float threshold = 0.7f);

        // ============================================================================
        // HELPER FUNCTIONS
        // ============================================================================

        /**
         * @brief Calculate Axis-Aligned Bounding Box for a polygon.
         *
         * @param polygon Polygon vertices in local space
         * @param pos World position of the polygon
         * @param outMin Output: minimum corner of AABB
         * @param outMax Output: maximum corner of AABB
         */
        static void CalculateAABB(const Polygon& polygon, const Math::Vector2& pos,
                                 Math::Vector2& outMin, Math::Vector2& outMax);

        /**
         * @brief Calculate swept AABB covering movement from start to end position.
         *
         * Creates an AABB that encompasses the entire movement path. Useful for
         * broad-phase culling in CCD.
         *
         * @param polygon Polygon vertices in local space
         * @param startPos Starting world position
         * @param endPos Ending world position
         * @param outMin Output: minimum corner of swept AABB
         * @param outMax Output: maximum corner of swept AABB
         */
        static void CalculateSweptAABB(const Polygon& polygon, const Math::Vector2& startPos,
                                      const Math::Vector2& endPos,
                                      Math::Vector2& outMin, Math::Vector2& outMax);

    private:
        // SAT algorithm helpers
        static Math::Vector2 GetEdgeNormal(const Math::Vector2& edge);
        static float DotProduct(const Math::Vector2& a, const Math::Vector2& b);
        static std::pair<float, float> ProjectPolygonOntoAxis(const Polygon& polygon,
                                                             const Math::Vector2& pos,
                                                             const Math::Vector2& axis);
        static bool CheckOverlap(float min1, float max1, float min2, float max2);
        static float CalculateIntervalOverlap(float min1, float max1, float min2, float max2);
        static Math::Vector2 CalculatePolygonCenter(const Polygon& polygon, const Math::Vector2& pos);

        // Ray-AABB intersection (used internally for CCD optimization)
        static bool RayAABBIntersection(const Math::Vector2& rayOrigin,
                                       const Math::Vector2& rayDirection,
                                       const Math::Vector2& aabbMin,
                                       const Math::Vector2& aabbMax,
                                       float& tMin, float& tMax);
    };
}