#pragma once

#include "nyon/math/Vector2.h"
#include <vector>
#include <algorithm>
#include <limits>
#include <cstdint>

namespace Nyon::Physics
{
    /**
     * @brief Axis-Aligned Bounding Box structure.
     * 
     * Represents a 2D AABB for spatial partitioning and collision detection.
     */
    struct AABB
    {
        Math::Vector2 lowerBound;  // Lower left corner
        Math::Vector2 upperBound;  // Upper right corner
        
        AABB() = default;
        AABB(const Math::Vector2& min, const Math::Vector2& max) 
            : lowerBound(min), upperBound(max) {}
        
        // Get center point
        Math::Vector2 GetCenter() const
        {
            return {(lowerBound.x + upperBound.x) * 0.5f, 
                    (lowerBound.y + upperBound.y) * 0.5f};
        }
        
        // Get extents (half-widths)
        Math::Vector2 GetExtents() const
        {
            return {(upperBound.x - lowerBound.x) * 0.5f,
                    (upperBound.y - lowerBound.y) * 0.5f};
        }
        
        // Get perimeter (used for tree balancing)
        float GetPerimeter() const
        {
            float wx = upperBound.x - lowerBound.x;
            float wy = upperBound.y - lowerBound.y;
            return 2.0f * (wx + wy);
        }
        
        // Combine with another AABB
        void Combine(const AABB& other)
        {
            lowerBound.x = std::min(lowerBound.x, other.lowerBound.x);
            lowerBound.y = std::min(lowerBound.y, other.lowerBound.y);
            upperBound.x = std::max(upperBound.x, other.upperBound.x);
            upperBound.y = std::max(upperBound.y, other.upperBound.y);
        }
        
        // Combine with a point
        void Combine(const Math::Vector2& point)
        {
            lowerBound.x = std::min(lowerBound.x, point.x);
            lowerBound.y = std::min(lowerBound.y, point.y);
            upperBound.x = std::max(upperBound.x, point.x);
            upperBound.y = std::max(upperBound.y, point.y);
        }
        
        // Check if contains a point
        bool Contains(const Math::Vector2& point) const
        {
            return point.x >= lowerBound.x && point.x <= upperBound.x &&
                   point.y >= lowerBound.y && point.y <= upperBound.y;
        }
        
        // Check if overlaps with another AABB
        bool Overlaps(const AABB& other) const
        {
            return lowerBound.x <= other.upperBound.x && 
                   upperBound.x >= other.lowerBound.x &&
                   lowerBound.y <= other.upperBound.y && 
                   upperBound.y >= other.lowerBound.y;
        }
        
        // Ray cast against AABB
        bool RayCast(const Math::Vector2& origin, const Math::Vector2& direction, 
                     float maxFraction, float& hitFraction) const
        {
            float tmin = -std::numeric_limits<float>::max();
            float tmax = std::numeric_limits<float>::max();
            
            // X plane intersection
            if (std::abs(direction.x) < std::numeric_limits<float>::epsilon())
            {
                // Ray parallel to Y planes
                if (origin.x < lowerBound.x || origin.x > upperBound.x)
                    return false;
            }
            else
            {
                float inv_dx = 1.0f / direction.x;
                float t1 = (lowerBound.x - origin.x) * inv_dx;
                float t2 = (upperBound.x - origin.x) * inv_dx;
                
                if (t1 > t2) std::swap(t1, t2);
                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);
                
                if (tmin > tmax || tmax < 0.0f || tmin > maxFraction)
                    return false;
            }
            
            // Y plane intersection
            if (std::abs(direction.y) < std::numeric_limits<float>::epsilon())
            {
                // Ray parallel to X planes
                if (origin.y < lowerBound.y || origin.y > upperBound.y)
                    return false;
            }
            else
            {
                float inv_dy = 1.0f / direction.y;
                float t1 = (lowerBound.y - origin.y) * inv_dy;
                float t2 = (upperBound.y - origin.y) * inv_dy;
                
                if (t1 > t2) std::swap(t1, t2);
                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);
                
                if (tmin > tmax || tmax < 0.0f || tmin > maxFraction)
                    return false;
            }
            
            hitFraction = tmin;
            return true;
        }
    };
    
    /**
     * @brief Dynamic Tree Node for spatial partitioning.
     * 
     * Internal node structure for the dynamic tree AABB hierarchy.
     */
    struct TreeNode
    {
        static constexpr uint32_t NULL_NODE = 0xFFFFFFFF;
        
        AABB aabb;                 // Node bounding box
        uint32_t parent;           // Parent node index
        uint32_t child1;           // Left child index
        uint32_t child2;           // Right child index
        uint32_t userData;         // User data (entity/shape ID)
        int32_t height;            // Node height for balancing (now int32_t)
        bool moved;                // Whether node moved significantly
        
        TreeNode() : parent(NULL_NODE), child1(NULL_NODE), child2(NULL_NODE), 
                     userData(0), height(0), moved(false) {}
        
        bool IsLeaf() const { return child1 == NULL_NODE; }
    };
    
    /**
     * @brief Dynamic Tree for broad-phase collision detection.
     * 
     * Implements a dynamic AABB tree for efficient spatial queries.
     * Inspired by Box2D's b2DynamicTree implementation.
     */
    class DynamicTree
    {
    public:
        DynamicTree();
        ~DynamicTree();
        
        // Proxy management
        uint32_t CreateProxy(const AABB& aabb, uint32_t userData);
        void DestroyProxy(uint32_t proxyId);
        bool MoveProxy(uint32_t proxyId, const AABB& aabb, const Math::Vector2& displacement);
        
        // Tree operations
        void Rebuild(bool fullRebuild = false);
        void Validate() const;
        
        // Queries
        template<typename T>
        void Query(const AABB& aabb, T* callback) const
        {
            QueryInternal(aabb, callback, m_root);
        }
        
        template<typename T>
        void RayCast(const Math::Vector2& origin, const Math::Vector2& direction, 
                     float maxFraction, T* callback) const
        {
            RayCastInternal(origin, direction, maxFraction, callback, m_root);
        }
        
        // Accessors
        const AABB& GetFatAABB(uint32_t proxyId) const;
        uint32_t GetUserData(uint32_t proxyId) const;
        bool WasMoved(uint32_t proxyId) const;
        void ClearMoved(uint32_t proxyId);
        
        // Statistics
        int GetHeight() const;
        int GetNodeCount() const { return static_cast<int>(m_nodeCount); }
        int GetProxyCount() const { return m_proxyCount; }
        
    private:
        static constexpr float AABB_EXTENSION = 0.1f;      // AABB extension factor
        static constexpr float AABB_MULTIPLIER = 2.0f;     // AABB multiplier for movement
        static constexpr int NODE_CAPACITY_INCREMENT = 16; // Node pool growth increment
        
        std::vector<TreeNode> m_nodes;     // Node pool
        uint32_t m_root;                   // Root node index
        uint32_t m_nodeCount;              // Number of allocated nodes
        uint32_t m_proxyCount;             // Number of proxies
        uint32_t m_freeList;               // Free node list head
        
        // Internal operations
        uint32_t AllocateNode();
        void FreeNode(uint32_t nodeId);
        void InsertLeaf(uint32_t leaf);
        void RemoveLeaf(uint32_t leaf);
        uint32_t Balance(uint32_t index);
        uint32_t ComputeHeight(uint32_t nodeId) const;
        void ValidateStructure(uint32_t index) const;
        void ValidateMetrics(uint32_t index) const;
        
        // Query implementations
        template<typename T>
        void QueryInternal(const AABB& aabb, T* callback, uint32_t nodeId) const
        {
            if (nodeId == TreeNode::NULL_NODE)
                return;
                
            const TreeNode& node = m_nodes[nodeId];
            
            if (aabb.Overlaps(node.aabb))
            {
                if (node.IsLeaf())
                {
                    // Leaf node - call callback
                    if (!callback->QueryCallback(nodeId, node.userData))
                        return; // Stop querying
                }
                else
                {
                    // Internal node - recurse
                    QueryInternal(aabb, callback, node.child1);
                    QueryInternal(aabb, callback, node.child2);
                }
            }
        }
        
        template<typename T>
        void RayCastInternal(const Math::Vector2& origin, const Math::Vector2& direction,
                           float maxFraction, T* callback, uint32_t nodeId) const
        {
            if (nodeId == TreeNode::NULL_NODE)
                return;
                
            const TreeNode& node = m_nodes[nodeId];
            
            float hitFraction;
            if (node.aabb.RayCast(origin, direction, maxFraction, hitFraction))
            {
                if (node.IsLeaf())
                {
                    // Leaf node - call callback
                    if (!callback->RayCastCallback(hitFraction, nodeId, node.userData))
                        return; // Stop ray casting
                }
                else
                {
                    // Internal node - recurse
                    RayCastInternal(origin, direction, maxFraction, callback, node.child1);
                    RayCastInternal(origin, direction, maxFraction, callback, node.child2);
                }
            }
        }
    };
    
    /**
     * @brief Query callback interface for tree queries.
     * 
     * Interface for custom query callbacks used with DynamicTree queries.
     */
    struct ITreeQueryCallback
    {
        virtual ~ITreeQueryCallback() = default;
        virtual bool QueryCallback(uint32_t nodeId, uint32_t userData) = 0;
    };
    
    /**
     * @brief Ray cast callback interface for tree ray casts.
     * 
     * Interface for custom ray cast callbacks used with DynamicTree ray casts.
     */
    struct ITreeRayCastCallback
    {
        virtual ~ITreeRayCastCallback() = default;
        virtual bool RayCastCallback(float fraction, uint32_t nodeId, uint32_t userData) = 0;
    };
}
