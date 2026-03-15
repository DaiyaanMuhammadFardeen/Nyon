#include "nyon/physics/DynamicTree.h"
#include <limits>
#include <algorithm>
#include <cassert>
#include <unordered_set>

namespace Nyon::Physics
{
    DynamicTree::DynamicTree()
        : m_root(TreeNode::NULL_NODE)
        , m_nodeCount(0)
        , m_proxyCount(0)
        , m_freeList(TreeNode::NULL_NODE)
    {
        // Pre-allocate some nodes
        m_nodes.resize(NODE_CAPACITY_INCREMENT);
        for (int i = 0; i < NODE_CAPACITY_INCREMENT - 1; ++i)
        {
            m_nodes[i].parent = i + 1;
        }
        m_nodes[NODE_CAPACITY_INCREMENT - 1].parent = TreeNode::NULL_NODE;
        m_freeList = 0;
    }
    
    DynamicTree::~DynamicTree()
    {
        // Destructor is empty - nodes are managed by vector
    }
    
    uint32_t DynamicTree::AllocateNode()
    {
        // Expand if necessary
        if (m_freeList == TreeNode::NULL_NODE)
        {
            uint32_t oldSize = static_cast<uint32_t>(m_nodes.size());
            m_nodes.resize(oldSize + NODE_CAPACITY_INCREMENT);
            
            // Link new nodes into free list
            for (uint32_t i = oldSize; i < oldSize + NODE_CAPACITY_INCREMENT - 1; ++i)
            {
                m_nodes[i].parent = i + 1;
            }
            m_nodes[oldSize + NODE_CAPACITY_INCREMENT - 1].parent = TreeNode::NULL_NODE;
            m_freeList = oldSize;
        }
        
        uint32_t nodeId = m_freeList;
        m_freeList = m_nodes[nodeId].parent;
        m_nodes[nodeId] = TreeNode(); // Reset node
        ++m_nodeCount;
        
        return nodeId;
    }
    
    void DynamicTree::FreeNode(uint32_t nodeId)
    {
        assert(0 <= nodeId && nodeId < m_nodes.size());
        assert(0 < m_nodeCount);
        
        m_nodes[nodeId].parent = m_freeList;
        m_freeList = nodeId;
        --m_nodeCount;
    }
    
    uint32_t DynamicTree::CreateProxy(const AABB& aabb, uint32_t userData)
    {
        uint32_t proxyId = AllocateNode();
        
        // Fatten the AABB
        Math::Vector2 r{AABB_EXTENSION, AABB_EXTENSION};
        m_nodes[proxyId].aabb.lowerBound = aabb.lowerBound - r;
        m_nodes[proxyId].aabb.upperBound = aabb.upperBound + r;
        m_nodes[proxyId].userData = userData;
        m_nodes[proxyId].height = 0;
        m_nodes[proxyId].moved = true;
        
        InsertLeaf(proxyId);
        ++m_proxyCount;
        
        return proxyId;
    }
    
    void DynamicTree::DestroyProxy(uint32_t proxyId)
    {
        assert(0 <= proxyId && proxyId < m_nodes.size());
        assert(m_nodes[proxyId].IsLeaf());
        
        RemoveLeaf(proxyId);
        FreeNode(proxyId);
        --m_proxyCount;
    }
    
    bool DynamicTree::MoveProxy(uint32_t proxyId, const AABB& aabb, const Math::Vector2& displacement)
    {
        assert(0 <= proxyId && proxyId < m_nodes.size());
        
        // Extended AABB
        Math::Vector2 r{AABB_EXTENSION, AABB_EXTENSION};
        AABB fatAABB;
        fatAABB.lowerBound = aabb.lowerBound - r;
        fatAABB.upperBound = aabb.upperBound + r;
        
        // Predict AABB movement
        Math::Vector2 d = displacement * AABB_MULTIPLIER;
        if (d.x < 0.0f) fatAABB.lowerBound.x += d.x;
        else fatAABB.upperBound.x += d.x;
        
        if (d.y < 0.0f) fatAABB.lowerBound.y += d.y;
        else fatAABB.upperBound.y += d.y;
        
        const AABB& treeAABB = m_nodes[proxyId].aabb;
        if (treeAABB.Contains(aabb.lowerBound) && treeAABB.Contains(aabb.upperBound))
        {
            // No need to update
            m_nodes[proxyId].moved = false;
            return false;
        }
        
        RemoveLeaf(proxyId);
        
        m_nodes[proxyId].aabb = fatAABB;
        
        // Update height (likely unchanged)
        m_nodes[proxyId].height = 0;
        
        InsertLeaf(proxyId);
        m_nodes[proxyId].moved = true;
        
        return true;
    }
    
    void DynamicTree::InsertLeaf(uint32_t leaf)
    {
        if (m_root == TreeNode::NULL_NODE)
        {
            m_root = leaf;
            m_nodes[m_root].parent = TreeNode::NULL_NODE;
            return;
        }
        
        // Find the best sibling for this leaf
        AABB leafAABB = m_nodes[leaf].aabb;
        uint32_t index = m_root;
        while (!m_nodes[index].IsLeaf())
        {
            uint32_t child1 = m_nodes[index].child1;
            uint32_t child2 = m_nodes[index].child2;
            
            float area = m_nodes[index].aabb.GetPerimeter();
            
            AABB combinedAABB = m_nodes[index].aabb;
            combinedAABB.Combine(leafAABB);
            float combinedArea = combinedAABB.GetPerimeter();
            
            // Cost of creating a new parent for this node and the new leaf
            float cost = 2.0f * combinedArea;
            
            // Minimum cost of pushing the leaf further down the tree
            float inheritanceCost = 2.0f * (combinedArea - area);
            
            // Cost of descending into child1
            float cost1;
            if (m_nodes[child1].IsLeaf())
            {
                AABB aabb = leafAABB;
                aabb.Combine(m_nodes[child1].aabb);
                cost1 = aabb.GetPerimeter() + inheritanceCost;
            }
            else
            {
                AABB aabb = leafAABB;
                aabb.Combine(m_nodes[child1].aabb);
                float oldArea = m_nodes[child1].aabb.GetPerimeter();
                float newArea = aabb.GetPerimeter();
                cost1 = (newArea - oldArea) + inheritanceCost;
            }
            
            // Cost of descending into child2
            float cost2;
            if (m_nodes[child2].IsLeaf())
            {
                AABB aabb = leafAABB;
                aabb.Combine(m_nodes[child2].aabb);
                cost2 = aabb.GetPerimeter() + inheritanceCost;
            }
            else
            {
                AABB aabb = leafAABB;
                aabb.Combine(m_nodes[child2].aabb);
                float oldArea = m_nodes[child2].aabb.GetPerimeter();
                float newArea = aabb.GetPerimeter();
                cost2 = (newArea - oldArea) + inheritanceCost;
            }
            
            // Descend according to the minimum cost
            if (cost < cost1 && cost < cost2)
            {
                break;
            }
            
            // Descend
            if (cost1 < cost2)
            {
                index = child1;
            }
            else
            {
                index = child2;
            }
        }
        
        uint32_t sibling = index;
        
        // Create a new parent
        uint32_t oldParent = m_nodes[sibling].parent;
        uint32_t newParent = AllocateNode();
        m_nodes[newParent].parent = oldParent;
        m_nodes[newParent].userData = 0;
        m_nodes[newParent].aabb = leafAABB;
        m_nodes[newParent].aabb.Combine(m_nodes[sibling].aabb);
        m_nodes[newParent].height = m_nodes[sibling].height + 1;
        
        if (oldParent != TreeNode::NULL_NODE)
        {
            // The sibling was not the root
            if (m_nodes[oldParent].child1 == sibling)
            {
                m_nodes[oldParent].child1 = newParent;
            }
            else
            {
                m_nodes[oldParent].child2 = newParent;
            }
            
            m_nodes[newParent].child1 = sibling;
            m_nodes[newParent].child2 = leaf;
            m_nodes[sibling].parent = newParent;
            m_nodes[leaf].parent = newParent;
        }
        else
        {
            // The sibling was the root
            m_nodes[newParent].child1 = sibling;
            m_nodes[newParent].child2 = leaf;
            m_nodes[sibling].parent = newParent;
            m_nodes[leaf].parent = newParent;
            m_root = newParent;
        }
        
        // Walk back up the tree fixing heights and AABBs
        index = m_nodes[leaf].parent;
        while (index != TreeNode::NULL_NODE)
        {
            index = Balance(index);
            
            uint32_t child1 = m_nodes[index].child1;
            uint32_t child2 = m_nodes[index].child2;
            
            assert(child1 != TreeNode::NULL_NODE);
            assert(child2 != TreeNode::NULL_NODE);
            
            m_nodes[index].height = 1 + std::max(m_nodes[child1].height, m_nodes[child2].height);
            m_nodes[index].aabb = m_nodes[child1].aabb;
            m_nodes[index].aabb.Combine(m_nodes[child2].aabb);
            
            index = m_nodes[index].parent;
        }
    }
    
    void DynamicTree::RemoveLeaf(uint32_t leaf)
    {
        if (leaf == m_root)
        {
            m_root = TreeNode::NULL_NODE;
            return;
        }
        
        uint32_t parent = m_nodes[leaf].parent;
        uint32_t grandParent = m_nodes[parent].parent;
        uint32_t sibling;
        
        if (m_nodes[parent].child1 == leaf)
        {
            sibling = m_nodes[parent].child2;
        }
        else
        {
            sibling = m_nodes[parent].child1;
        }
        
        if (grandParent != TreeNode::NULL_NODE)
        {
            // Destroy parent and connect sibling to grandParent
            if (m_nodes[grandParent].child1 == parent)
            {
                m_nodes[grandParent].child1 = sibling;
            }
            else
            {
                m_nodes[grandParent].child2 = sibling;
            }
            m_nodes[sibling].parent = grandParent;
            FreeNode(parent);
            
            // Adjust ancestor bounds
            uint32_t index = grandParent;
            while (index != TreeNode::NULL_NODE)
            {
                index = Balance(index);
                
                uint32_t child1 = m_nodes[index].child1;
                uint32_t child2 = m_nodes[index].child2;
                
                m_nodes[index].aabb = m_nodes[child1].aabb;
                m_nodes[index].aabb.Combine(m_nodes[child2].aabb);
                m_nodes[index].height = 1 + std::max(m_nodes[child1].height, m_nodes[child2].height);
                
                index = m_nodes[index].parent;
            }
        }
        else
        {
            m_root = sibling;
            m_nodes[sibling].parent = TreeNode::NULL_NODE;
            FreeNode(parent);
        }
    }
    
    uint32_t DynamicTree::Balance(uint32_t iA)
    {
        assert(iA != TreeNode::NULL_NODE);
        
        TreeNode& A = m_nodes[iA];
        if (A.IsLeaf() || A.height < 2)
        {
            return iA;
        }
        
        uint32_t iB = A.child1;
        uint32_t iC = A.child2;
        assert(0 <= iB && iB < m_nodes.size());
        assert(0 <= iC && iC < m_nodes.size());
        
        TreeNode& B = m_nodes[iB];
        TreeNode& C = m_nodes[iC];
        
        int balance = C.height - B.height;
        
        // Rotate C up
        if (balance > 1)
        {
            uint32_t iF = C.child1;
            uint32_t iG = C.child2;
            TreeNode& F = m_nodes[iF];
            TreeNode& G = m_nodes[iG];
            assert(0 <= iF && iF < m_nodes.size());
            assert(0 <= iG && iG < m_nodes.size());
            
            // Swap A and C
            C.child1 = iA;
            C.parent = A.parent;
            A.parent = iC;
            
            // A's old parent should point to C
            if (C.parent != TreeNode::NULL_NODE)
            {
                if (m_nodes[C.parent].child1 == iA)
                {
                    m_nodes[C.parent].child1 = iC;
                }
                else
                {
                    assert(m_nodes[C.parent].child2 == iA);
                    m_nodes[C.parent].child2 = iC;
                }
            }
            else
            {
                m_root = iC;
            }
            
            // Rotate
            if (F.height > G.height)
            {
                C.child2 = iF;
                A.child2 = iG;
                G.parent = iA;
                A.aabb = B.aabb;
                A.aabb.Combine(G.aabb);
                C.aabb = A.aabb;
                C.aabb.Combine(F.aabb);
                
                A.height = 1 + std::max(B.height, G.height);
                C.height = 1 + std::max(A.height, F.height);
            }
            else
            {
                C.child2 = iG;
                A.child2 = iF;
                F.parent = iA;
                A.aabb = B.aabb;
                A.aabb.Combine(F.aabb);
                C.aabb = A.aabb;
                C.aabb.Combine(G.aabb);
                
                A.height = 1 + std::max(B.height, F.height);
                C.height = 1 + std::max(A.height, G.height);
            }
            
            return iC;
        }
        
        // Rotate B up
        if (balance < -1)
        {
            uint32_t iD = B.child1;
            uint32_t iE = B.child2;
            TreeNode& D = m_nodes[iD];
            TreeNode& E = m_nodes[iE];
            assert(0 <= iD && iD < m_nodes.size());
            assert(0 <= iE && iE < m_nodes.size());
            
            // Swap A and B
            B.child1 = iA;
            B.parent = A.parent;
            A.parent = iB;
            
            // A's old parent should point to B
            if (B.parent != TreeNode::NULL_NODE)
            {
                if (m_nodes[B.parent].child1 == iA)
                {
                    m_nodes[B.parent].child1 = iB;
                }
                else
                {
                    assert(m_nodes[B.parent].child2 == iA);
                    m_nodes[B.parent].child2 = iB;
                }
            }
            else
            {
                m_root = iB;
            }
            
            // Rotate
            if (D.height > E.height)
            {
                B.child2 = iD;
                A.child1 = iE;
                E.parent = iA;
                A.aabb = C.aabb;
                A.aabb.Combine(E.aabb);
                B.aabb = A.aabb;
                B.aabb.Combine(D.aabb);
                
                A.height = 1 + std::max(C.height, E.height);
                B.height = 1 + std::max(A.height, D.height);
            }
            else
            {
                B.child2 = iE;
                A.child1 = iD;
                D.parent = iA;
                A.aabb = C.aabb;
                A.aabb.Combine(D.aabb);
                B.aabb = A.aabb;
                B.aabb.Combine(E.aabb);
                
                A.height = 1 + std::max(C.height, D.height);
                B.height = 1 + std::max(A.height, E.height);
            }
            
            return iB;
        }
        
        return iA;
    }
    
    void DynamicTree::Rebuild(bool fullRebuild)
    {
        // Simple rebuild strategy - could be optimized further
        std::vector<uint32_t> proxies;
        proxies.reserve(m_proxyCount);
        
        // Build a set of free node indices for O(1) lookup
        std::unordered_set<uint32_t> freeSet;
        uint32_t f = m_freeList;
        while (f != TreeNode::NULL_NODE)
        {
            freeSet.insert(f);
            f = m_nodes[f].parent;
        }
        
        // Collect all proxies (nodes that are not in free list and are leaves)
        for (uint32_t i = 0; i < m_nodes.size(); ++i)
        {
            if (freeSet.count(i) == 0 && m_nodes[i].IsLeaf())
            {
                proxies.push_back(i);
            }
        }
        
        // Remove all proxies
        for (uint32_t proxyId : proxies)
        {
            RemoveLeaf(proxyId);
        }
        
        // Insert all proxies
        for (uint32_t proxyId : proxies)
        {
            InsertLeaf(proxyId);
        }
    }
    
    const AABB& DynamicTree::GetFatAABB(uint32_t proxyId) const
    {
        assert(0 <= proxyId && proxyId < m_nodes.size());
        return m_nodes[proxyId].aabb;
    }
    
    uint32_t DynamicTree::GetUserData(uint32_t proxyId) const
    {
        assert(0 <= proxyId && proxyId < m_nodes.size());
        return m_nodes[proxyId].userData;
    }
    
    bool DynamicTree::WasMoved(uint32_t proxyId) const
    {
        assert(0 <= proxyId && proxyId < m_nodes.size());
        return m_nodes[proxyId].moved;
    }
    
    void DynamicTree::ClearMoved(uint32_t proxyId)
    {
        assert(0 <= proxyId && proxyId < m_nodes.size());
        m_nodes[proxyId].moved = false;
    }
    
    int DynamicTree::GetHeight() const
    {
        if (m_root == TreeNode::NULL_NODE)
        {
            return 0;
        }
        
        return static_cast<int>(m_nodes[m_root].height);
    }
    
    void DynamicTree::Validate() const
    {
#ifndef NDEBUG
        ValidateStructure(m_root);
        ValidateMetrics(m_root);
        
        uint32_t freeCount = 0;
        uint32_t freeIndex = m_freeList;
        while (freeIndex != TreeNode::NULL_NODE)
        {
            assert(0 <= freeIndex && freeIndex < m_nodes.size());
            freeIndex = m_nodes[freeIndex].parent;
            ++freeCount;
        }
        
        assert(GetHeight() == static_cast<int>(ComputeHeight(m_root)));
        assert(m_nodeCount + freeCount == m_nodes.size());
#endif
    }
    
    void DynamicTree::ValidateStructure(uint32_t index) const
    {
#ifndef NDEBUG
        if (index == TreeNode::NULL_NODE)
        {
            return;
        }
        
        if (index == m_root)
        {
            assert(m_nodes[index].parent == TreeNode::NULL_NODE);
        }
        
        const TreeNode& node = m_nodes[index];
        
        uint32_t child1 = node.child1;
        uint32_t child2 = node.child2;
        
        if (node.IsLeaf())
        {
            assert(child1 == TreeNode::NULL_NODE);
            assert(child2 == TreeNode::NULL_NODE);
            assert(node.height == 0);
            return;
        }
        
        assert(0 <= child1 && child1 < m_nodes.size());
        assert(0 <= child2 && child2 < m_nodes.size());
        
        assert(m_nodes[child1].parent == index);
        assert(m_nodes[child2].parent == index);
        
        ValidateStructure(child1);
        ValidateStructure(child2);
#endif
    }
    
    void DynamicTree::ValidateMetrics(uint32_t index) const
    {
#ifndef NDEBUG
        if (index == TreeNode::NULL_NODE)
        {
            return;
        }
        
        const TreeNode& node = m_nodes[index];
        
        uint32_t child1 = node.child1;
        uint32_t child2 = node.child2;
        
        if (node.IsLeaf())
        {
            assert(child1 == TreeNode::NULL_NODE);
            assert(child2 == TreeNode::NULL_NODE);
            assert(node.height == 0);
            return;
        }
        
        assert(0 <= child1 && child1 < m_nodes.size());
        assert(0 <= child2 && child2 < m_nodes.size());
        
        int height1 = static_cast<int>(m_nodes[child1].height);
        int height2 = static_cast<int>(m_nodes[child2].height);
        int height = 1 + std::max(height1, height2);
        assert(node.height == height);
        
        AABB aabb = m_nodes[child1].aabb;
        aabb.Combine(m_nodes[child2].aabb);
        
        assert(aabb.lowerBound.x == node.aabb.lowerBound.x);
        assert(aabb.lowerBound.y == node.aabb.lowerBound.y);
        assert(aabb.upperBound.x == node.aabb.upperBound.x);
        assert(aabb.upperBound.y == node.aabb.upperBound.y);
        
        ValidateMetrics(child1);
        ValidateMetrics(child2);
#endif
    }
    
    uint32_t DynamicTree::ComputeHeight(uint32_t nodeId) const
    {
        assert(0 <= nodeId && nodeId < m_nodes.size());
        const TreeNode& node = m_nodes[nodeId];
        
        if (node.IsLeaf())
        {
            return 0;
        }
        
        uint32_t height1 = ComputeHeight(node.child1);
        uint32_t height2 = ComputeHeight(node.child2);
        return 1 + std::max(height1, height2);
    }
}
