#ifndef BVH_INL
#define BVH_INL

#include "BVH.hpp"

namespace Spatium
{
    template <typename T>
    BVH<T>::BVH() : m_Root(nullptr), m_ObjectCount(0)
    {

    }

    template <typename T>
    BVH<T>::~BVH()
    {
        Clear();
    }

    template <typename T>
    void BVH<T>::Clear()
    {
        if (m_Root != nullptr)
        {
            // Recursively delete nodes starting from the root.
            std::function<void(Node*)> DeleteNodes = [&](Node* node)
            {
                if (node != nullptr)
                {
                    // Leafs contain objects.
                    if (node->IsLeaf())
                    {
                        // Delete all objects in the linked list and reset their bvhInfo accordingly.
                        T currentObject = node->m_FirstObject;
                        while (currentObject != nullptr)
                        {
                            T nextObject = currentObject->bvhInfo.next;
                            currentObject->bvhInfo.next = nullptr;
                            currentObject->bvhInfo.prev = nullptr;
                            currentObject->bvhInfo.node = nullptr;
                            currentObject = nextObject;
                        }
                    }
                    else // Otherwise, recursively delete children.
                    {
                        DeleteNodes(node->m_Children[0]);
                        DeleteNodes(node->m_Children[1]);
                    }

                    // Finally, delete the node itself.
                    delete node;
                }
            };

            // Start deletion from the root itself.
            DeleteNodes(m_Root);

            // Reset the BVH state.
            m_Root = nullptr;
            m_ObjectCount = 0;
        }
    }

    template <typename T>
    template <typename Iterator>
    void BVH<T>::BuildTopDown(Iterator itBegin, Iterator itEnd, const BVHBuildConfiguration& buildConfiguration)
    {
        Clear();
        std::vector<T> sceneObjects(itBegin, itEnd);
        m_ObjectCount = (uint32_t)sceneObjects.size();
        m_Root = BuildTopDownRecursive(sceneObjects, 0, sceneObjects.size(), buildConfiguration, 0);
    }

    template <typename T>
    typename BVH<T>::BVHNode* BVH<T>::BuildTopDownRecursive(std::vector<T>& targetObjects, size_t beginIndex, size_t endIndex, const BVHBuildConfiguration& buildConfiguration, uint32_t currentDepth)
    {
        if (beginIndex >= endIndex)
        {
            return nullptr;
        }

        BVHNode* node = new BVHNode();
        node->m_AABB = CreateEncapsulatingBoundingVolume(targetObjects, beginIndex, endIndex);

        // Add objects to leaf if any of the following conditions are met.
        if (currentDepth >= config.m_MaxDepth || (endIndex - beginIndex) <= buildConfiguration.m_MinimumObjects || node->m_AABB.GetVolume() <= buildConfiguration.m_MinimumVolume)
        {
            for (size_t i = beginIndex; i < endIndex; i++)
            {
                node->AddObject(objects[i]);
            }

            return node;
        }

        // Otherwise, proceed to split. We will use the K-Splits Points approach here.
        size_t bestSplitPoint = PartitionObjects(targetObjects, beginIndex, endIndex);

        node->children[0] = BuildTopDownRecursive(targetObjects, beginIndex, bestSplitPoint, buildConfiguration, currentDepth + 1);
        node->children[1] = BuildTopDownRecursive(targetObjects, bestSplitPoint, endIndex, buildConfiguration, currentDepth + 1);

        return node;
    }

    template <typename T>
    size_t BVH<T>::PartitionObjects(std::vector<T>& targetObjects, size_t beginIndex, size_t endIndex, const BVHBuildConfiguration& buildConfiguration)
    {
        // Choose the best split based on Surface Area Heuristics.
        const size_t kSplitPoints = buildConfiguration.m_TopDownKSplitPoints;

        // Cache
        int bestAxis = -1; // 0 for X, 1 for Y and 2 for Z.
        float bestCost = std::numeric_limits<float>::max();
        size_t bestSplitPoint = beginIndex;

        for (int axis = 0; axis < 3; axis++)
        {
            // Sorts objects along each axis based on the center of their bounding volumes.
            std::sort(targetObjects.begin() + beginIndex, targetObjects.begin() + endIndex, [axis](const T& a, const T& b)
            {
                return a->bv.Center()[axis] < b->bv.Center()[axis];
            });

            std::vector<AABB> leftBounds(kSplitPoints);
            std::vector<AABB> rightBounds(kSplitPoints);

            // Compute bounds for left and right splits
            for (size_t i = 1; i < kSplitPoints; i++)
            {
                // Obtain a potential split point. We essentially "climb" in steps across the entire object interval to find the best point.
                size_t middleIndex = beginIndex + (endIndex - beginIndex) * i / kSplitPoints;

                // Create a bounding volume for all objects to its left and right respectively.
                leftBounds[i] = CreateEncapsulatingBoundingVolume(targetObjects, beginIndex, middleIndex);
                rightBounds[i] = CreateEncapsulatingBoundingVolume(targetObjects, middleIndex, endIndex);

                // Obtain SA for these two bounding volumes.
                float leftSurfaceArea = leftBounds[i].GetSurfaceArea();
                float rightSurfaceArea = rightBounds[i].GetSurfaceArea();

                // Obtain the cost of this split, normalized. This is important to avoid skewing costs naively towards ranges with more objects.
                float cost = (leftSurfaceArea * (float)(middleIndex - beginIndex) + rightSurfaceArea * (float)(endIndex - middleIndex)) / (float)(endIndex - beginIndex);

                // Cache the split position and axis if ideal.
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestSplitPoint = middleIndex;
                    bestAxis = axis;
                }
            }
        }

        // Once we're done, we do a final sort of the objects based on the best axis to ensure that subsequent operations use the found split position correctly.
        if (bestAxis != -1)
        {
            std::sort(targetObjects.begin() + beginIndex, targetObjects.begin() + endIndex, [bestAxis](const T& a, const T& b)
            {
                return a->m_AABB.GetCenter()[bestAxis] < b->m_AABB.GetCenter()[bestAxis];
            });
        }

        return bestSplitPoint;
    }


    template <typename T>
    template <typename Function>
    void BVH<T>::TraverseLevelOrderObjects(Function traversalFunction) const
    {
        if (m_Root != nullptr)
        {
            // Applies the function to each object.
            m_Root->TraverseLevelOrderObjects(traversalFunction);
        }
    }

    template <typename T>
    template <typename Function>
    void BVH<T>::TraverseLevelOrder(Function traversalFunction) const
    {
        if (m_Root != nullptr)
        {
            // Applies the function to each node.
            m_Root->TraverseLevelOrder(traversalFunction);
        }
    }

    template <typename T>
    bool BVH<T>::IsEmpty() const
    {
        return m_ObjectCount == 0;
    }

    template <typename T>
    int BVH<T>::GetDepth() const
    {
        if (m_Root != nullptr)
        {
            return m_Root->GetDepth();
        }

        return -1;
    }

    // Returns the number of nodes in the tree.
    template <typename T>
    int BVH<T>::GetSize() const
    {
        if (m_Root != nullptr)
        {
            return m_Root->GetSize();
        }

        return 0;
    }

    template <typename T>
    AABB BVH<T>::CreateEncapsulatingBoundingVolume(const std::vector<T>& targetObjects, size_t beginIndex, size_t endIndex)
    {
        AABB aabb = targetObjects[beginIndex]->m_AABB;
        for (size_t i = beginIndex + 1; i < endIndex; ++i)
        {
            aabb.Expand(targetObjects[i]->m_AABB);
        }
        return aabb;
    }
}

#endif