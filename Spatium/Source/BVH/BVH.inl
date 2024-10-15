#ifndef BVH_INL
#define BVH_INL

#include <queue>

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
    template <typename Iterator>
    void BVH<T>::BuildBottomUp(Iterator itBegin, Iterator itEnd, const BVHBuildConfiguration& buildConfiguration)
    {
        // Ryan: Not using any configuration options here. This BottomUp build technique is optimized for speed and hence adheres strictly to it.
        // UNUSED_PARAMETER(config);

        Clear();

        // Insert all objects into vector.
        std::vector<BVHNode*> objectNodes;
        objectNodes.reserve(itEnd - itBegin);
        for (Iterator it = itBegin; it != itEnd; it++)
        {
            BVHNode* leafNode = new BVHNode();
            leafNode->AddObject(*it);
            objectNodes.push_back(leafNode);
            m_ObjectCount++;
        }

        m_Root = BuildBottomUpIterative(objectNodes);
    }

    template <typename T>
    typename BVH<T>::BVHNode* BVH<T>::BuildBottomUpIterative(std::vector<BVHNode*>& objectNodes)
    {
        // Perform a first pass for each node to keep track of which other node is the best one. This cuts down the construction time drastically!
        auto nodeComparator = [](const std::pair<BVHNode*, float>& a, const std::pair<BVHNode*, float>& b)
        {
            return a.second > b.second;
        };
        // Create a priority queue to store nodes, ordered by the SAH comparator above.
        std::priority_queue<std::pair<BVHNode*, float>, std::vector<std::pair<BVHNode*, float>>, decltype(nodeComparator)> priorityQueue(nodeComparator);

        // Initialize the priority queue with initial best-pair costs, such that the cheapest is always at the front of the PQ.
        for (const auto& objectNode : objectNodes)
        {
            // We could also return the node, but we will run into issues if there are other nodes also with that candidate.
            // This can also severaly litter the priority queue with invalid pairs if cached. Hence, we will just obtain a cost directly.
            float initialBestPairCost = ComputeBestPairCost(objectNode, objectNodes);
            priorityQueue.push({ objectNode, initialBestPairCost });
        }

        // The final object left in this priority queue will always be the root.
        while (priorityQueue.size() > 1)
        {
            BVHNode* bestNode = priorityQueue.top().first;
            priorityQueue.pop();

            // If the node's parent pointer has been assigned, it means this node has already been merged. We can prune it immediately.
            if (bestNode->m_Parent != nullptr)
            {
                continue;
            }

            BVHNode* bestMergeCandidate = FindBestMergeCandidate(bestNode, objectNodes);
            // If a candidate could not be found, or if this candidate already has a parent...
            if (bestMergeCandidate == nullptr || bestMergeCandidate->m_Parent != nullptr)
            {
                // We will return this node back to the priority queue in hopes of finding another candidate.
                float newCost = ComputeBestPairCost(bestNode, objectNodes);
                priorityQueue.push({ bestNode, newCost });
                continue;
            }

            // Create a new parent node
            BVHNode* parentNode = CreateParentNode(bestNode, bestMergeCandidate);

            // Push parent node back into object list for evaluation with other nodes.
            objectNodes.push_back(parentNode);
            // Compute the new best-pair cost for the parent and add it to the priority queue.
            float parentCost = ComputeBestPairCost(parentNode, objectNodes);
            priorityQueue.push({ parentNode, parentCost });
        }

        return priorityQueue.top().first;  // This is the root of the final BVH
    }

    template <typename T>
    typename BVH<T>::BVHNode* BVH<T>::FindBestMergeCandidate(BVHNode* node, const std::vector<BVHNode*>& nodes)
    {
        BVHNode* bestCandidate = nullptr;
        float bestCost = std::numeric_limits<float>::max();

        for (BVHNode* candidateNode : nodes)
        {
            if (candidateNode == node || candidateNode->m_Parent != nullptr)
            {
                continue;
            }

            float currentCost = node->m_AABB.Union(candidateNode->m_AABB).GetSurfaceArea();
            if (currentCost < bestCost)
            {
                bestCost = currentCost;
                bestCandidate = candidateNode;
            }
        }

        return bestCandidate;
    }

    template <typename T>
    typename BVH<T>::BVHNode* BVH<T>::CreateParentNode(BVHNode* leftNode, BVHNode* rightNode)
    {
        // Create a new parent node.
        BVHNode* parentNode = new BVHNode();

        // Set the left and right children of the parent node.
        parentNode->m_Children[0] = leftNode;
        parentNode->m_Children[1] = rightNode;

        // Compute the bounding volume that encompasses both child nodes.
        parentNode->m_AABB = leftNode->m_AABB.Union(rightNode->m_AABB);

        // Set the parent pointers for the child nodes.
        leftNode->m_Parent = parentNode;
        rightNode->m_Parent = parentNode;

        return parentNode;
    }

    template <typename T>
    float BVH<T>::ComputeBestPairCost(BVHNode* node, const std::vector<BVHNode*>& nodes)
    {
        // Keep track of the best cost thus far between the node and all other candidate nodes.
        float bestCost = std::numeric_limits<float>::max();

        for (BVHNode* candidateNode : nodes)
        {
            // Skip if this candidate node is the subject itself, or if its parent has already been assigned.
            if (candidateNode == node || candidateNode->m_Parent != nullptr)
            {
                continue;
            }

            float currentCost = node->m_AABB.Union(candidateNode->m_AABB).GetSurfaceArea();
            if (currentCost < bestCost)
            {
                bestCost = currentCost;
            }
        }

        return bestCost;
    }

    template <typename T>
    template <typename Iterator>
    void BVH<T>::Insert(Iterator itBegin, Iterator itEnd, const BVHBuildConfiguration& buildConfiguration)
    {
        for (auto it = itBegin; it != itEnd; ++it)
        {
            Insert(*it, buildConfiguration);
            m_ObjectCount++;
        }
    }

    // Inserts a single object into the BVH.
    template <typename T>
    void BVH<T>::Insert(T targetObject, const BVHBuildConfiguration& buildConfiguration)
    {
        // The first object that comes into the empty tree will always be its root.
        if (m_Root == nullptr)
        {
            m_Root = new BVHNode();
            m_Root->AddObject(object);
            return;
        }

        // First, we first find the best sibling for the object.
        BVHNode* siblingNode = FindBestSibling(targetObject);

        // If the volume exceeds the minimumly allowed volume, we split. This means the creation of a new parent node.
        if (siblingNode->m_AABB.Union(targetObject->m_AABB).GetVolume() > buildConfiguration.m_MinimumVolume)
        {
            // Create node for the current object.
            BVHNode* newNode = new BVHNode();
            newNode->AddObject(object);

            // Obtain the old parent of the sibling node for reconnection later.
            BVHNode* oldParent = siblingNode->m_Parent;
            // Create node for our new parent.
            BVHNode* newParent = new BVHNode();
            // Reconnect to old parent.
            newParent->m_Parent = oldParent;
            // Create new bounding volume for the sibling and object.
            newParent->m_AABB = siblingNode->m_AABB.Union(targetObject->m_AABB);

            // If the old parent wasn't a null pointer, it means we're somewhere in the hierarchy.
            if (oldParent != nullptr)
            {
                // Ensure that we attach to correct node (left/right) of the old parent.
                if (oldParent->m_Children[0] == siblingNode)
                {
                    oldParent->m_Children[0] = newParent;
                }
                else
                {
                    oldParent->m_Children[1] = newParent;
                }

                // Hook new parent to our sibling and new node.
                newParent->m_Children[0] = siblingNode;
                newParent->m_Children[1] = newNode;
                // Vice-versa for the latter two.
                siblingNode->m_Parent = newParent;
                newNode->m_Parent = newParent;
            }
            else // Otherwise, the sibling was the root of the tree.
            {
                // Hook new parent to our sibling and new node.
                newParent->m_Children[0] = siblingNode;
                newParent->m_Children[1] = newNode;
                // Vice-versa for the latter two.
                siblingNode->m_Parent = newParent;
                newNode->m_Parent = newParent;

                // Change the root of the tree accordingly.
                m_Root = newParent;
            }

            // Head back up through the parent node and refit AABBs.
            BVHNode* parentNode = newNode->m_Parent;
            while (parentNode != nullptr)
            {
                BVHNode* leftChild = parentNode->m_Children[0];
                BVHNode* rightChild = parentNode->m_Children[1];

                parentNode->m_AABB = leftChild->m_AABB.Union(rightChild->m_AABB);

                // Rebalance.
                RotateRebalance(parentNode);

                parentNode = parentNode->m_Parent;
            }
        }
        else
        {
            // Add object to found node as it does not exceed volume capacity.
            siblingNode->AddObject(object);

            // Head back up through the parent node and refit AABBs accordingly.
            BVHNode* parentNode = siblingNode->parent;
            while (parentNode != nullptr)
            {
                BVHNode* leftChild = parentNode->m_Children[0];
                BVHNode* rightChild = parentNode->m_Children[1];

                parentNode->m_AABB = leftChild->m_AABB.Union(rightChild->m_AABB);

                // Rebalance based on configuration
                RotateRebalance(parentNode);

                parentNode = parentNode->m_Parent;
            }
        }
    }

    template <typename T>
    typename BVH<T>::BVHNode* BVH<T>::FindBestSibling(T newObject)
    {
        BVHNode* currentNode = m_Root;

        // Begin query from root.
        while (!currentNode->IsLeaf())
        {
            // Pair the new object with the current node.
            AABB mergedAabb = currentNode->m_AABB.Union(newObject->m_AABB);
            float mergedVolume = mergedAabb.GetVolume();

            auto cost = 2.0 * mergedVolume;
            auto inheritenceCost = 2.0f * (mergedVolume - currentNode->m_AABB.GetVolume());

            // Cost to descend left.
            auto& leftNode = currentNode->m_Children[0];
            mergedAabb = leftNode->m_AABB.Union(newObject->m_AABB);
            auto leftCost = leftNode->IsLeaf() ? mergedAabb.GetVolume() + inheritenceCost : (mergedAabb.GetVolume() - leftNode->m_AABB.GetVolume()) + inheritenceCost;

            // Cost to descend right.
            auto& rightNode = currentNode->m_Children[1];
            mergedAabb = rightNode->m_AABB.Union(newObject->m_AABB);
            auto rightCost = rightNode->IsLeaf() ? mergedAabb.GetVolume() + inheritenceCost : (mergedAabb.GetVolume() - rightNode->m_AABB.GetVolume()) + inheritenceCost;

            // Already descended correctly.
            if ((cost < leftCost) && (cost < rightCost))
            {
                break;
            }

            // Descend further otherwise.
            if (leftCost < rightCost)
            {
                currentNode = currentNode->m_Children[0] ;
            }
            else
            {
                currentNode = currentNode->m_Children[1];
            }
        };

        return currentNode;
    }

    template <typename T>
    void BVH<T>::RotateRebalance(BVHNode* node)
    {
        if (node == nullptr || node->IsLeaf())
        {
            return;
        }

        BVHNode* nodeB = node->m_Children[0];
        BVHNode* nodeC = node->m_Children[1];

        if (nodeB == nullptr || nodeC == nullptr)
        {
            return;
        }

        BVHNode* nodeD = nodeB->m_Children[0];
        BVHNode* nodeE = nodeB->m_Children[1];
        BVHNode* nodeF = nodeC->m_Children[0];
        BVHNode* nodeG = nodeC->m_Children[1];

        float b_cost = nodeB->m_AABB.GetVolume();
        float c_cost = nodeC->m_AABB.GetVolume();

        float cd_cost = (nodeE) ? nodeC->m_AABB.Union(nodeE->m_AABB).GetVolume() - b_cost : std::numeric_limits<float>::max();
        float ce_cost = (nodeD) ? nodeC->m_AABB.Union(nodeD->m_AABB).GetVolume() - b_cost : std::numeric_limits<float>::max();
        float bf_cost = (nodeG) ? nodeB->m_AABB.Union(nodeG->m_AABB).GetVolume() - c_cost : std::numeric_limits<float>::max();
        float bg_cost = (nodeF) ? nodeB->m_AABB.Union(nodeF->m_AABB).GetVolume() - c_cost : std::numeric_limits<float>::max();

        float minCost = std::min({ bf_cost, bg_cost, cd_cost, ce_cost });

        // No rotation gives gain, do not rotate.
        if (minCost >= 0)
        {
            return;
        }

        auto Swap = [&](BVHNode* child, BVHNode* grandchild)
        {
            BVHNode* otherChild = grandchild->m_Parent;
            child->m_Parent = grandchild->m_Parent;
            grandchild->m_Parent = node;

            if (node->m_Children[0] == child)
            {
                node->m_Children[0] = grandchild;
            }
            else
            {
                node->m_Children[1] = grandchild;
            }

            if (otherChild->m_Children[0] == grandchild)
            {
                otherChild->m_Children[0] = child;
            }
            else
            {
                otherChild->m_Children[1] = child;
            }

            otherChild->m_AABB = otherChild->m_Children[0]->m_AABB.Union(otherChild->m_Children[1]->m_AABB);
        };

        if (minCost == bf_cost)
        {
            Swap(nodeB, nodeF); // BF
        }
        else if (minCost == bg_cost)
        {
            Swap(nodeB, nodeG); // BG
        }
        else if (minCost == cd_cost)
        {
            Swap(nodeC, nodeD); // CD
        }
        else if (minCost == ce_cost)
        {
            Swap(nodeC, nodeE); // CE
        }
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
    typename BVH<T>::BVHNode const* BVH<T>::GetRoot() const
    {
        return m_Root;
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