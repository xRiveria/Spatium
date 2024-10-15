#ifndef BVH_HPP
#define BVH_HPP

#include <cstdint>
#include <vector>
#include <limits>

#include "Core/Geometry.h"

namespace Spatium
{
	struct BVHBuildConfiguration
	{
		uint32_t m_MaxDepth = std::numeric_limits<uint32_t>::max();
		uint32_t m_MinimumObjects = 20; // Nodes should have more than this amount of objects to be split.
		float m_MinimumVolume = 250.0f; // Nodes with smaller volume than this will not be split.

		const int m_TopDownKSplitPoints = 16;
	};

	template <typename T>
	class BVH
	{
	public:
		struct BVHNode
		{
		public:
			BVHNode();

			void AddObject(T targetObject); // Only Leaf Nodes

			int GetDepth() const;
			int GetSize() const; // Node Count
			bool IsLeaf() const;
			uint32_t GetObjectCount() const;

			template <typename Function>
			void TraverseLevelOrder(Function traversalFunction); // Applies the function to all nodes within this node's subtree.

			template <typename Function>
			void TraverseLevelOrderObjects(Function traversalFunction); // Applies the function to all objects within this node's subtree.

			// Linked list of objects within this node.
			T m_FirstObject;
			T m_LastObject;
			
			AABB m_AABB;
			BVHNode* m_Children[2];
			BVHNode* m_Parent = nullptr;
		};

	public:
		BVH();
		~BVH();

		template <typename Iterator>
		void BuildTopDown(Iterator itBegin, Iterator itEnd, const BVHBuildConfiguration& buildConfiguration);

		template <typename Iterator>
		void BuildBottomUp(Iterator itBegin, Iterator itEnd, const BVHBuildConfiguration& buildConfiguration);

		template <typename Iterator>
		void Insert(Iterator itBegin, Iterator itEnd, const BVHBuildConfiguration& buildConfiguration);

		void Insert(T targetObject, const BVHBuildConfiguration& buildConfiguration);

		template <typename Function> 
		void TraverseLevelOrder(Function traversalFunction) const;

		template <typename Function> 
		void TraverseLevelOrderObjects(Function func) const;

		void Clear();

		bool IsEmpty() const;
		int GetDepth() const;
		int GetSize() const;
		const BVHNode* GetRoot() const;
		uint32_t GetObjectCount() const { return m_ObjectCount; }

	private:
		BVHNode* BuildTopDownRecursive(std::vector<T>& targetObjects, size_t beginIndex, size_t endIndex, const BVHBuildConfiguration& buildConfiguration, uint32_t currentDepth);
		AABB CreateEncapsulatingBoundingVolume(const std::vector<T>& targetObjects, size_t beginIndex, size_t endIndex);
		size_t PartitionObjects(std::vector<T>& targetObjects, size_t beginIndex, size_t endIndex, const BVHBuildConfiguration& buildConfiguration);

		BVHNode* FindBestMergeCandidate(BVHNode* node, const std::vector<BVHNode*>& nodes);
		BVHNode* BuildBottomUpIterative(std::vector<BVHNode*>& objectNodes);
		BVHNode* CreateParentNode(BVHNode* leftNode, BVHNode* rightNode);
		float ComputeBestPairCost(BVHNode* node, const std::vector<BVHNode*>& nodes);

		BVHNode* FindBestSibling(T newObject);
		void RotateRebalance(BVHNode* node);

	private:
		BVHNode* m_Root;
		uint32_t m_ObjectCount;
	};
}

#include "BVH.inl"

#endif