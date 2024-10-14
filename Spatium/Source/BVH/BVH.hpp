#pragma once
#include <cstdint>
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

		template <typename T>
		void BuildTopDown(T itBegin, T itEnd, const BVHBuildConfiguration& buildConfiguration);

		template <typename T>
		void BuildBottomUp(T itBegin, T itEnd, const BVHBuildConfiguration& buildConfiguration);

		template <typename T>
		void Insert(T targetObject, const BVHBuildConfiguration& buildConfiguration);

		void Clear();

		bool IsEmpty() const;
		int GetDepth() const;
		int GetSize() const;
		BVHNode* GetRoot() const;
		uint32_t GetObjectCount() const;

	private:
		BVHNode* m_Root;
		uint32_t m_ObjectCount;
	};
}