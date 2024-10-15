#include "Core/Geometry.h"

#include <vector>

namespace Spatium
{
	struct KDTreeConfiguration
	{
		float m_TraversalCost = 1.0f;
		float m_IntersectionCost = 80.0f;
		int m_MaxDepth = 50; // The tree should not grow larger than this.
		int m_MinimumTriangles = 50; // Splits should not happen if there are less triangles than this.
		int m_SampleCount = 100;
	};

	class KDTree
	{
	public:
		struct KDTreeNode
		{
		public:
			void SetLeaf(uint32_t firstPrimitiveIndex, uint32_t primitiveCount);
			void SetInternal(uint32_t axisIndex, float splitPoint, unsigned subnodeIndex);

			bool IsLeaf() const { return (m_PrimitiveCount & 0b11) == 0b11; }
			bool IsInternal() const { return !IsLeaf(); }
			uint32_t GetPrimitiveCount() const { return m_PrimitiveCount >> 2; }
			uint32_t GetPrimitiveStartIndex() const { return m_PrimitiveStartIndex; }
			uint32_t GetNextChild() const { return m_SubnodeIndex >> 2; }
			float GetSplitPosition() const { return m_SplitPosition; }
			uint32_t GetSplitAxis() const { return m_SubnodeIndex & 0b11; }

		private:
			union
			{
				float m_SplitPosition; // Internal Nodes Only
				uint32_t m_PrimitiveStartIndex; // Index of first triangle for leafs only.
			};

			union
			{
				uint32_t m_SubnodeIndex; // Subnode Index (30MSB) + Axis (2LSB) for Internal Only
				uint32_t m_PrimitiveCount; // Triangle Count (30MSB) + Leaf Indicator (2LSB) for Leads Only
			};
		};

		void Build(const std::vector<Triangle>& targetTriangles, const KDTreeConfiguration& treeConfiguration);

		// Getters
		const std::vector<KDTreeNode>& GetNodes() const { return m_Nodes; }
		const std::vector<size_t>& GetIndices() const { return m_Indices; }
		const std::vector<AABB>& GetAABBs() const { return m_AABBs; }
		bool IsEmpty() const { return m_Indices.empty(); }

	public:
		void BuildTreeRecursive(const std::vector<Triangle>& targetTriangles, size_t currentNodeIndex, size_t currentDepth);
		float FindBestSplitPoint(const std::vector<Triangle>& targetTriangles, const AABB& aabb, unsigned int axis, unsigned primitiveStartIndex, unsigned primitiveCount);
		float EvaluateSAH(const std::vector<Triangle>& targetTriangles, const AABB& aabb, unsigned axis, float splitPoint, unsigned primitiveStartIndex, unsigned primitiveCount);
		uint32_t PartitionPrimitives(const std::vector<Triangle>& targetTriangles, unsigned axis, float splitPoint, unsigned primitiveStartIndex, unsigned primitiveCount);
		std::vector<size_t> GetTriangles(size_t nodeIndex);
		AABB CalculateEncapsulatingAABB(const std::vector<Triangle>& targetTriangles, const std::vector<size_t>& indices);

	private:
		std::vector<size_t> m_Indices; // All recorded triangles (may contain duplicates).
		std::vector<KDTreeNode> m_Nodes;
		std::vector<AABB> m_AABBs; // AABBs of above nodes in the same order.
		KDTreeConfiguration m_Configuration;

		const float c_Epsilon = 0.001f;
	};
}