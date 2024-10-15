#include "KDTree.h"

#include <numeric>

namespace Spatium
{
	void KDTree::KDTreeNode::SetLeaf(uint32_t firstPrimitiveIndex, uint32_t primitiveCount)
	{
		m_PrimitiveStartIndex = firstPrimitiveIndex;
		// Keep right most 2 bits for leaf node.
		m_PrimitiveCount = (primitiveCount << 2) | 0b11;
	}

	void KDTree::KDTreeNode::SetInternal(uint32_t axisIndex, float splitPoint, unsigned subnodeIndex)
	{
		m_SplitPosition = splitPoint;
		m_SubnodeIndex = (subnodeIndex << 2) | axisIndex;
	}

	// ====

	void KDTree::Build(const std::vector<Triangle>& targetTriangles, const KDTreeConfiguration& treeConfiguration)
	{
		// Clear
		m_Nodes.clear();
		m_AABBs.clear();
		m_Indices.clear();

		m_Configuration = treeConfiguration;

		// Initialize the root node with all triangles.
		m_Indices.resize(targetTriangles.size());

		// Fill all index values for each triangle beginning with 0 sequentially incrementing.
		std::iota(m_Indices.begin(), m_Indices.end(), 0);

		// Create the root node.
		m_Nodes.emplace_back();

		// First node is a leaf node. We will create a corresponding AABB accordingly.
		m_Nodes.back().SetLeaf(0, static_cast<unsigned int>(targetTriangles.size()));
		m_AABBs.emplace_back(CalculateEncapsulatingAABB(targetTriangles, m_Indices));

		// Recursively build the tree.
		BuildTreeRecursive(targetTriangles, 0, 0);
	}

	void KDTree::BuildTreeRecursive(const std::vector<Triangle>& targetTriangles, size_t currentNodeIndex, size_t currentDepth)
	{
		int primitiveCount = static_cast<int>(m_Nodes[currentNodeIndex].GetPrimitiveCount());
		int primitiveStart = static_cast<int>(m_Nodes[currentNodeIndex].GetPrimitiveStartIndex());

		// Check termination criteria.
		if (primitiveCount <= m_Configuration.m_MinimumTriangles || (m_Configuration.m_MaxDepth != 0 && currentDepth >= m_Configuration.m_MaxDepth))
		{
			return; // Keep as leaf node.
		}

		// Choose the split axis and find the best split point.
		unsigned axis = currentDepth % 3;
		float splitPoint = FindBestSplitPoint(targetTriangles, m_AABBs[currentNodeIndex], axis, primitiveStart, primitiveCount);

		// Partition the primitives
		int midIndex = static_cast<int>(PartitionPrimitives(targetTriangles, axis, splitPoint, primitiveStart, primitiveCount));

		// Create the left child node.
		size_t leftChildIndex = m_Nodes.size();
		m_Nodes.emplace_back();
		m_Nodes.back().SetLeaf(primitiveStart, midIndex - primitiveStart);
		m_AABBs.push_back(CalculateEncapsulatingAABB(targetTriangles, std::vector<size_t>(m_Indices.begin() + primitiveStart, m_Indices.begin() + midIndex)));

		// Recursively build the entire left subtree.
		BuildTreeRecursive(targetTriangles, leftChildIndex, currentDepth + 1);

		// After the entire left subtree is built, create the right child node.
		size_t rightChildIndex = m_Nodes.size();
		m_Nodes.emplace_back();
		m_Nodes.back().SetLeaf(midIndex, primitiveCount - (midIndex - primitiveStart));
		m_AABBs.push_back(CalculateEncapsulatingAABB(targetTriangles, std::vector<size_t>(m_Indices.begin() + midIndex, m_Indices.begin() + primitiveStart + primitiveCount)));

		// Update the current node to be an internal node.
		m_Nodes[currentNodeIndex].SetInternal(axis, splitPoint, static_cast<unsigned int>(rightChildIndex));

		// Recursively build the right subtree.
		BuildTreeRecursive(targetTriangles, rightChildIndex, currentDepth + 1);
	}

	float KDTree::FindBestSplitPoint(const std::vector<Triangle>& targetTriangles, const AABB& aabb, unsigned int axis, unsigned primitiveStartIndex, unsigned primitiveCount)
	{
		int currentAxis = static_cast<int>(axis);

		float bestSplitPoint = 0.0f;
		float bestCost = std::numeric_limits<float>::max();

		// AABB information as needed in the current axis.
		float axisMin = aabb.m_Minimum[currentAxis];
		float axisMax = aabb.m_Maximum[currentAxis];
		float axisExtent = axisMax - axisMin;

		// Try N positions uniformly inside the AABB and record the cheapest one.
		for (int i = 1; i < m_Configuration.m_SampleCount - 1; i++)
		{
			float t = static_cast<float>(i) / m_Configuration.m_SampleCount;
			float splitPoint = axisMin + t * axisExtent;

			// Evaluate the cost at this split.
			float currentCost = EvaluateSAH(targetTriangles, aabb, axis, splitPoint, primitiveStartIndex, primitiveCount);

			if (currentCost < bestCost)
			{
				bestCost = currentCost;
				bestSplitPoint = splitPoint;
			}
		}

		return bestSplitPoint;
	}

	float KDTree::EvaluateSAH(const std::vector<Triangle>& targetTriangles, const AABB& aabb, unsigned axis, float splitPoint, unsigned primitiveStartIndex, unsigned primitiveCount)
	{
		int currentAxis = static_cast<int>(axis);
		// Split into left AABB by setting its max at the split point.
		AABB leftAabb = aabb;
		leftAabb.m_Maximum[currentAxis] = splitPoint;
		float leftArea = leftAabb.GetSurfaceArea();
		float leftObjectCount = 0.0f;

		// Split into right AABB by setting its max to the end of the current AABB.
		AABB rightAabb = aabb;
		rightAabb.m_Minimum[currentAxis] = splitPoint;
		float rightArea = rightAabb.GetSurfaceArea();
		float rightObjectCount = 0.0f;

		// Obtain information needed to calculate heuristics.
		for (size_t i = primitiveStartIndex; i < static_cast<size_t>(primitiveStartIndex + primitiveCount); i++)
		{
			const Triangle& triangle = targetTriangles[m_Indices[i]];

			float triangleMinimumPoint = std::min({ triangle[0][currentAxis], triangle[1][currentAxis], triangle[2][currentAxis] });
			float triangleMaximumPoint = std::max({ triangle[0][currentAxis], triangle[1][currentAxis], triangle[2][currentAxis] });

			if (triangleMaximumPoint <= splitPoint + c_Epsilon)
			{
				leftObjectCount += 1.0f;
			}
			else if (triangleMinimumPoint >= splitPoint - c_Epsilon)
			{
				rightObjectCount += 1.0f;
			}
			else
			{
				// Triangle straddles the split plane.
				float leftPortion = (splitPoint - triangleMinimumPoint) / (triangleMaximumPoint - triangleMinimumPoint);
				float rightPortion = 1.0f - leftPortion;

				leftObjectCount += leftPortion;
				rightObjectCount += rightPortion;
			}
		}

		float totalArea = aabb.GetSurfaceArea();
		float leftAreaCost = leftArea / totalArea;
		float rightAreaCost = rightArea / totalArea;

		return m_Configuration.m_TraversalCost + m_Configuration.m_IntersectionCost * (leftAreaCost * leftObjectCount + rightAreaCost * rightObjectCount);
	}

	uint32_t KDTree::PartitionPrimitives(const std::vector<Triangle>& targetTriangles, unsigned axis, float splitPoint, unsigned primitiveStartIndex, unsigned primitiveCount)
	{
		// Simply partition triangles based on the centroid of each triangle in question in the axis.
		uint32_t middleIndex = primitiveStartIndex;
		for (uint32_t i = primitiveStartIndex; i < primitiveStartIndex + primitiveCount; ++i)
		{

			if (targetTriangles[m_Indices[i]].GetCenter(axis) < splitPoint + c_Epsilon)
			{
				std::swap(m_Indices[middleIndex], m_Indices[i]);
				middleIndex++;
			}
		}

		return middleIndex;
	}

	std::vector<size_t> KDTree::GetTriangles(size_t nodeIndex)
	{
		// Effectively grabs all triangles in the subtree of this node_index parameter.
		const KDTreeNode& currentNode = m_Nodes[nodeIndex];
		int primitiveStartIndex = static_cast<int>(currentNode.GetPrimitiveStartIndex());
		int primitiveCount = static_cast<int>(currentNode.GetPrimitiveCount());

		if (currentNode.IsLeaf())
		{
			return std::vector<size_t>(m_Indices.begin() + primitiveStartIndex, m_Indices.begin() + primitiveStartIndex + primitiveCount);
		}

		std::vector<size_t> triangles;
		size_t left_child_index = nodeIndex + 1;
		size_t right_child_index = currentNode.GetNextChild();

		// Recursively get triangles from the left child.
		std::vector<size_t> left_triangles = GetTriangles(left_child_index);
		triangles.insert(triangles.end(), left_triangles.begin(), left_triangles.end());

		// Recursively get triangles from the right child.
		std::vector<size_t> right_triangles = GetTriangles(right_child_index);
		triangles.insert(triangles.end(), right_triangles.begin(), right_triangles.end());

		return triangles;
	}

	AABB KDTree::CalculateEncapsulatingAABB(const std::vector<Triangle>& targetTriangles, const std::vector<size_t>& indices)
	{
		AABB aabb;

		aabb.m_Minimum = glm::vec3(std::numeric_limits<float>::max());
		aabb.m_Maximum = glm::vec3(-std::numeric_limits<float>::max());

		// Iterate over the triangles and update the AABB.
		for (size_t index : indices)
		{
			const Triangle& triangle = targetTriangles[index];

			// Update minimum and maximum points for each vertex of the triangle.
			for (int i = 0; i < 3; i++)
			{
				aabb.m_Minimum = glm::min(aabb.m_Minimum, triangle[i]);
				aabb.m_Maximum = glm::max(aabb.m_Maximum, triangle[i]);
			}
		}

		return aabb;
	}
}