#pragma once
#include <GLM/glm.hpp>
#include <vector >

#include "Core/Geometry.h"

namespace Spatium
{
	class Octree
	{
	public:
		// A point in this case.
		struct OctreeObject
		{
			glm::vec3 m_Position = {};
		};

	public:
		Octree(const glm::vec3& treeOrigin, const glm::vec3& halfDimensions);
		~Octree();

		void Insert(OctreeObject* targetObject);

		bool IsLeafNode() const;
		int GetPointOctant(const glm::vec3& targetObject) const;
		void GetAllObjectsInRange(const AABB& boundingVolume, std::vector<OctreeObject*>& outObjects);

	private:
		glm::vec3 m_Origin = {}; // Physical center of this node.
		glm::vec3 m_HalfDimensions = {}; // In Width/Height/Depth
		
		/*
			Here, - means less than the origin in that dimension, + means greater than.
			Child:	0 1 2 3 4 5 6 7
			X:      - - - - + + + +
			Y:      - - + + - - + +
			Z:      - + - + - + - +
		 */
		Octree* m_Children[8] = { nullptr }; // 8 children at max capacity and can store points depending on overlap.
		OctreeObject* m_Data;
	};
}