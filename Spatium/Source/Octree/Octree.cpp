#include "Octree.h"

namespace Spatium
{
	Octree::Octree(const glm::vec3& treeOrigin, const glm::vec3& halfDimensions) : m_Origin(treeOrigin), m_HalfDimensions(halfDimensions), m_Data(nullptr)
	{
	}

	Octree::~Octree()
	{
		for (int i = 0; i < 8; ++i)
		{
			delete m_Children[i];
		}
	}

	void Octree::Insert(OctreeObject* targetObject)
	{
		if (IsLeafNode())
		{
			// If this is the first data point, we're done.
			if (m_Data == nullptr)
			{
				m_Data = targetObject;
				return;
			}
			else
			{
				// We're at a leaf, but something is already stored here. Hence, we split into 8 octants and insert the current point as well as the new object.
				OctreeObject* oldObject = m_Data;
				m_Data = nullptr;

				// Split the current node.
				for (int i = 0; i < 8; ++i)
				{
					glm::vec3 newOrigin = m_Origin;
					newOrigin.x += m_HalfDimensions.x * (i & 4 ? 0.5f : -0.5f);
					newOrigin.y += m_HalfDimensions.y * (i & 2 ? 0.5f : -0.5f);
					newOrigin.z += m_HalfDimensions.z * (i & 1 ? 0.5f : -0.5f);

					m_Children[i] = new Octree(newOrigin, m_HalfDimensions * 0.5f);
				}

				// Reinsert the old point, and insert this new point.
				m_Children[GetPointOctant(oldObject->m_Position)]->Insert(oldObject);
				m_Children[GetPointOctant(targetObject->m_Position)]->Insert(targetObject);
			}
		}
		else
		{
			// We're at an interior node. Insert recursively into the appropriate octant accordingly.
			int octant = GetPointOctant(targetObject->m_Position);
			m_Children[octant]->Insert(targetObject);
		}
	}

	bool Octree::IsLeafNode() const
	{
		return m_Children[0] == nullptr; // We're a leaf node if we have no child. Its as straightforward as it gets.
	}

	int Octree::GetPointOctant(const glm::vec3& targetObject) const
	{
		int octantIndex = 0;

		// Manipulate octant index based on the position of the object relative to the origin (front or back, left or right, top or bottom).
		if (targetObject.x >= m_Origin.x)
		{
			octantIndex |= 4;
		}

		if (targetObject.y >= m_Origin.y)
		{
			octantIndex |= 2;
		}

		if (targetObject.z >= m_Origin.z)
		{
			octantIndex |= 1;
		}

		return octantIndex;
	}

	void Octree::GetAllObjectsInRange(const AABB& boundingVolume, std::vector<OctreeObject*>& outObjects)
	{
		// If we're at a leaf node, simply check if the object contained is within the bounding volume.
		if (IsLeafNode()) 
		{
			if (m_Data != NULL) 
			{
				const glm::vec3& containedObjectPosition = m_Data->m_Position;
				
				if (containedObjectPosition.x > boundingVolume.m_Maximum.x || containedObjectPosition.y > boundingVolume.m_Maximum.y || containedObjectPosition.z > boundingVolume.m_Maximum.z) return;
				if (containedObjectPosition.x < boundingVolume.m_Minimum.x || containedObjectPosition.y < boundingVolume.m_Minimum.y || containedObjectPosition.z < boundingVolume.m_Minimum.z) return;
				
				outObjects.push_back(m_Data);
			}
		}
		else 
		{
			// We're at an interior node of the tree. We'll use the bounding volume here as a checking criteria.
			for (int i = 0; i < 8; ++i) 
			{
				// Compute the min/max corners of this child octant.
				glm::vec3 childMax = m_Children[i]->m_Origin + m_Children[i]->m_HalfDimensions;
				glm::vec3 childMin = m_Children[i]->m_Origin - m_Children[i]->m_HalfDimensions;

				// If the queried rectangle is outside the child's bounding box, continue.
				if (childMax.x < boundingVolume.m_Minimum.x || childMax.y < boundingVolume.m_Minimum.y || childMax.z < boundingVolume.m_Minimum.z) continue;
				if (childMin.x > boundingVolume.m_Maximum.x || childMin.y > boundingVolume.m_Maximum.y || childMin.z > boundingVolume.m_Maximum.z) continue;

				// At this point, we've determined that this child is intersecting the query bounding box.
				m_Children[i]->GetAllObjectsInRange(boundingVolume, outObjects);
			}
		}
	}
}