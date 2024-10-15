#include "Quadtree.h"

namespace Spatium
{
	Quadtree::Quadtree(const glm::vec2& treePosition, const glm::vec2& treeSize, int currentLevel, int maxLevel) : m_Position(treePosition), m_Size(treeSize),
																												   m_Level(currentLevel), m_MaxLevel(maxLevel)
	{
		if (m_Level == m_MaxLevel)
		{
			return;
		}

		const float halfWidth = m_Size.x * 0.5f;
		const float halfHeight = m_Size.y * 0.5f;

		m_NorthWest = new Quadtree(m_Size, { halfWidth, halfHeight }, m_Level + 1, maxLevel);
		m_NorthEast = new Quadtree({ m_Position.x + halfWidth, m_Position.y }, { halfWidth, halfHeight }, m_Level + 1, maxLevel);
		m_SouthWest = new Quadtree({ m_Position.x, m_Position.y + halfHeight }, { halfWidth, halfHeight }, m_Level + 1, maxLevel);
		m_SouthEast = new Quadtree({ m_Position.x + halfWidth, m_Position.y + halfHeight }, { halfWidth, halfHeight }, m_Level + 1, maxLevel);
	}

	void Quadtree::Clear()
	{
		if (m_Level < m_MaxLevel)
		{
			m_NorthWest->Clear();
			m_NorthEast->Clear();
			m_SouthWest->Clear();
			m_SouthEast->Clear();
		}

		if (!m_Objects.empty())
		{
			m_Objects.clear();
		}
	}

	std::vector<Quadtree::QuadtreeObject*> Quadtree::GetObjectsInPositionQuadrant(const glm::vec2& position)
	{
		if (m_Level == m_MaxLevel) 
		{
			return m_Objects;
		}

		std::vector<QuadtreeObject*> returnObjects, childReturnObjects;

		if (!m_Objects.empty()) 
		{
			returnObjects = m_Objects;
		}

		if (position.x > m_Position.x + m_Size.x / 2.0f && position.x < m_Position.x + m_Size.x)
		{
			if (position.y > m_Position.y + m_Size.y / 2.0f && position.y < m_Position.y + m_Size.y)
			{
				childReturnObjects = m_SouthEast->GetObjectsInPositionQuadrant(position);
				returnObjects.insert(returnObjects.end(), childReturnObjects.begin(), childReturnObjects.end());
				return returnObjects;
			}
			else if (position.y > m_Position.y && position.y <= m_Position.y + m_Size.y / 2.0f)
			{
				childReturnObjects = m_NorthEast->GetObjectsInPositionQuadrant(position);
				returnObjects.insert(returnObjects.end(), childReturnObjects.begin(), childReturnObjects.end());
				return returnObjects;
			}
		}
		else if (position.x > m_Position.x && position.x <= m_Position.x + m_Size.x / 2.0f)
		{
			if (position.y > m_Position.y + m_Size.y / 2.0f && position.y < m_Position.y + m_Size.y)
			{
				childReturnObjects = m_SouthWest->GetObjectsInPositionQuadrant(position);
				returnObjects.insert(returnObjects.end(), childReturnObjects.begin(), childReturnObjects.end());
				return returnObjects;
			}
			else if (position.y > m_Position.y && position.y <= m_Position.y + m_Size.y / 2.0f) 
			{
				childReturnObjects = m_NorthWest->GetObjectsInPositionQuadrant(position);
				returnObjects.insert(returnObjects.end(), childReturnObjects.begin(), childReturnObjects.end());
				return returnObjects;
			}
		}

		return returnObjects;
	}

	void Quadtree::AddObject(QuadtreeObject* targetObject)
	{
		if (m_Level == m_MaxLevel)
		{
			m_Objects.push_back(targetObject);
		}
		else if (Contains(m_NorthWest, targetObject))
		{
			m_NorthWest->AddObject(targetObject);
		}
		else if (Contains(m_NorthEast, targetObject))
		{
			m_NorthEast->AddObject(targetObject);
		}
		else if (Contains(m_SouthWest, targetObject))
		{
			m_SouthWest->AddObject(targetObject);
		}
		else if (Contains(m_SouthEast, targetObject))
		{
			m_SouthEast->AddObject(targetObject);
		}
		else if (Contains(this, targetObject))
		{
			m_Objects.push_back(targetObject);
		}
	}

	bool Quadtree::Contains(Quadtree* child, QuadtreeObject* targetObject)
	{
		return !(targetObject->m_Position.x < child->m_Position.x ||
				 targetObject->m_Position.y < child->m_Position.y ||
				 targetObject->m_Position.x > child->m_Position.x + child->m_Size.x ||
				 targetObject->m_Position.y > child->m_Position.y + child->m_Size.y ||
				 targetObject->m_Position.x + targetObject->m_Size.x  < child->m_Position.x ||
				 targetObject->m_Position.y + targetObject->m_Size.y  < child->m_Position.y ||
				 targetObject->m_Position.x + targetObject->m_Size.x  > child->m_Position.x + child->m_Size.x ||
				 targetObject->m_Position.y + targetObject->m_Size.y  > child->m_Position.y + child->m_Size.y);
	}
}