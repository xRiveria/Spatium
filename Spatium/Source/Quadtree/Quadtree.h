#pragma once
#include <vector>
#include <GLM/glm.hpp>

namespace Spatium
{
	class Quadtree
	{
	public:
		struct QuadtreeObject
		{
			glm::vec2 m_Position;
			glm::vec2 m_Size;
		};

	public:
		Quadtree(const glm::vec2& treePosition, const glm::vec2& treeSize, int currentLevel, int maxLevel);
		~Quadtree();

		void AddObject(QuadtreeObject* targetObject);
		void Clear();

		std::vector<QuadtreeObject*> GetObjectsInPositionQuadrant(const glm::vec2& position);

	private:
		bool Contains(Quadtree* child, QuadtreeObject* targetObject);

	private:
		glm::vec2 m_Position = {};
		glm::vec2 m_Size = {};

		int m_Level = 0;
		int m_MaxLevel = 0;
		std::vector<QuadtreeObject*> m_Objects;

		Quadtree* m_Parent = nullptr;
		Quadtree* m_NorthWest = nullptr; 
		Quadtree* m_NorthEast = nullptr; 
		Quadtree* m_SouthWest = nullptr; 
		Quadtree* m_SouthEast = nullptr; 
	};
}