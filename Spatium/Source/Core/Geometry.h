#pragma once
#include <GLM/glm.hpp>

namespace Spatium
{
	struct AABB
	{
	public:
		AABB() = default;
		AABB(const glm::vec3& minimum, const glm::vec3& maximum) : m_Minimum(minimum), m_Maximum(maximum) { }

		void Expand(const AABB& other);
		AABB Union(const AABB& other) const;

		float GetVolume() const;
		float GetSurfaceArea() const;
		glm::vec3 GetCenter() const;

	public:
		glm::vec3 m_Minimum = { 0.0f, 0.0f, 0.0f };
		glm::vec3 m_Maximum = { 0.0f, 0.0f, 0.0f };
	};

	struct Triangle
	{
	public:
		Triangle(const glm::vec3& pointA, const glm::vec3& pointB, const glm::vec3& pointC);

		glm::vec3& operator[](int axisIndex);
		const glm::vec3& operator[](int axisIndex) const;

		float GetCenter(uint32_t axisIndex) const;
		glm::vec3 GetMinimumPoint() const;
		glm::vec3 GetMaximumPoint() const;

	public:
		glm::vec3 m_Points[3] = { };
	};
}