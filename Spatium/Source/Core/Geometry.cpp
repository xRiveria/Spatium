#include "Geometry.h"
#include <stdexcept>

namespace Spatium
{
	void AABB::Expand(const AABB& other)
	{
		m_Minimum = glm::min(m_Minimum, other.m_Minimum);
		m_Maximum = glm::max(m_Maximum, other.m_Maximum);
	}

	AABB AABB::Union(const AABB& other) const
	{
		AABB result;
		result.m_Minimum = glm::min(m_Minimum, other.m_Minimum);
		result.m_Maximum = glm::max(m_Maximum, other.m_Maximum);
		return result;
	}

	float AABB::GetVolume() const
	{
		glm::vec3 dimensions = m_Maximum - m_Minimum;
		return dimensions.x * dimensions.y * dimensions.z;
	}

	float AABB::GetSurfaceArea() const
	{
		glm::vec3 dimensions = m_Maximum - m_Minimum;
		return 2.0f * (dimensions.x * dimensions.y + dimensions.x * dimensions.z + dimensions.y * dimensions.z);
	}

	glm::vec3 AABB::GetCenter() const
	{
		return (m_Minimum + m_Maximum) * 0.5f;
	}

	Triangle::Triangle(const glm::vec3& pointA, const glm::vec3& pointB, const glm::vec3& pointC)
	{
		m_Points[0] = pointA;
		m_Points[1] = pointB;
		m_Points[2] = pointC;
	}

	glm::vec3& Triangle::operator[](int axisIndex)
	{
		if (axisIndex < 0 || axisIndex >= 3)
		{
			throw std::runtime_error("Invalid index used to access triangle data!");
		}

		return m_Points[axisIndex];
	}

	const glm::vec3& Triangle::operator[](int axisIndex) const
	{
		if (axisIndex < 0 || axisIndex >= 3)
		{
			throw std::runtime_error("Invalid index used to access triangle data!");
		}

		return m_Points[axisIndex];
	}

	float Triangle::GetCenter(uint32_t axis) const
	{
		int signedAxis = static_cast<int>(axis);
		return (m_Points[0][signedAxis] + m_Points[1][signedAxis] + m_Points[2][signedAxis]) / 3.0f;
	}

	glm::vec3 Triangle::GetMinimumPoint() const
	{
		return glm::min(glm::min(m_Points[0], m_Points[1]), m_Points[2]);
	}

	glm::vec3 Triangle::GetMaximumPoint() const
	{
		return glm::max(glm::max(m_Points[0], m_Points[1]), m_Points[2]);
	}
}