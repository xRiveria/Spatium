#include "Geometry.h"

namespace Spatium
{
	void AABB::Expand(const AABB& other)
	{
		m_Minimum = glm::min(m_Minimum, other.m_Minimum);
		m_Maximum = glm::max(m_Maximum, other.m_Maximum);
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
}