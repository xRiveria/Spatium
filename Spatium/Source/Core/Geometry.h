#pragma once
#include <GLM/glm.hpp>

namespace Spatium
{
	struct AABB
	{
	public:
		void Expand(const AABB& other);

		float GetVolume() const;
		float GetSurfaceArea() const;
		glm::vec3 GetCenter() const;

	public:
		glm::vec3 m_Minimum = { 0.0f, 0.0f, 0.0f };
		glm::vec3 m_Maximum = { 0.0f, 0.0f, 0.0f };
	};
}