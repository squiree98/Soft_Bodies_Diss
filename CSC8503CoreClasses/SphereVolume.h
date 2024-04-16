#pragma once
#include "CollisionVolume.h"

namespace NCL {
	class SphereVolume : CollisionVolume
	{
	public:
		SphereVolume(float sphereRadius = 1.0f) {
			type	= VolumeType::Sphere;
			radius	= sphereRadius;
			applyCollisions = true;
		}
		SphereVolume(float sphereRadius, bool applyCollision) {
			type = VolumeType::Sphere;
			radius = sphereRadius;
			this->applyCollisions = applyCollision;
		}
		~SphereVolume() {}

		float GetRadius() const {
			return radius;
		}

	protected:
		float	radius;
	};
}

