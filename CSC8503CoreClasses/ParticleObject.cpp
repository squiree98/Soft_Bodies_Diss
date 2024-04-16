#include "ParticleObject.h"

ParticleObject::ParticleObject(Vector3 position, float radius)
{
	Vector3 tempPosition = position;
	transform.SetPosition(tempPosition);

	mRadius = radius;
}