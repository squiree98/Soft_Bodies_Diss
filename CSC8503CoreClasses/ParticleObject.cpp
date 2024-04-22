#include "ParticleObject.h"

ParticleObject::ParticleObject()
{
	transform.SetPosition(Vector3(0,0,0));
	mRadius = 0;
}

ParticleObject::ParticleObject(Vector3 position, float radius)
{
	Vector3 tempPosition = position;
	transform.SetPosition(tempPosition);

	mRadius = radius;
}

ParticleObject::~ParticleObject()
{
}
