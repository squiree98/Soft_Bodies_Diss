#include "ParticleObject.h"
#include "PhysicsObject.h"

ParticleObject::ParticleObject()
{
	transform.SetPosition(Vector3(0,0,0));
	mRadius = 0;
}

ParticleObject::ParticleObject(Vector3 position, float radius, bool applyGravity)
{
	Vector3 tempPosition = position;
	transform.SetPosition(tempPosition);

	SetPhysicsObject(new PhysicsObject(&GetTransform(), GetBoundingVolume(), applyGravity));
	GetPhysicsObject()->SetInverseMass(1.f);
	GetPhysicsObject()->InitSphereInertia(false);

	mRadius = radius;
}

ParticleObject::~ParticleObject()
{
}
