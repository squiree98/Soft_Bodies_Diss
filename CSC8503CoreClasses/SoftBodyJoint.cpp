#include "SoftBodyJoint.h"

#include "GameWorld.h"
#include "Debug.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "SphereVolume.h"

SoftBodyJoint::SoftBodyJoint() {
	transform.SetPosition(Vector3(0, 0, 0));
	mRadius = 0;
	basePosition = Vector3(0, 0, 0);
	relativePosition = Vector3(0, 0, 0);
}

SoftBodyJoint::SoftBodyJoint(Vector3 position, float radius, GameWorld* world) {
	Vector3 particleSize = Vector3(radius, radius, radius);
	NCL::SphereVolume* volume = new NCL::SphereVolume(radius, true);
	SetBoundingVolume((NCL::CollisionVolume*)volume);

	GetTransform()
		.SetScale(particleSize)
		.SetPosition(position);

	SetPhysicsObject(new PhysicsObject(&GetTransform(), GetBoundingVolume(), true));

	GetPhysicsObject()->SetInverseMass(1);
	GetPhysicsObject()->InitSphereInertia(false);

	world->AddGameObject(this);

	basePosition = position;
	relativePosition = Vector3(0, 0, 0);

	mRadius = radius;
}

SoftBodyJoint::~SoftBodyJoint() {

}

void SoftBodyJoint::UpdateRelativePos(Vector3 movedDistance) {
	Vector3 relativeBase = basePosition + movedDistance;
	relativePosition = transform.GetPosition() - relativeBase;
}
