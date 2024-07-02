#include "SoftBodyJoint.h"

#include "GameWorld.h"
#include "Debug.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "SphereVolume.h"

SoftBodyJoint::SoftBodyJoint() {
	transform.SetPosition(Vector3(0, 0, 0));
	mRadius = 0;
}

SoftBodyJoint::SoftBodyJoint(Vector3 position, float radius, GameWorld* world, Vector3 softBodyBasePosition) {
	basePosition = position - softBodyBasePosition;
	Vector3 particleSize = Vector3(radius, radius, radius);
	NCL::SphereVolume* volume = new NCL::SphereVolume(radius, true);
	SetBoundingVolume((NCL::CollisionVolume*)volume);

	GetTransform()
		.SetScale(particleSize)
		.SetPosition(position);

	SetPhysicsObject(new PhysicsObject(&GetTransform(), GetBoundingVolume(), true));

	GetPhysicsObject()->SetInverseMass(1.f);
	GetPhysicsObject()->InitSphereInertia(false);

	world->AddGameObject(this);

	mRadius = radius;
}

SoftBodyJoint::~SoftBodyJoint() {

}

void SoftBodyJoint::DrawDebugJoint() {
	NCL::Debug::DrawLine(transform.GetPosition(), Vector3(0, 0, 0));
}
