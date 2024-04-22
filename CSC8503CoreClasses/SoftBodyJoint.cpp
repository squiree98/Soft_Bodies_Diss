#include "SoftBodyJoint.h"

#include "Debug.h"

SoftBodyJoint::SoftBodyJoint() {
	transform.SetPosition(Vector3(0, 0, 0));
	mRadius = 0;
	basePosition = Vector3(0, 0, 0);
	relativePosition = Vector3(0, 0, 0);
}

SoftBodyJoint::SoftBodyJoint(Vector3 position, float radius) {
	transform.SetPosition(position);
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
