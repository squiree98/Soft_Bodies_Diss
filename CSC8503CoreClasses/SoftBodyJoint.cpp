#include "SoftBodyJoint.h"

SoftBodyJoint::SoftBodyJoint()
{
	transform.SetPosition(Vector3(0, 0, 0));
	mRadius = 0;
}

SoftBodyJoint::SoftBodyJoint(Vector3 position, float radius)
{
	Vector3 tempPosition = position;
	transform.SetPosition(tempPosition);

	mRadius = radius;
}

SoftBodyJoint::~SoftBodyJoint()
{
}
