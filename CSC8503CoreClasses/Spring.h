#pragma once
#include "SoftBodyJoint.h"

class Spring
{
public:
	Spring(SoftBodyJoint* anchor, SoftBodyJoint* bob, float springConstant, Vector3 restLength);
	~Spring();

	void Update(float dt);

protected:

	SoftBodyJoint* mAnchor;
	SoftBodyJoint* mBob;

	float mSpringConstant;

	Vector3 mRestLength;

	// to be calculated per frame to determine offset
	Vector3 mCurrentLength;
};
