#include "SoftBodyObject.h"

SoftBodyObject::SoftBodyObject()
{
}

SoftBodyObject::~SoftBodyObject()
{
	for (SoftBodyJoint* joint : softBodyJoints) {
		delete(joint);
	}

	for (Spring* spring : softBodySprings) {
		delete(spring);
	}
}

void SoftBodyObject::UpdateSoftBody(float dt)
{
	for (Spring* x : softBodySprings)
	{
		x->Update(dt);
	}
}
