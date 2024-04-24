#include "SoftBodyObject.h"

#include "Debug.h"

SoftBodyObject::SoftBodyObject()
{
	averagePosition = Vector3(0,0,0);
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

void SoftBodyObject::UpdateSoftBody(float dt) {
	UpdateAveragePosition();

	UpdateSprings(dt);
}

void SoftBodyObject::UpdateAveragePosition() {
	int counter = 0;
	averagePosition = Vector3(0,0,0);
	Vector3 base = Vector3(0, 0, 0);
	for (SoftBodyJoint* joint : softBodyJoints)
	{
		base += joint->GetTransform().GetPosition();
		counter++;
	}
	averagePosition = base / counter;
}

void SoftBodyObject::UpdateSprings(float dt) const {
	int counter = 0;
	for (Spring* x : softBodySprings) {
		x->Update(dt);
		counter++;
	}
}

void SoftBodyObject::UpdateJoints() {
	for (SoftBodyJoint* x : softBodyJoints) {
		x->UpdateRelativePos(averagePosition - basePosition);
	}
}

SoftBodyJoint* SoftBodyObject::GetJointWithVertIndex(int index) {
	for (SoftBodyJoint* joint : softBodyJoints) {
		for (unsigned int x : joint->GetVertexIndices()) {
			if (x == index)
				return joint;
		}
	}
	return nullptr;
}
