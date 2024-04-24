#pragma once
#include "ParticleObject.h"

using namespace NCL::CSC8503;

class SoftBodyJoint : public ParticleObject
{
public:
	SoftBodyJoint();
	SoftBodyJoint(Vector3 position, float radius);
	~SoftBodyJoint();

	Vector3 GetRelativePosition() const { return relativePosition; }

	void UpdateRelativePos(Vector3 movedDistance);

protected:
	vector<int> respectiveVertIndices;

	Vector3 basePosition;
	Vector3 relativePosition;;
};