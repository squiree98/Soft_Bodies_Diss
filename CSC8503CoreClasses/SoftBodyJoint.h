#pragma once
#include "ParticleObject.h"

using namespace NCL::CSC8503;

class SoftBodyJoint : public ParticleObject
{
public:
	SoftBodyJoint(Vector3 position, float radius);
	~SoftBodyJoint();

protected:

};