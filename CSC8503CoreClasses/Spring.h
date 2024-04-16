#pragma once
#include "ParticleObject.h"

class Spring
{
public:
	Spring(ParticleObject* anchor, ParticleObject* bob, float springConstant, float restLength);
	~Spring();

	void Update(float dt);

protected:

	ParticleObject* mAnchor;
	ParticleObject* mBob;

	float mSpringConstant;

	float mRestLength;

	// to be calculated per frame to determine offset
	Vector3 mCurrentLength;
};
