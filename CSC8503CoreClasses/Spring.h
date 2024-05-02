#pragma once
#include "ParticleObject.h"

class Spring
{
public:
	Spring(ParticleObject* anchor, ParticleObject* bob, float springConstant, float restLength, int ix = 0);
	~Spring();

	ParticleObject* GetAnchor() { return mAnchor; }
	ParticleObject* GetBob() { return mBob; }

	void Update(float dt);

protected:

	ParticleObject* mAnchor;
	ParticleObject* mBob;

	float mSpringConstant;

	float mRestLength;

	int x;

	// to be calculated per frame to determine offset
	Vector3 mCurrentLength;
};
