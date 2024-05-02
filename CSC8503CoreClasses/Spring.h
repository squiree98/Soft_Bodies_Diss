#pragma once
#include "ParticleObject.h"

class Spring
{
public:
	Spring(ParticleObject* anchor, ParticleObject* bob, float springConstant, bool debugSpring = false);
	Spring(ParticleObject* anchor, ParticleObject* bob, float springConstant, float restLength, bool debugSpring = false);
	~Spring();

	ParticleObject* GetAnchor() { return mAnchor; }
	ParticleObject* GetBob() { return mBob; }

	void Update(float dt);

protected:

	ParticleObject* mAnchor;
	ParticleObject* mBob;

	float mSpringConstant;

	float mRestLength;

	bool showDebugSpring;

	// to be calculated per frame to determine offset
	Vector3 mCurrentLength;
};
