#pragma once
#include "Debug.h"
#include "ParticleObject.h"

class Spring
{
public:
	Spring(ParticleObject* anchor, ParticleObject* bob, float springConstant, bool debugSpring = false, Vector4 colour = NCL::Debug::MAGENTA);
	Spring(ParticleObject* anchor, ParticleObject* bob, float springConstant, float restLength, bool debugSpring = false, Vector4 colour = NCL::Debug::MAGENTA);
	~Spring();

	ParticleObject* GetAnchor() { return mAnchor; }
	ParticleObject* GetBob() { return mBob; }

	void Update(float dt);

	float GetLength() { return (mBob->GetTransform().GetPosition() - mAnchor->GetTransform().GetPosition()).Length(); }

	Vector3 GetMidPoint();

protected:
	Vector4 springColour;

	ParticleObject* mAnchor;
	ParticleObject* mBob;

	float mSpringConstant;

	float mRestLength;

	bool showDebugSpring;

	// to be calculated per frame to determine offset
	Vector3 mCurrentLength;
};
