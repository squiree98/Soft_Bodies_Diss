#pragma once
#include "GameObject.h"

using namespace NCL::CSC8503;

class ParticleObject : public GameObject
{
public:
	ParticleObject();
	ParticleObject(Vector3 position, float radius);
	~ParticleObject();

	const float GetRadius() { return mRadius; }

protected:
	float mRadius;
};