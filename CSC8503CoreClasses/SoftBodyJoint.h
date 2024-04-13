#pragma once
#include "GameObject.h"

using namespace NCL::CSC8503;

class SoftBodyJoint
{
public:
	SoftBodyJoint(Vector3 position, float radius);
	~SoftBodyJoint();

	void SetPosition(Vector3 position)
	{
		mPosition = position;
	}

	Vector3 GetPosition()
	{
		return mPosition;
	}

protected:
	Vector3 mPosition;

	float mRadius;

};