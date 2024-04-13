#include "Spring.h"

#include "Debug.h"
#include "GameObject.h"

using namespace NCL::CSC8503;

Spring::Spring(SoftBodyJoint* anchor, SoftBodyJoint* bob, float springConstant, Vector3 restLength)
{
	mAnchor = anchor;
	mBob = bob;

	mRestLength = restLength;

	mSpringConstant = springConstant;
}

Spring::~Spring()
{
}

void Spring::Update(float dt)
{
	NCL::Debug::DrawLine(mAnchor->GetPosition(), mBob->GetPosition());

	mCurrentLength = mBob->GetPosition() - mAnchor->GetPosition();
	float displacement = mCurrentLength.Length();
	mCurrentLength.Normalise();
	mCurrentLength *= (-1) * mSpringConstant * displacement;
}