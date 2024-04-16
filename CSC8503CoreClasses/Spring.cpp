#include "Spring.h"

#include "Debug.h"
#include "GameObject.h"
#include "PhysicsObject.h"

using namespace NCL::CSC8503;

Spring::Spring(ParticleObject* anchor, ParticleObject* bob, float springConstant, float restLength)
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
	NCL::Debug::DrawLine(mBob->GetTransform().GetPosition(), mAnchor->GetTransform().GetPosition(), NCL::Debug::MAGENTA);

	mCurrentLength = mBob->GetTransform().GetPosition() - mAnchor->GetTransform().GetPosition();
	float displacement = mCurrentLength.Length() - (mRestLength / 2);
	displacement = abs(displacement);

	// determine the direction of the springs force
	Vector3 force;
	switch (displacement > (mRestLength / 2))
	{
	case(true):
		force = -(mCurrentLength.Normalised());
		break;
	case(false):
		force = mCurrentLength.Normalised();
		break;
	}

	force *= mSpringConstant * displacement;

	mBob->GetPhysicsObject()->ApplyLinearImpulse(force);
	mAnchor->GetPhysicsObject()->ApplyLinearImpulse(-force);
}