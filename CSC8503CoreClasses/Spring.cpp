#include "Spring.h"

#include "Debug.h"
#include "GameObject.h"
#include "PhysicsObject.h"

using namespace NCL::CSC8503;

Spring::Spring(ParticleObject* anchor, ParticleObject* bob, float springConstant, bool debugSpring) {
	mAnchor = anchor;
	mBob = bob;

	mRestLength = (anchor->GetTransform().GetPosition() - mBob->GetTransform().GetPosition()).Length();

	mSpringConstant = springConstant;

	showDebugSpring = debugSpring;
}

Spring::Spring(ParticleObject* anchor, ParticleObject* bob, float springConstant, float restLength, bool debugSpring)
{
	mAnchor = anchor;
	mBob = bob;

	mRestLength = restLength;

	mSpringConstant = springConstant;

	showDebugSpring = debugSpring;
}

Spring::~Spring()
{
}

void Spring::Update(float dt)
{
	if (showDebugSpring)
		NCL::Debug::DrawLine(mBob->GetTransform().GetPosition(), mAnchor->GetTransform().GetPosition(), NCL::Debug::MAGENTA);

	mCurrentLength = mBob->GetTransform().GetPosition() - mAnchor->GetTransform().GetPosition();

	if (int(mCurrentLength.Length()) == int(mRestLength))
	{
		mBob->GetPhysicsObject()->ClearForces();
		mAnchor->GetPhysicsObject()->ClearForces();

	}
	else
	{
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
}