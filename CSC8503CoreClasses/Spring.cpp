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
	Vector3 springLength = mBob->GetTransform().GetPosition() - mAnchor->GetTransform().GetPosition();
	if (springLength.y > 0)
		NCL::Debug::DrawLine(mBob->GetTransform().GetPosition(), mAnchor->GetTransform().GetPosition(), NCL::Debug::BLUE);
	else
		NCL::Debug::DrawLine(mBob->GetTransform().GetPosition(), mAnchor->GetTransform().GetPosition(), NCL::Debug::MAGENTA);
	float x = springLength.Length() - mRestLength; // determines how strong the force is (further the spring from rest position, the stronger the force)

	Vector3 force = -1 * mSpringConstant * x;

	mBob->GetPhysicsObject()->ApplyLinearImpulse(force);
}