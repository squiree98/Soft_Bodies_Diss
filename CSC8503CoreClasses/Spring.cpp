#include "Spring.h"

#include "Debug.h"
#include "GameObject.h"
#include "GeneralUse.h"
#include "PhysicsObject.h"

using namespace NCL::CSC8503;

Spring::Spring(ParticleObject* anchor, ParticleObject* bob, float springConstant, bool debugSpring, Vector4 colour) {
	mAnchor = anchor;
	mBob = bob;

	mCurrentLength = mBob->GetTransform().GetPosition() - mAnchor->GetTransform().GetPosition();
	mRestLength = mCurrentLength.Length();

	mSpringConstant = springConstant;

	showDebugSpring = debugSpring;

	springColour = colour;
}

Spring::Spring(ParticleObject* anchor, ParticleObject* bob, float springConstant, float restLength, bool debugSpring, Vector4 colour)
{
	mAnchor = anchor;
	mBob = bob;

	mRestLength = restLength;

	mSpringConstant = springConstant;

	showDebugSpring = debugSpring;

	springColour = colour;
}

Spring::~Spring()
{
}

void Spring::Update(float dt) {
	mCurrentLength = mBob->GetTransform().GetPosition() - mAnchor->GetTransform().GetPosition();
	
	float displacement = mCurrentLength.Length() - mRestLength/2;
	displacement = abs(displacement);
	// determine the direction of the springs force
	Vector3 force;
	switch (displacement > mRestLength/2) {
	case(true):
		force = -(mCurrentLength.Normalised());
		//NCL::Debug::DrawLine(mAnchor->GetTransform().GetPosition(), mBob->GetTransform().GetPosition(), NCL::Debug::RED);
		break;
	case(false):
		force = mCurrentLength.Normalised();
		//NCL::Debug::DrawLine(mAnchor->GetTransform().GetPosition(), mBob->GetTransform().GetPosition(), NCL::Debug::BLUE);
		break;
	}

	displacement = mCurrentLength.Length() - mRestLength;
	displacement = abs(displacement);

	force *= mSpringConstant * displacement;

	/*NCL::Debug::Print(std::to_string(mCurrentLength.Length()), Vector2(5,5));
	NCL::Debug::Print(std::to_string(force.Length()), Vector2(5, 10));
	NCL::Debug::Print(std::to_string(displacement), Vector2(5, 15));
	NCL::Debug::Print(std::to_string(mCurrentLength.Length()), Vector2(5, 20));
	NCL::Debug::Print(std::to_string(mRestLength), Vector2(5, 25));*/

	mBob->GetPhysicsObject()->ApplyLinearImpulse(force);
	mAnchor->GetPhysicsObject()->ApplyLinearImpulse(-force);		

	if (showDebugSpring) {
		//NCL::Debug::DrawLine(mBob->GetTransform().GetPosition(), mAnchor->GetTransform().GetPosition(), springColour);
		/*std::cout << "\nDisplacement  : " << displacement << '\n';
		std::cout << "Current Length: " << mCurrentLength.Length() << '\n';
		std::cout << "Rest Length   : " << mRestLength << '\n';*/
	}
}

Vector3 Spring::GetMidPoint() {
	return ((mAnchor->GetTransform().GetPosition() - mBob->GetTransform().GetPosition()) / 2) + mBob->GetTransform().GetPosition();
}
