#pragma once

#include "GameObject.h"
#include "SoftBodyJoint.h"
#include "Spring.h"

using namespace NCL::CSC8503;

class SoftBodyObject : public GameObject
{
public:
	SoftBodyObject();
	~SoftBodyObject();

	// will update two things
	// 1- ensure joints match vertices positions
	// 2- update springs on object
	void UpdateSoftBody(float dt);

	void AddJoint(SoftBodyJoint* joint) { softBodyJoints.push_back(joint); }

	void AddSpring(Spring* spring) { softBodySprings.push_back(spring); }

protected:
	// used to ensure mesh vertices are matching joint position
	vector<SoftBodyJoint*> softBodyJoints;
	// used to update springs to move joints
	vector<Spring*> softBodySprings;
};