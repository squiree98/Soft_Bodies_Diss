#pragma once

#include "GameObject.h"
#include "Mesh.h"
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
	void UpdateJoints();

	vector<SoftBodyJoint*> GetJoints() { return softBodyJoints; }
	vector<Spring*> GetSprings() { return softBodySprings; }
	SoftBodyJoint* GetJointWithVertIndex(int index);
	void AddJoint(SoftBodyJoint* joint) {
		softBodyJoints.push_back(joint);
		UpdateAveragePosition();
		basePosition = averagePosition;
	}

	void AddSpring(Spring* spring) { softBodySprings.push_back(spring); }

	void CreateJoints(NCL::Rendering::Mesh* mesh, GameWorld* world);
	void CreateBodyVertices(NCL::Mesh* mesh, GameWorld* world);
	void CreateBodySprings(NCL::Mesh* mesh);

protected:

	void UpdateAveragePosition();

	void UpdateSprings(float dt) const;

	

	Vector3 averagePosition;
	Vector3 basePosition;

	// used to ensure mesh vertices are matching joint position
	vector<SoftBodyJoint*> softBodyJoints;
	// used to update springs to move joints
	vector<Spring*> softBodySprings;
};