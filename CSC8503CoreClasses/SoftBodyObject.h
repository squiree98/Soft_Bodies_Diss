#pragma once

#include "GameObject.h"
#include "Mesh.h"
#include "SoftBodyJoint.h"
#include "Spring.h"
#include "Shader.h"

using namespace NCL::CSC8503;

class SoftBodyObject : public GameObject
{
public:
	SoftBodyObject();
	SoftBodyObject(NCL::Mesh* mesh, GameWorld* world, NCL::Texture* texture, NCL::Shader* shader, Vector3 position = Vector3(0, 0, 0), Vector3 scale = Vector3(1, 1, 1), float particleSize = .5f);
	~SoftBodyObject();

	// will update two things
	// 1- ensure joints match vertices positions
	// 2- update springs on object
	void UpdateSoftBody(float dt);

	vector<SoftBodyJoint*> GetJoints() { return softBodyJoints; }

	vector<Spring*> GetSprings() { return softBodySprings; }

	SoftBodyJoint* GetJointWithVertIndex(int index);

	void AddJoint(SoftBodyJoint* joint) {
		softBodyJoints.push_back(joint);
	}

	void AddSpring(Spring* spring) { softBodySprings.push_back(spring); }

	void CreateJoints(NCL::Rendering::Mesh* mesh, GameWorld* world, Vector3 position, Vector3 scale, float particleSize);

	void CreateBodyVertices(NCL::Mesh* mesh, GameWorld* world, Vector3 position, Vector3 scale, float particleSize);

	void CreateBodySprings(NCL::Mesh* mesh);

protected:

	void UpdateSprings(float dt) const;

	void ConvertParticlesToVertices();

	Vector3 basePosition;

	// used to ensure mesh vertices are matching joint position
	vector<SoftBodyJoint*> softBodyJoints;
	// used to update springs to move joints
	vector<Spring*> softBodySprings;

	int numberOfVertices;
};