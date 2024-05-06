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
	SoftBodyObject(NCL::Mesh* mesh, GameWorld* world, NCL::Texture* texture, NCL::Shader* shader, Vector3 position = Vector3(0, 0, 0), Vector3 scale = Vector3(1, 1, 1), float springStrength = .25f, float particleSize = 1);
	~SoftBodyObject();

	// will update two things
	// 1- ensure joints match vertices positions
	// 2- update springs on object
	void UpdateSoftBody(float dt);

	vector<SoftBodyJoint*> GetJoints() { return softBodyMeshJoints; }

	vector<Spring*> GetSprings() { return softBodySpringsTemp; }

	SoftBodyJoint* GetJointWithVertIndex(int index);

	void AddJoint(SoftBodyJoint* joint) {
		softBodyMeshJoints.push_back(joint);
	}

	void AddSpring(Spring* spring) { softBodySpringsTemp.push_back(spring); }


protected:

	void UpdateSprings(float dt) const;

	void ConvertParticlesToVertices();

	void CreateJoints(NCL::Rendering::Mesh* mesh, Vector3 position, Vector3 scale);

	void CreateBodyVertices(NCL::Mesh* mesh, Vector3 position, Vector3 scale);

	void CreateBodySprings(NCL::Mesh* mesh);

	void EnforceMaxSpringLength(Spring* spring);

	void GiveShapeVolume();

	void CreateXYZSprings(bool showSprings);

	SoftBodyJoint* GetSmallestAndLargestValue(char axis, SoftBodyJoint* smallAndLarge[2]);

	SoftBodyJoint* GetMostCenterJoint(vector<SoftBodyJoint*> smallestJoints, vector<SoftBodyJoint*> largestJoints, SoftBodyJoint* smallAndLarge[2]);

	void GetShapeCorners();

	void CreateShapeCornerSprings(bool showSprings);

	void ConnectShapeCornersToAxisSprings();
	void ConnectShapeCornersXAxis(bool showSprings);
	void ConnectShapeCornersYAxis(bool showSprings);
	void ConnectShapeCornersZAxis(bool showSprings);

	SoftBodyJoint* GetShapeCorners(bool lowX, bool lowY, bool lowZ);

	Vector3 basePosition;

	// used to ensure mesh vertices are matching joint position
	vector<SoftBodyJoint*> softBodyMeshJoints;
	vector<SoftBodyJoint*> extraSupportJoints;
	vector<SoftBodyJoint*> allJoints;

	// highest/lowest joints
	std::map<std::string, SoftBodyJoint*> lowMaxJoints;

	// used to update springs to move joints
	vector<Spring*> softBodySpringsTemp;
	vector<Spring*> softBodySpringsFinal;

	int numberOfVertices;

	GameWorld* currentWorld;

	float particleRadius;
	float springConstant;

	float maxSpringLength = 30.f;
};