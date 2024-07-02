#pragma once
#include "GameWorld.h"
#include "ParticleObject.h"

using namespace NCL::CSC8503;

class SoftBodyJoint : public ParticleObject
{
public:
	SoftBodyJoint();
	SoftBodyJoint(Vector3 position, float radius, GameWorld* world, Vector3 softBodyBasePosition = Vector3(0,0,0), float particleMass = 1.f);
	~SoftBodyJoint();

	void DrawDebugJoint();

	void AddVertIndex(int index) { vertexIndices.push_back(index); }
	vector<int> GetVertexIndices() { return vertexIndices; }

	Vector3 GetBasePosition() { return basePosition; }

	void SetBasePosition(Vector3 newBasePosition) { basePosition = newBasePosition; }

protected:
	Vector3 basePosition;

	vector<int> vertexIndices;

	float xyRotation;
	float xzRotation;
	float yzRotation;
};