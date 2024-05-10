#pragma once
#include "GameWorld.h"
#include "ParticleObject.h"

using namespace NCL::CSC8503;

class SoftBodyJoint : public ParticleObject
{
public:
	SoftBodyJoint();
	SoftBodyJoint(Vector3 position, float radius, GameWorld* world);
	~SoftBodyJoint();

	void DrawDebugJoint();

	void AddVertIndex(int index) { vertexIndices.push_back(index); }
	vector<int> GetVertexIndices() { return vertexIndices; }

	Vector3 GetBaseOffset() { return baseOffset; }

protected:
	Vector3 baseOffset;;

	vector<int> vertexIndices;
};