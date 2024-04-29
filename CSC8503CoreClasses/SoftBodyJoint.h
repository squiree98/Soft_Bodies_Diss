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

	void AddVertIndex(int index) { vertexIndices.push_back(index); }
	vector<int> GetVertexIndices() { return vertexIndices; }

protected:
	
	vector<int> vertexIndices;
};