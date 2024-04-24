#pragma once
#include "ParticleObject.h"

using namespace NCL::CSC8503;

class SoftBodyJoint : public ParticleObject
{
public:
	SoftBodyJoint();
	SoftBodyJoint(Vector3 position, float radius);
	~SoftBodyJoint();

	Vector3 GetRelativePosition() const { return relativePosition; }

	void UpdateRelativePos(Vector3 movedDistance);

	void AddVertIndex(int index) { vertexIndices.push_back(index); }
	vector<int> GetVertexIndices() { return vertexIndices; }

protected:
	
	vector<int> vertexIndices;

	Vector3 basePosition;
	Vector3 relativePosition;;
};