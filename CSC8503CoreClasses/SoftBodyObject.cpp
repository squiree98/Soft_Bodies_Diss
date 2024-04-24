#include "SoftBodyObject.h"
#include "GeneralUse.h"

#include "Debug.h"

SoftBodyObject::SoftBodyObject()
{
	averagePosition = Vector3(0,0,0);
}

SoftBodyObject::~SoftBodyObject()
{
	for (SoftBodyJoint* joint : softBodyJoints) {
		delete(joint);
	}

	for (Spring* spring : softBodySprings) {
		delete(spring);
	}
}

void SoftBodyObject::UpdateSoftBody(float dt) {
	UpdateAveragePosition();

	UpdateSprings(dt);
}

void SoftBodyObject::UpdateAveragePosition() {
	int counter = 0;
	averagePosition = Vector3(0,0,0);
	Vector3 base = Vector3(0, 0, 0);
	for (SoftBodyJoint* joint : softBodyJoints)
	{
		base += joint->GetTransform().GetPosition();
		counter++;
	}
	averagePosition = base / counter;
}

void SoftBodyObject::UpdateSprings(float dt) const {
	int counter = 0;
	for (Spring* x : softBodySprings) {
		x->Update(dt);
		counter++;
	}
}

void SoftBodyObject::CreateJoints(NCL::Rendering::Mesh* mesh, GameWorld* world) {
	CreateBodyVertices(mesh, world);

	CreateBodySprings(mesh);
}

void SoftBodyObject::CreateBodyVertices(NCL::Mesh* mesh, GameWorld* world) {
	vector<Vector3> previousPositions;

	int counter = 0;
	// create joints using vertices
	for (Vector3 vertPos : mesh->GetPositionData()) {
		if (!CheckVectorHasValue(previousPositions, vertPos)) {
			// doesn't exist in soft body
			previousPositions.push_back(vertPos);
			SoftBodyJoint* joint = new SoftBodyJoint(vertPos, 0.1f, world);
			joint->AddVertIndex(counter);
			AddJoint(joint);
		}
		else {
			for (size_t x = 0; x < softBodyJoints.size(); x++) {
				if (softBodyJoints[x]->GetTransform().GetPosition() == vertPos)
					softBodyJoints[x]->AddVertIndex(counter);
			}
		}
		counter++;
	}
}

void SoftBodyObject::CreateBodySprings(NCL::Mesh* mesh) {
	int counter = 0;
	// created springs using indices
	for (signed int indicesIndex : mesh->GetIndexData()) {
		if (counter != mesh->GetIndexData().size() - 1) {
			SoftBodyJoint* tempJoint1 = GetJointWithVertIndex(indicesIndex);
			SoftBodyJoint* tempJoint2 = GetJointWithVertIndex(indicesIndex + 1);
			bool addSpring = true;
			for (Spring* spring : softBodySprings) {
				if ((tempJoint1 == spring->GetBob() && tempJoint2 == spring->GetAnchor()) || (tempJoint1 == spring->GetAnchor() && tempJoint2 == spring->GetBob()))
					addSpring = false;
			}
			if (addSpring) {
				Spring* tempSpring = new Spring(tempJoint1, tempJoint2, 1, 1);
				AddSpring(tempSpring);
			}
		}
		counter++;
	}
}

void SoftBodyObject::UpdateJoints() {
	for (SoftBodyJoint* x : softBodyJoints) {
		x->UpdateRelativePos(averagePosition - basePosition);
	}
}

SoftBodyJoint* SoftBodyObject::GetJointWithVertIndex(int index) {
	for (SoftBodyJoint* joint : softBodyJoints) {
		for (unsigned int x : joint->GetVertexIndices()) {
			if (x == index)
				return joint;
		}
	}
	return nullptr;
}
