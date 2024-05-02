#include "SoftBodyObject.h"
#include "GeneralUse.h"
#include "RenderObject.h"

#include "Debug.h"
#include "PhysicsObject.h"

SoftBodyObject::SoftBodyObject() {}

SoftBodyObject::SoftBodyObject(NCL::Mesh* mesh, GameWorld* world, NCL::Texture* texture, NCL::Shader* shader, Vector3 position, Vector3 scale, float springStrength, float particleSize)
{
	GetTransform().SetPosition(Vector3(0,0,0));
	CreateJoints(mesh, world, position, scale, particleSize, springStrength);
	SetRenderObject(new RenderObject(&GetTransform(), mesh, texture, shader));

	numberOfVertices = mesh->GetPositionData().size();

	world->AddGameObject(this);

	GiveShapeVolume();
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
	UpdateSprings(dt);

	ConvertParticlesToVertices();
}

void SoftBodyObject::UpdateSprings(float dt) const {
	int counter = 0;
	for (Spring* x : softBodySprings) {
		x->Update(dt);
		counter++;
	}
}

void SoftBodyObject::ConvertParticlesToVertices() {
	vector<Vector3> tempVertices(numberOfVertices);

	for (SoftBodyJoint* joint : softBodyJoints) {
		for (int x : joint->GetVertexIndices()) {
			tempVertices[x] = joint->GetTransform().GetPosition();
		}
	}

	renderObject->GetMesh()->SetVertexPositions(tempVertices);
}

void SoftBodyObject::CreateJoints(NCL::Rendering::Mesh* mesh, GameWorld* world , Vector3 position, Vector3 scale, float particleSize, float springStrength) {
	CreateBodyVertices(mesh, world, position, scale, particleSize);

	CreateBodySprings(mesh, springStrength);
}

void SoftBodyObject::CreateBodyVertices(NCL::Mesh* mesh, GameWorld* world, Vector3 position, Vector3 scale, float particleSize) {
	vector<Vector3> previousPositions;

	int counter = 0;
	// create joints using vertices
	for (Vector3 vertPos : mesh->GetPositionData()) {
		Transform tempTransform;
		vertPos *= scale;
		vertPos += (position);
		if (!CheckVectorHasValue(previousPositions, vertPos)) {
			// doesn't exist in soft body
			previousPositions.push_back(vertPos);
			SoftBodyJoint* joint = new SoftBodyJoint(vertPos, particleSize, world);
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

void SoftBodyObject::CreateBodySprings(NCL::Mesh* mesh, float springStrength) {
	int counter = 0;
	// created springs using indices
	for (signed int indicesIndex : mesh->GetIndexData()) {
		if (counter != mesh->GetIndexData().size() - 1)
		{
			SoftBodyJoint* tempJoint1 = GetJointWithVertIndex(mesh->GetIndexData()[counter]);
			SoftBodyJoint* tempJoint2 = GetJointWithVertIndex(mesh->GetIndexData()[counter + 1]);
			bool addSpring = true;
			for (Spring* spring : softBodySprings) {
				if ((tempJoint1 == spring->GetBob() && tempJoint2 == spring->GetAnchor()) || (tempJoint1 == spring->GetAnchor() && tempJoint2 == spring->GetBob()))
					addSpring = false;
			}
			if (addSpring) {
				Spring* tempSpring = new Spring(tempJoint1, tempJoint2, springStrength);
				AddSpring(tempSpring);
			}
			counter++;
		}
	}
}

void SoftBodyObject::GiveShapeVolume() {
	CreateXYZSprings();

	GetShapeCorners();

	CreateShapeCornerSprings();

	ConnectShapeCornersToAxisSprings();
}

void SoftBodyObject::CreateXYZSprings() {
	SoftBodyJoint* array[2];

	GetSmallestAndLargestValue('y', array);
	Spring* ySpring = new Spring(array[0], array[1], 1, true);
	AddSpring(ySpring);
	lowMaxJoints["lowY"] = array[0];
	lowMaxJoints["maxY"] = array[1];

	GetSmallestAndLargestValue('x', array);
	Spring* xSpring = new Spring(array[0], array[1], 1, true);
	AddSpring(xSpring);
	lowMaxJoints["lowX"] = array[0];
	lowMaxJoints["maxX"] = array[1];

	GetSmallestAndLargestValue('z', array);
	Spring* zSpring = new Spring(array[0], array[1], 1, true);
	AddSpring(zSpring);
	lowMaxJoints["lowZ"] = array[0];
	lowMaxJoints["maxZ"] = array[1];
}

SoftBodyJoint* SoftBodyObject::GetSmallestAndLargestValue(const char axis, SoftBodyJoint* smallAndLarge[2]) {
	SoftBodyJoint* tempSmallest = softBodyJoints[0];
	SoftBodyJoint* tempLargest = softBodyJoints[1];

	for (SoftBodyJoint* joint : softBodyJoints) {
		switch (axis)
		{
		case('x'):
			if (joint->GetTransform().GetPosition().x > tempLargest->GetTransform().GetPosition().x)
				tempLargest = joint;
			if (joint->GetTransform().GetPosition().x < tempSmallest->GetTransform().GetPosition().x)
				tempSmallest = joint;
			break;
		case('y'):
			if (joint->GetTransform().GetPosition().y > tempLargest->GetTransform().GetPosition().y)
				tempLargest = joint;
			if (joint->GetTransform().GetPosition().y < tempSmallest->GetTransform().GetPosition().y)
				tempSmallest = joint;
			break;
		case('z'):
			if (joint->GetTransform().GetPosition().z > tempLargest->GetTransform().GetPosition().z)
				tempLargest = joint;
			if (joint->GetTransform().GetPosition().z < tempSmallest->GetTransform().GetPosition().z)
				tempSmallest = joint;
			break;
		}
	}
	smallAndLarge[0] = tempSmallest;
	smallAndLarge[1] = tempLargest;
	return smallAndLarge[0];
}

void SoftBodyObject::GetShapeCorners() {
	SoftBodyJoint* botBotLeft = GetShapeCorners(true, true, true);
	lowMaxJoints["botBotLeft"] = botBotLeft;

	SoftBodyJoint* botTopRight = GetShapeCorners(false, true, false);
	lowMaxJoints["botTopRight"] = botTopRight;

	SoftBodyJoint* botTopLeft = GetShapeCorners(true, true, false);
	lowMaxJoints["botTopLeft"] = botTopLeft;

	SoftBodyJoint* botBotRight = GetShapeCorners(false, true, true);
	lowMaxJoints["botBotRight"] = botBotRight;

	SoftBodyJoint* topBotLeft = GetShapeCorners(true, false, true);
	lowMaxJoints["topBotLeft"] = topBotLeft;

	SoftBodyJoint* topTopRight = GetShapeCorners(false, false, false);
	lowMaxJoints["topTopRight"] = topTopRight;

	SoftBodyJoint* topTopLeft = GetShapeCorners(true, false, false);
	lowMaxJoints["topTopLeft"] = topTopLeft;

	SoftBodyJoint* topBotRight = GetShapeCorners(false, false, true);
	lowMaxJoints["topBotRight"] = topBotRight;
}

void SoftBodyObject::CreateShapeCornerSprings() {
	// diagonal springs
	Spring* supportSpring1 = new Spring(lowMaxJoints.at("botBotLeft"), lowMaxJoints.at("topTopRight"), 1, true);
	AddSpring(supportSpring1);

	Spring* supportSpring2 = new Spring(lowMaxJoints.at("topBotLeft"), lowMaxJoints.at("botTopRight"), 1, true);
	AddSpring(supportSpring2);

	Spring* supportSpring3 = new Spring(lowMaxJoints.at("botTopLeft"), lowMaxJoints.at("topBotRight"), 1, true);
	AddSpring(supportSpring3);

	Spring* supportSpring4 = new Spring(lowMaxJoints.at("botBotRight"), lowMaxJoints.at("topTopLeft"), 1, true);
	AddSpring(supportSpring4);

	// cube springs
	Spring* supportSpring5 = new Spring(lowMaxJoints.at("botBotLeft"), lowMaxJoints.at("topBotLeft"), 1, true);
	AddSpring(supportSpring5);

	Spring* supportSpring6 = new Spring(lowMaxJoints.at("topTopRight"), lowMaxJoints.at("botTopRight"), 1, true);
	AddSpring(supportSpring6);

	Spring* supportSpring7 = new Spring(lowMaxJoints.at("botTopLeft"), lowMaxJoints.at("topTopLeft"), 1, true);
	AddSpring(supportSpring7);

	Spring* supportSpring8 = new Spring(lowMaxJoints.at("botBotRight"), lowMaxJoints.at("topBotRight"), 1, true);
	AddSpring(supportSpring8);
}

void SoftBodyObject::ConnectShapeCornersToAxisSprings() {
	ConnectShapeCornersXAxis();
	ConnectShapeCornersYAxis();
	ConnectShapeCornersZAxis();
}

void SoftBodyObject::ConnectShapeCornersXAxis() {
	// x axis
	Spring* supportSpring1 = new Spring(lowMaxJoints.at("botBotLeft"), lowMaxJoints.at("lowX"), 1, true);
	AddSpring(supportSpring1);

	Spring* supportSpring2 = new Spring(lowMaxJoints.at("botTopLeft"), lowMaxJoints.at("lowX"), 1, true);
	AddSpring(supportSpring2);

	Spring* supportSpring3 = new Spring(lowMaxJoints.at("topBotLeft"), lowMaxJoints.at("lowX"), 1, true);
	AddSpring(supportSpring3);

	Spring* supportSpring4 = new Spring(lowMaxJoints.at("topTopLeft"), lowMaxJoints.at("lowX"), 1, true);
	AddSpring(supportSpring4);

	Spring* supportSpring5 = new Spring(lowMaxJoints.at("botBotRight"), lowMaxJoints.at("maxX"), 1, true);
	AddSpring(supportSpring5);

	Spring* supportSpring6 = new Spring(lowMaxJoints.at("botTopRight"), lowMaxJoints.at("maxX"), 1, true);
	AddSpring(supportSpring6);

	Spring* supportSpring7 = new Spring(lowMaxJoints.at("topBotRight"), lowMaxJoints.at("maxX"), 1, true);
	AddSpring(supportSpring7);

	Spring* supportSpring8 = new Spring(lowMaxJoints.at("topTopRight"), lowMaxJoints.at("maxX"), 1, true);
	AddSpring(supportSpring8);
}

void SoftBodyObject::ConnectShapeCornersYAxis() {
	// y axis
	Spring* supportSpring1 = new Spring(lowMaxJoints.at("topBotLeft"), lowMaxJoints.at("maxY"), 1, true);
	AddSpring(supportSpring1);

	Spring* supportSpring2 = new Spring(lowMaxJoints.at("topTopRight"), lowMaxJoints.at("maxY"), 1, true);
	AddSpring(supportSpring2);

	Spring* supportSpring3 = new Spring(lowMaxJoints.at("topTopLeft"), lowMaxJoints.at("maxY"), 1, true);
	AddSpring(supportSpring3);

	Spring* supportSpring4 = new Spring(lowMaxJoints.at("topBotRight"), lowMaxJoints.at("maxY"), 1, true);
	AddSpring(supportSpring4);

	Spring* supportSpring5 = new Spring(lowMaxJoints.at("botBotLeft"), lowMaxJoints.at("lowY"), 1, true);
	AddSpring(supportSpring5);

	Spring* supportSpring6 = new Spring(lowMaxJoints.at("botTopRight"), lowMaxJoints.at("lowY"), 1, true);
	AddSpring(supportSpring6);

	Spring* supportSpring7 = new Spring(lowMaxJoints.at("botTopLeft"), lowMaxJoints.at("lowY"), 1, true);
	AddSpring(supportSpring7);

	Spring* supportSpring8 = new Spring(lowMaxJoints.at("botBotRight"), lowMaxJoints.at("lowY"), 1, true);
	AddSpring(supportSpring8);
}

void SoftBodyObject::ConnectShapeCornersZAxis() {
	// z axis
	Spring* supportSpring1 = new Spring(lowMaxJoints.at("botBotLeft"), lowMaxJoints.at("lowZ"), 1, true);
	AddSpring(supportSpring1);

	Spring* supportSpring2 = new Spring(lowMaxJoints.at("botBotRight"), lowMaxJoints.at("lowZ"), 1, true);
	AddSpring(supportSpring2);

	Spring* supportSpring3 = new Spring(lowMaxJoints.at("topBotLeft"), lowMaxJoints.at("lowZ"), 1, true);
	AddSpring(supportSpring3);

	Spring* supportSpring4 = new Spring(lowMaxJoints.at("topBotRight"), lowMaxJoints.at("lowZ"), true, true);
	AddSpring(supportSpring4);

	Spring* supportSpring5 = new Spring(lowMaxJoints.at("botTopLeft"), lowMaxJoints.at("maxZ"), 1, true);
	AddSpring(supportSpring5);

	Spring* supportSpring6 = new Spring(lowMaxJoints.at("botTopRight"), lowMaxJoints.at("maxZ"), 1, true);
	AddSpring(supportSpring6);

	Spring* supportSpring7 = new Spring(lowMaxJoints.at("topTopLeft"), lowMaxJoints.at("maxZ"), 1, true);
	AddSpring(supportSpring7);

	Spring* supportSpring8 = new Spring(lowMaxJoints.at("topTopRight"), lowMaxJoints.at("maxZ"), 1, true);
	AddSpring(supportSpring8);
}

SoftBodyJoint* SoftBodyObject::GetShapeCorners(bool lowX, bool lowY, bool lowZ) {
	SoftBodyJoint* temp = softBodyJoints[0];
	float currentLowestPos = FLT_MAX;
	for (SoftBodyJoint* x : softBodyJoints) {
		Vector3 jointPos = x->GetTransform().GetPosition();
		float total = 0;
		switch (lowX) {
		case(true):
			total += jointPos.x;
			break;
		case(false):
			total -= jointPos.x;
			break;
		}
		switch (lowY) {
		case(true):
			total += jointPos.y;
			break;
		case(false):
			total -= jointPos.y;
			break;
		}
		switch (lowZ) {
		case(true):
			total += jointPos.z;
			break;
		case(false):
			total -= jointPos.z;
			break;
		}
		if (total < currentLowestPos) {
			temp = x;
			currentLowestPos = total;
		}
	}
	return temp;
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
