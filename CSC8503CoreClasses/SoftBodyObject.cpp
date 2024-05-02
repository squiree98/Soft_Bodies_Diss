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
				Spring* tempSpring = new Spring(tempJoint1, tempJoint2, springStrength, (tempJoint1->GetTransform().GetPosition() - tempJoint2->GetTransform().GetPosition()).Length());
				AddSpring(tempSpring);
			}
			counter++;
		}
	}
}

void SoftBodyObject::GiveShapeVolume() {
	CreateXYZSprings();

	CreateShapeCornerSprings();
}

void SoftBodyObject::CreateXYZSprings() {
	SoftBodyJoint* array[2];

	GetSmallestAndLargestValue('y', array);
	Spring* ySpring = new Spring(array[0], array[1], 1, (array[0]->GetTransform().GetPosition() - array[1]->GetTransform().GetPosition()).Length(), 1);
	AddSpring(ySpring);
	lowMaxJoints["lowY"] = array[0];
	lowMaxJoints["maxY"] = array[1];

	GetSmallestAndLargestValue('x', array);
	Spring* xSpring = new Spring(array[0], array[1], 1, (array[0]->GetTransform().GetPosition() - array[1]->GetTransform().GetPosition()).Length(), 1);
	AddSpring(xSpring);
	lowMaxJoints["lowX"] = array[0];
	lowMaxJoints["maxX"] = array[1];

	GetSmallestAndLargestValue('z', array);
	Spring* zSpring = new Spring(array[0], array[1], 1, (array[0]->GetTransform().GetPosition() - array[1]->GetTransform().GetPosition()).Length(), 1);
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

void SoftBodyObject::CreateShapeCornerSprings() {
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

	// diagonal springs
	Spring* supportSpring1 = new Spring(botBotLeft, topTopRight, 1, (botBotLeft->GetTransform().GetPosition() - topTopRight->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring1);

	Spring* supportSpring2 = new Spring(topBotLeft, botTopRight, 1, (topBotLeft->GetTransform().GetPosition() - botTopRight->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring2);

	Spring* supportSpring3 = new Spring(botTopLeft, topBotRight, 1, (botBotLeft->GetTransform().GetPosition() - topTopRight->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring3);

	Spring* supportSpring4 = new Spring(botBotRight, topTopLeft, 1, (topBotLeft->GetTransform().GetPosition() - botTopRight->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring4);

	// cube springs
	Spring* supportSpring5 = new Spring(botBotLeft, topBotLeft, 1, (botBotLeft->GetTransform().GetPosition() - topBotLeft->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring5);

	Spring* supportSpring6 = new Spring(topTopRight, botTopRight, 1, (topTopRight->GetTransform().GetPosition() - botTopRight->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring6);

	Spring* supportSpring7 = new Spring(botTopLeft, topTopLeft, 1, (botTopLeft->GetTransform().GetPosition() - topTopLeft->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring7);

	Spring* supportSpring8 = new Spring(botBotRight, topBotRight, 1, (botBotRight->GetTransform().GetPosition() - topBotRight->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring8);

	// connect cube to axis springs

	// y axis
	Spring* supportSpring9 = new Spring(topBotLeft, lowMaxJoints.at("maxY"), 1, (topBotLeft->GetTransform().GetPosition() - lowMaxJoints.at("maxY")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring9);

	Spring* supportSpring10 = new Spring(topTopRight, lowMaxJoints.at("maxY"), 1, (topTopRight->GetTransform().GetPosition() - lowMaxJoints.at("maxY")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring10);

	Spring* supportSpring11 = new Spring(topTopLeft, lowMaxJoints.at("maxY"), 1, (topTopLeft->GetTransform().GetPosition() - lowMaxJoints.at("maxY")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring11);

	Spring* supportSpring12 = new Spring(topBotRight, lowMaxJoints.at("maxY"), 1, (topBotRight->GetTransform().GetPosition() - lowMaxJoints.at("maxY")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring12);

	Spring* supportSpring13 = new Spring(botBotLeft, lowMaxJoints.at("lowY"), 1, (botBotLeft->GetTransform().GetPosition() - lowMaxJoints.at("lowY")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring13);

	Spring* supportSpring14 = new Spring(botTopRight, lowMaxJoints.at("lowY"), 1, (botTopRight->GetTransform().GetPosition() - lowMaxJoints.at("lowY")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring14);

	Spring* supportSpring15 = new Spring(botTopLeft, lowMaxJoints.at("lowY"), 1, (botTopLeft->GetTransform().GetPosition() - lowMaxJoints.at("lowY")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring15);

	Spring* supportSpring16 = new Spring(botBotRight, lowMaxJoints.at("lowY"), 1, (botBotRight->GetTransform().GetPosition() - lowMaxJoints.at("lowY")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring16);

	// x axis
	Spring* supportSpring17 = new Spring(botBotLeft, lowMaxJoints.at("lowX"), 1, (botBotLeft->GetTransform().GetPosition() - lowMaxJoints.at("lowX")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring17);

	Spring* supportSpring18 = new Spring(botTopLeft, lowMaxJoints.at("lowX"), 1, (botTopLeft->GetTransform().GetPosition() - lowMaxJoints.at("lowX")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring18);

	Spring* supportSpring19 = new Spring(topBotLeft, lowMaxJoints.at("lowX"), 1, (topBotLeft->GetTransform().GetPosition() - lowMaxJoints.at("lowX")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring19);

	Spring* supportSpring20 = new Spring(topTopLeft, lowMaxJoints.at("lowX"), 1, (topTopLeft->GetTransform().GetPosition() - lowMaxJoints.at("lowX")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring20);

	Spring* supportSpring21 = new Spring(botBotRight, lowMaxJoints.at("maxX"), 1, (botBotRight->GetTransform().GetPosition() - lowMaxJoints.at("maxX")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring21);

	Spring* supportSpring22 = new Spring(botTopRight, lowMaxJoints.at("maxX"), 1, (botTopRight->GetTransform().GetPosition() - lowMaxJoints.at("maxX")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring22);

	Spring* supportSpring23 = new Spring(topBotRight, lowMaxJoints.at("maxX"), 1, (topBotRight->GetTransform().GetPosition() - lowMaxJoints.at("maxX")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring23);

	Spring* supportSpring24 = new Spring(topTopRight, lowMaxJoints.at("maxX"), 1, (topTopRight->GetTransform().GetPosition() - lowMaxJoints.at("maxX")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring24);

	// z axis
	Spring* supportSpring25 = new Spring(botBotLeft, lowMaxJoints.at("lowZ"), 1, (botBotLeft->GetTransform().GetPosition() - lowMaxJoints.at("lowZ")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring25);

	Spring* supportSpring26 = new Spring(botBotRight, lowMaxJoints.at("lowZ"), 1, (botBotRight->GetTransform().GetPosition() - lowMaxJoints.at("lowZ")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring26);

	Spring* supportSpring27 = new Spring(topBotLeft, lowMaxJoints.at("lowZ"), 1, (topBotLeft->GetTransform().GetPosition() - lowMaxJoints.at("lowZ")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring27);

	Spring* supportSpring28 = new Spring(topBotRight, lowMaxJoints.at("lowZ"), 1, (topBotRight->GetTransform().GetPosition() - lowMaxJoints.at("lowZ")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring28);

	Spring* supportSpring29 = new Spring(botTopLeft, lowMaxJoints.at("maxZ"), 1, (botTopLeft->GetTransform().GetPosition() - lowMaxJoints.at("maxZ")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring29);

	Spring* supportSpring30 = new Spring(botTopRight, lowMaxJoints.at("maxZ"), 1, (botTopRight->GetTransform().GetPosition() - lowMaxJoints.at("maxZ")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring30);

	Spring* supportSpring31 = new Spring(topTopLeft, lowMaxJoints.at("maxZ"), 1, (topTopLeft->GetTransform().GetPosition() - lowMaxJoints.at("maxZ")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring31);

	Spring* supportSpring32 = new Spring(topTopRight, lowMaxJoints.at("maxZ"), 1, (topTopRight->GetTransform().GetPosition() - lowMaxJoints.at("maxZ")->GetTransform().GetPosition()).Length(), 1);
	AddSpring(supportSpring32);
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
