#include "SoftBodyObject.h"
#include "GeneralUse.h"
#include "RenderObject.h"

#include "Debug.h"
#include "PhysicsObject.h"

SoftBodyObject::SoftBodyObject() {}

SoftBodyObject::SoftBodyObject(NCL::Mesh* mesh, GameWorld* world, NCL::Texture* texture, NCL::Shader* shader, Vector3 position, Vector3 scale, float springStrength, float particleSize)
{
	currentWorld = world;

	springConstant = springStrength;
	particleRadius = particleSize;

	GetTransform().SetPosition(Vector3(0,0,0));
	CreateJoints(mesh, position, scale);
	SetRenderObject(new RenderObject(&GetTransform(), mesh, texture, shader));

	currentWorld->AddGameObject(this);

	GiveShapeVolume();
}

SoftBodyObject::~SoftBodyObject()
{
	for (SoftBodyJoint* joint : allJoints) {
		delete(joint);
	}

	for (Spring* spring : softBodyAllSprings) {
		delete(spring);
	}
}

void SoftBodyObject::UpdateSoftBody(float dt) {
	UpdateSprings(dt);

	UpdateGPUData();

	//UpdateAveragePositionAndAngle();

	ConvertParticlesToVertices();
}

void SoftBodyObject::UpdateSprings(float dt) const {
	for (Spring* x : softBodyAllSprings) {
		x->Update(dt);
	}
}

void SoftBodyObject::UpdateGPUData() {
	unsigned int start = 0;
	unsigned int count = renderObject->GetMesh()->GetPositionData().size();
	renderObject->GetMesh()->UpdateGPUPositionData(start, count);
}

void SoftBodyObject::UpdateAveragePositionAndAngle() {
	Vector3 averagePos = Vector3(0, 0, 0);
	int count = 0;
	for (SoftBodyJoint* joint : allJoints) {
		averagePos += joint->GetTransform().GetPosition();
		count++;
	}
	averagePosition = averagePos / count;

	for (SoftBodyJoint* joint : allJoints) {
		Vector3 jointOffset = joint->GetTransform().GetPosition() - averagePosition;

		Vector3 sideOne = jointOffset - averagePosition;
		Vector3 sideTwo = joint->GetTransform().GetPosition() - averagePosition;
		Vector3 force = (joint->GetBaseOffset() + averagePosition) - joint->GetTransform().GetPosition();

		//joint->GetPhysicsObject()->AddForce(force);
	}
}

void SoftBodyObject::ConvertParticlesToVertices() {
	vector<Vector3> tempVertices(renderObject->GetMesh()->GetPositionData().size());

	for (SoftBodyJoint* joint : softBodyMeshJoints) {
		for (int x : joint->GetVertexIndices()) {
			tempVertices[x] = joint->GetTransform().GetPosition();
		}
	}

	renderObject->GetMesh()->SetVertexPositions(tempVertices);
}

void SoftBodyObject::CreateJoints(NCL::Rendering::Mesh* mesh, Vector3 position, Vector3 scale) {
	CreateBodyVertices(mesh, position, scale);

	CreateBodySprings(mesh);

	for (Spring* spring : softBodyMeshSprings) {
		EnforceMaxSpringLength(spring);
	}

	allJoints = softBodyMeshJoints;

	for (SoftBodyJoint* supportJoint : extraSupportJoints) {
		allJoints.push_back(supportJoint);
	}
}

void SoftBodyObject::CreateBodyVertices(NCL::Mesh* mesh, Vector3 position, Vector3 scale) {
	vector<Vector3> previousPositions;

	int counter = 0;
	// create joints using vertices
	for (Vector3 vertPos : mesh->GetPositionData()) {
		vertPos *= scale;
		vertPos += (position);
		if (!CheckVectorHasValue(previousPositions, vertPos)) {
			// doesn't exist in soft body
			previousPositions.push_back(vertPos);
			SoftBodyJoint* joint = new SoftBodyJoint(vertPos, particleRadius, currentWorld);
			joint->AddVertIndex(counter);
			AddJoint(joint);
		}
		else {
			for (size_t x = 0; x < softBodyMeshJoints.size(); x++) {
				if (softBodyMeshJoints[x]->GetTransform().GetPosition() == vertPos)
					softBodyMeshJoints[x]->AddVertIndex(counter);
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
			SoftBodyJoint* tempJoint1 = GetJointWithVertIndex(mesh->GetIndexData()[counter]);
			SoftBodyJoint* tempJoint2 = GetJointWithVertIndex(mesh->GetIndexData()[counter + 1]);
			bool addSpring = true;
			for (Spring* spring : softBodyMeshSprings) {
				if ((tempJoint1 == spring->GetBob() && tempJoint2 == spring->GetAnchor()) || (tempJoint1 == spring->GetAnchor() && tempJoint2 == spring->GetBob()))
					addSpring = false;
			}
			if (addSpring) {
				Spring* tempSpring = new Spring(tempJoint1, tempJoint2, springConstant);
				softBodyMeshSprings.push_back(tempSpring);
			}
			counter++;
		}
	}
}

void SoftBodyObject::EnforceMaxSpringLength(Spring* spring) {
	if (spring->GetLength() > maxSpringLength) {
		SoftBodyJoint* supportJoint = new SoftBodyJoint(spring->GetMidPoint(), particleRadius, currentWorld);
		Spring* newSpring1 = new Spring(spring->GetAnchor(), supportJoint, springConstant/2, true, NCL::Debug::BLUE);
		Spring* newSpring2 = new Spring(spring->GetBob(), supportJoint, springConstant/2, true, NCL::Debug::GREEN);
		extraSupportJoints.push_back(supportJoint);
		EnforceMaxSpringLength(newSpring1);
		EnforceMaxSpringLength(newSpring2);
	}
	else {
		softBodyAllSprings.push_back(spring);
	}
}

void SoftBodyObject::GiveShapeVolume() {
	CreateTonesOfSprings(true);

	//CreateXYZSprings(true);

	//GetShapeCorners();

	//CreateShapeCornerSprings(true);

	//ConnectShapeCornersToAxisSprings();
}

void SoftBodyObject::CreateTonesOfSprings(bool showSprings) {
	for (SoftBodyJoint* currentJoint : softBodyMeshJoints) {
		SoftBodyJoint* furthestJoint = GetFurthestAwayJoint(currentJoint);
		bool addSpring = true;
		for (Spring* spring : softBodyMeshSprings) {
			if ((currentJoint == spring->GetBob() && furthestJoint == spring->GetAnchor()) || (currentJoint == spring->GetAnchor() && furthestJoint == spring->GetBob()))
				addSpring = false;
		}
		if (addSpring) {
			Spring* tempSpring = new Spring(currentJoint, furthestJoint, springConstant, showSprings, NCL::Debug::YELLOW);
			softBodyAllSprings.push_back(tempSpring);
		}
	}
}

SoftBodyJoint* SoftBodyObject::GetFurthestAwayJoint(SoftBodyJoint* joint) {
	float maxDistance = FLT_MIN;
	SoftBodyJoint* furthestJoint;
	for (SoftBodyJoint* tempJoint : softBodyMeshJoints) {
		if ((joint->GetTransform().GetPosition() - tempJoint->GetTransform().GetPosition()).Length() > maxDistance) {
			furthestJoint = tempJoint;
			maxDistance = (joint->GetTransform().GetPosition() - tempJoint->GetTransform().GetPosition()).Length();
		}
	}
	return furthestJoint;
}

void SoftBodyObject::CreateXYZSprings(bool showSprings) {
	SoftBodyJoint* array[2];

	GetSmallestAndLargestValue('y', array);
	Spring* ySpring = new Spring(array[0], array[1], springConstant, showSprings, NCL::Debug::RED);
	softBodyAllSprings.push_back(ySpring);
	lowMaxJoints["lowY"] = array[0];
	lowMaxJoints["maxY"] = array[1];

	GetSmallestAndLargestValue('x', array);
	Spring* xSpring = new Spring(array[0], array[1], springConstant, showSprings, NCL::Debug::RED);
	softBodyAllSprings.push_back(xSpring);
	lowMaxJoints["lowX"] = array[0];
	lowMaxJoints["maxX"] = array[1];

	GetSmallestAndLargestValue('z', array);
	Spring* zSpring = new Spring(array[0], array[1], springConstant, showSprings, NCL::Debug::RED);
	softBodyAllSprings.push_back(zSpring);
	lowMaxJoints["lowZ"] = array[0];
	lowMaxJoints["maxZ"] = array[1];
}

SoftBodyJoint* SoftBodyObject::GetSmallestAndLargestValue(const char axis, SoftBodyJoint* smallAndLarge[2]) {
	vector<SoftBodyJoint*> smallestVertices;
	smallestVertices.push_back(softBodyMeshJoints[0]);
	vector<SoftBodyJoint*> largestVertices;
	largestVertices.push_back(softBodyMeshJoints[1]);

	for (SoftBodyJoint* joint : allJoints) {
		switch (axis)
		{
		case('x'):
			if (joint->GetTransform().GetPosition().x > largestVertices[0]->GetTransform().GetPosition().x) {
				largestVertices.clear();
				largestVertices.push_back(joint);
			}
			if (joint->GetTransform().GetPosition().x < smallestVertices[0]->GetTransform().GetPosition().x) {
				smallestVertices.clear();
				smallestVertices.push_back(joint);
			}
			if (joint->GetTransform().GetPosition().x == largestVertices[0]->GetTransform().GetPosition().x)
				largestVertices.push_back(joint);
			if (joint->GetTransform().GetPosition().x == smallestVertices[0]->GetTransform().GetPosition().x)
				smallestVertices.push_back(joint);
			break;
		case('y'):
			if (joint->GetTransform().GetPosition().y > largestVertices[0]->GetTransform().GetPosition().y) {
				largestVertices.clear();
				largestVertices.push_back(joint);
			}
			if (joint->GetTransform().GetPosition().y < smallestVertices[0]->GetTransform().GetPosition().y) {
				smallestVertices.clear();
				smallestVertices.push_back(joint);
			}
			if (joint->GetTransform().GetPosition().y == largestVertices[0]->GetTransform().GetPosition().y)
				largestVertices.push_back(joint);
			if (joint->GetTransform().GetPosition().y == smallestVertices[0]->GetTransform().GetPosition().y)
				smallestVertices.push_back(joint);
			break;
		case('z'):
			if (joint->GetTransform().GetPosition().z > largestVertices[0]->GetTransform().GetPosition().z) {
				largestVertices.clear();
				largestVertices.push_back(joint);
			}
			if (joint->GetTransform().GetPosition().z < smallestVertices[0]->GetTransform().GetPosition().z) {
				smallestVertices.clear();
				smallestVertices.push_back(joint);
			}
			if (joint->GetTransform().GetPosition().z == largestVertices[0]->GetTransform().GetPosition().z)
				largestVertices.push_back(joint);
			if (joint->GetTransform().GetPosition().z == smallestVertices[0]->GetTransform().GetPosition().z)
				smallestVertices.push_back(joint);
			break;
		}
	}

	if (smallestVertices.size() == 2 && largestVertices.size() == 2) {
		smallAndLarge[0] = smallestVertices[0];
		smallAndLarge[1] = largestVertices[0];
		return smallAndLarge[0];
	}
	return GetMostCenterJoint(smallestVertices, largestVertices, smallAndLarge);
}

SoftBodyJoint* SoftBodyObject::GetMostCenterJoint(vector<SoftBodyJoint*> smallestJoints, vector<SoftBodyJoint*> largestJoints, SoftBodyJoint* smallAndLarge[2]) {
	SoftBodyJoint* tempJoint = smallestJoints[0];
	for (SoftBodyJoint* joint : smallestJoints) {
		if (joint->GetTransform().GetPosition().Length() < tempJoint->GetTransform().GetPosition().Length()) {
			tempJoint = joint;
		}
	}
	smallAndLarge[0] = tempJoint;

	tempJoint = largestJoints[0];
	for (SoftBodyJoint* joint : largestJoints) {
		if (joint->GetTransform().GetPosition().Length() < tempJoint->GetTransform().GetPosition().Length()) {
			tempJoint = joint;
		}
	}
	smallAndLarge[1] = tempJoint;

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

void SoftBodyObject::CreateShapeCornerSprings(bool showSprings) {
	// diagonal springs
	Spring* supportSpring1 = new Spring(lowMaxJoints.at("botBotLeft"), lowMaxJoints.at("topTopRight"), springConstant, showSprings, NCL::Debug::CYAN);
	softBodyAllSprings.push_back(supportSpring1);

	Spring* supportSpring2 = new Spring(lowMaxJoints.at("topBotLeft"), lowMaxJoints.at("botTopRight"), springConstant, showSprings, NCL::Debug::CYAN);
	softBodyAllSprings.push_back(supportSpring2);

	Spring* supportSpring3 = new Spring(lowMaxJoints.at("botTopLeft"), lowMaxJoints.at("topBotRight"), springConstant, showSprings, NCL::Debug::CYAN);
	softBodyAllSprings.push_back(supportSpring3);

	Spring* supportSpring4 = new Spring(lowMaxJoints.at("botBotRight"), lowMaxJoints.at("topTopLeft"), springConstant, showSprings, NCL::Debug::CYAN);
	softBodyAllSprings.push_back(supportSpring4);
}

void SoftBodyObject::ConnectShapeCornersToAxisSprings() {
	ConnectShapeCornersXAxis(true);
	ConnectShapeCornersYAxis(true);
	ConnectShapeCornersZAxis(true);
}

void SoftBodyObject::ConnectShapeCornersXAxis(bool showSprings) {
	// x axis
	Spring* supportSpring1 = new Spring(lowMaxJoints.at("botBotLeft"), lowMaxJoints.at("lowX"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring1);

	Spring* supportSpring2 = new Spring(lowMaxJoints.at("botTopLeft"), lowMaxJoints.at("lowX"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring2);

	Spring* supportSpring3 = new Spring(lowMaxJoints.at("topBotLeft"), lowMaxJoints.at("lowX"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring3);

	Spring* supportSpring4 = new Spring(lowMaxJoints.at("topTopLeft"), lowMaxJoints.at("lowX"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring4);

	Spring* supportSpring5 = new Spring(lowMaxJoints.at("botBotRight"), lowMaxJoints.at("maxX"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring5);

	Spring* supportSpring6 = new Spring(lowMaxJoints.at("botTopRight"), lowMaxJoints.at("maxX"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring6);

	Spring* supportSpring7 = new Spring(lowMaxJoints.at("topBotRight"), lowMaxJoints.at("maxX"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring7);

	Spring* supportSpring8 = new Spring(lowMaxJoints.at("topTopRight"), lowMaxJoints.at("maxX"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring8);
}

void SoftBodyObject::ConnectShapeCornersYAxis(bool showSprings) {
	// y axis
	Spring* supportSpring1 = new Spring(lowMaxJoints.at("topBotLeft"), lowMaxJoints.at("maxY"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring1);

	Spring* supportSpring2 = new Spring(lowMaxJoints.at("topTopRight"), lowMaxJoints.at("maxY"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring2);

	Spring* supportSpring3 = new Spring(lowMaxJoints.at("topTopLeft"), lowMaxJoints.at("maxY"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring3);

	Spring* supportSpring4 = new Spring(lowMaxJoints.at("topBotRight"), lowMaxJoints.at("maxY"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring4);

	Spring* supportSpring5 = new Spring(lowMaxJoints.at("botBotLeft"), lowMaxJoints.at("lowY"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring5);

	Spring* supportSpring6 = new Spring(lowMaxJoints.at("botTopRight"), lowMaxJoints.at("lowY"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring6);

	Spring* supportSpring7 = new Spring(lowMaxJoints.at("botTopLeft"), lowMaxJoints.at("lowY"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring7);

	Spring* supportSpring8 = new Spring(lowMaxJoints.at("botBotRight"), lowMaxJoints.at("lowY"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring8);
}

void SoftBodyObject::ConnectShapeCornersZAxis(bool showSprings) {
	// z axis
	Spring* supportSpring1 = new Spring(lowMaxJoints.at("botBotLeft"), lowMaxJoints.at("lowZ"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring1);

	Spring* supportSpring2 = new Spring(lowMaxJoints.at("botBotRight"), lowMaxJoints.at("lowZ"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring2);

	Spring* supportSpring3 = new Spring(lowMaxJoints.at("topBotLeft"), lowMaxJoints.at("lowZ"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring3);

	Spring* supportSpring4 = new Spring(lowMaxJoints.at("topBotRight"), lowMaxJoints.at("lowZ"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring4);

	Spring* supportSpring5 = new Spring(lowMaxJoints.at("botTopLeft"), lowMaxJoints.at("maxZ"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring5);

	Spring* supportSpring6 = new Spring(lowMaxJoints.at("botTopRight"), lowMaxJoints.at("maxZ"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring6);

	Spring* supportSpring7 = new Spring(lowMaxJoints.at("topTopLeft"), lowMaxJoints.at("maxZ"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring7);

	Spring* supportSpring8 = new Spring(lowMaxJoints.at("topTopRight"), lowMaxJoints.at("maxZ"), springConstant, showSprings, NCL::Debug::YELLOW);
	softBodyAllSprings.push_back(supportSpring8);
}

SoftBodyJoint* SoftBodyObject::GetShapeCorners(bool lowX, bool lowY, bool lowZ) {
	SoftBodyJoint* temp = softBodyMeshJoints[0];
	float currentLowestPos = FLT_MAX;
	for (SoftBodyJoint* x : softBodyMeshJoints) {
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
	for (SoftBodyJoint* joint : softBodyMeshJoints) {
		for (unsigned int x : joint->GetVertexIndices()) {
			if (x == index)
				return joint;
		}
	}
	return nullptr;
}