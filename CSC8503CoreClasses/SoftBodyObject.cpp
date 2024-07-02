#include "SoftBodyObject.h"

#include <valarray>

#include "GeneralUse.h"
#include "RenderObject.h"

#include "Debug.h"
#include "PhysicsObject.h"

const float supportSpringMultiplier = 1.f;
const float shapeMatchingStrength = 5.f;
const float pressureForceScaler = 125;

SoftBodyObject::SoftBodyObject() {}

SoftBodyObject::SoftBodyObject(SupportMethod supportMethod, NCL::Mesh* mesh, GameWorld* world, NCL::Texture* texture, NCL::Shader* shader, Vector3 position, Vector3 scale, float springStrength, float newMaxSpringLength, float particleSize) {
	supportMethodUsed = supportMethod;
	maxSpringLength = newMaxSpringLength;
	basePosition = position;

	// transform is only used for orientation so it set to default values
	softBodyTransform.SetPosition(Vector3());
	softBodyTransform.SetScale(Vector3(1,1,1));

	currentWorld = world;

	springConstant = springStrength;
	particleRadius = particleSize;

	GetTransform().SetPosition(Vector3(0,0,0));
	CreateJoints(mesh, position, scale);
	SetRenderObject(new RenderObject(&GetTransform(), mesh, texture, shader));

	currentWorld->AddGameObject(this);

	CreateSupportSprings();

	vector<float> volume = GetCurrentPressure();
	initialHeight = volume[0];
	initialWidth = volume[1];
	initialDepth = volume[2];
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

	UpdateAveragePosition();

	if (supportMethodUsed == BasicPressure)
		UpdateBasicPressureModel();

	if (supportMethodUsed == VolumePressure)
		UpdateVolumePressureModel();

	if (supportMethodUsed == SeparateAxisVolumePressure)
		UpdateAxisVolumePressureModel();

	if (supportMethodUsed == ShapeMatching)
		PullJointsToBase();

	ConvertParticlesToVertices();

	loops++;
}

void SoftBodyObject::UpdateSprings(float dt) const {
	for (Spring* x : softBodyAllSprings) {
		x->Update(dt);
	}
}

void SoftBodyObject::UpdateAveragePosition() {
	Vector3 totalPos = Vector3(0, 0, 0);
	for (SoftBodyJoint* joint : allJoints) {
		totalPos += joint->GetTransform().GetPosition();
	}
	averagePosition = totalPos / static_cast<float>(allJoints.size());
}

void SoftBodyObject::UpdateAverageRotation() {
	float averageAngle = 0;
	for (SoftBodyJoint* joint : allJoints) {

		float angleOfRotation = Vector3::Dot(joint->GetBasePosition(), joint->GetTransform().GetPosition() - averagePosition);
		averageAngle += angleOfRotation;
	}

	averageAngle /= static_cast<float>(allJoints.size());

	Quaternion averageRotation = Quaternion::AxisAngleToQuaterion(Vector3(0,1,0), averageAngle);
	softBodyTransform.SetOrientation(averageRotation.Normalised());

	for (SoftBodyJoint* joint : allJoints) {
		Vector4 vec4Pos = Vector4(joint->GetBasePosition(), 1);

		Vector4 transformedPosition = softBodyTransform.GetMatrix() * vec4Pos;

		Vector3 newPos = Vector3(transformedPosition.x, transformedPosition.y, transformedPosition.z);
		joint->SetBasePosition(newPos);
		NCL::Debug::DrawLine(joint->GetTransform().GetPosition(), joint->GetBasePosition() + averagePosition, NCL::Debug::YELLOW);
	}
}

void SoftBodyObject::UpdateGPUData() {
	unsigned int start = 0;
	unsigned int count = renderObject->GetMesh()->GetPositionData().size();
	renderObject->GetMesh()->UpdateGPUPositionData(start, count);
}

void SoftBodyObject::PullJointsToBase() {
	// UpdateAverageRotation();

	for (SoftBodyJoint* joint : allJoints) {
		Vector3 jointCurrentOffset = joint->GetTransform().GetPosition() - (averagePosition + basePosition);
		Vector3 jointBaseOffset = joint->GetBasePosition() - basePosition;

		Vector3 force = jointBaseOffset - jointCurrentOffset;

		joint->GetPhysicsObject()->AddForce(force * shapeMatchingStrength);

		NCL::Debug::DrawLine(joint->GetTransform().GetPosition(), joint->GetTransform().GetPosition() + force, NCL::Debug::YELLOW);
	}
}

void SoftBodyObject::UpdateBasicPressureModel() {
	for (SoftBodyJoint* joint : allJoints) {
		// get outwards direction
		Vector3 force = (joint->GetTransform().GetPosition() - averagePosition).Normalised();

		joint->GetPhysicsObject()->AddForce(force * pressureForceScaler);

		NCL::Debug::DrawLine(joint->GetTransform().GetPosition(), joint->GetTransform().GetPosition() + force, NCL::Debug::RED);
	}
}

void SoftBodyObject::UpdateVolumePressureModel() {
	vector<float> currentVolume = GetCurrentPressure();
	float yPressure = abs(initialHeight - currentVolume[0]);
	float xPressure = abs(initialWidth - currentVolume[1]);
	float zPressure = abs(initialDepth - currentVolume[2]);


	for (SoftBodyJoint* joint : allJoints) {
		// get outwards direction
		Vector3 force = (joint->GetTransform().GetPosition() - averagePosition).Normalised();
		if (xPressure * yPressure * zPressure > initialWidth * initialHeight * initialDepth)
			force = -force;

		joint->GetPhysicsObject()->AddForce(force * pressureForceScaler);

		NCL::Debug::DrawLine(joint->GetTransform().GetPosition(), joint->GetTransform().GetPosition() + force, NCL::Debug::YELLOW);
	}
}

void SoftBodyObject::UpdateAxisVolumePressureModel() {
	vector<float> currentVolume = GetCurrentPressure();
	float yPressure = abs(initialHeight - currentVolume[0]);
	float xPressure = abs(initialWidth - currentVolume[1]);
	float zPressure = abs(initialDepth - currentVolume[2]);

	Vector3 totalForce = Vector3(0, 0, 0);

	for (SoftBodyJoint* joint : allJoints) {
		// get outward direction
		Vector3 force = (joint->GetTransform().GetPosition() - averagePosition).Normalised();

		if (xPressure > initialWidth)
			xPressure = -xPressure;
		if (yPressure > initialHeight)
			yPressure = -yPressure;
		if (zPressure > initialDepth)
			zPressure = -zPressure;


		force.x *= xPressure;
		force.y *= yPressure;
		force.z *= zPressure;

		force.Normalise();

		joint->GetPhysicsObject()->AddForce(force * pressureForceScaler);

		NCL::Debug::DrawLine(joint->GetTransform().GetPosition(), joint->GetTransform().GetPosition() + force, NCL::Debug::BLUE);
	}
}

vector<float> SoftBodyObject::GetCurrentPressure() {
	SoftBodyJoint* array[2];

	if (lowMaxJoints["lowY"] == nullptr) {
		GetSmallestAndLargestValue('y', array);
		lowMaxJoints["lowY"] = array[0];
		lowMaxJoints["maxY"] = array[1];

		GetSmallestAndLargestValue('x', array);
		lowMaxJoints["lowX"] = array[0];
		lowMaxJoints["maxX"] = array[1];

		GetSmallestAndLargestValue('z', array);
		lowMaxJoints["lowZ"] = array[0];
		lowMaxJoints["maxZ"] = array[1];
	}

	float height = lowMaxJoints["maxY"]->GetTransform().GetPosition().y - lowMaxJoints["lowY"]->GetTransform().GetPosition().y;
	float width = lowMaxJoints["maxX"]->GetTransform().GetPosition().x - lowMaxJoints["lowX"]->GetTransform().GetPosition().x;
	float depth = lowMaxJoints["maxZ"]->GetTransform().GetPosition().z - lowMaxJoints["lowZ"]->GetTransform().GetPosition().z;

	vector<float> volume;

	volume.push_back(height);
	volume.push_back(width);
	volume.push_back(depth);

	return volume;
}

void SoftBodyObject::ConvertParticlesToVertices() {
	vector<Vector3> tempVertices(renderObject->GetMesh()->GetPositionData().size());

	for (SoftBodyJoint* joint : allJoints) {
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
			SoftBodyJoint* joint = new SoftBodyJoint(vertPos, particleRadius, currentWorld, basePosition);
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
		SoftBodyJoint* supportJoint = new SoftBodyJoint(spring->GetMidPoint(), particleRadius, currentWorld, basePosition);
		Spring* newSpring1 = new Spring(spring->GetAnchor(), supportJoint, springConstant, true, NCL::Debug::BLUE);
		Spring* newSpring2 = new Spring(spring->GetBob(), supportJoint, springConstant, true, NCL::Debug::GREEN);
		extraSupportJoints.push_back(supportJoint);
		EnforceMaxSpringLength(newSpring1);
		EnforceMaxSpringLength(newSpring2);
	}
	else {
		softBodyAllSprings.push_back(spring);
	}
}

void SoftBodyObject::CreateSupportSprings() {
	springConstant *= supportSpringMultiplier;

	if (supportMethodUsed == SupportMethod::BruteForce)
		BruteForce(true);

	if (supportMethodUsed == SupportMethod::SemiBruteForce)
		SemiBruteForce(true);

	if(supportMethodUsed == Selective)
		SelectiveSupportSprings(true);
}

void SoftBodyObject::BruteForce(bool showSprings) {
	for (SoftBodyJoint* currentJoint : softBodyMeshJoints) {
		for (SoftBodyJoint* nextJoint : softBodyMeshJoints) {
			bool addSpring = true;
			for (Spring* spring : softBodyMeshSprings) {
				if ((currentJoint == spring->GetBob() && nextJoint == spring->GetAnchor()) || (currentJoint == spring->GetAnchor() && nextJoint == spring->GetBob()))
					addSpring = false;
			}
			if (addSpring) {
				Spring* tempSpring = new Spring(currentJoint, nextJoint, springConstant, showSprings, NCL::Debug::YELLOW);
				softBodyAllSprings.push_back(tempSpring);
			}
		}
	}
}

void SoftBodyObject::SemiBruteForce(bool showSprings) {
	for (SoftBodyJoint* currentJoint : allJoints) {
		SoftBodyJoint* furthestJoint = GetFurthestAwayJoint(currentJoint);
		bool addSpring = true;
		for (Spring* spring : softBodyAllSprings) {
			if ((currentJoint == spring->GetBob() && furthestJoint == spring->GetAnchor()) || (currentJoint == spring->GetAnchor() && furthestJoint == spring->GetBob()))
				addSpring = false;
		}
		if (addSpring) {
			Spring* tempSpring = new Spring(currentJoint, furthestJoint, springConstant, showSprings, NCL::Debug::YELLOW);
			softBodyAllSprings.push_back(tempSpring);
		}
	}
}

void SoftBodyObject::SelectiveSupportSprings(bool showSprings) {
	CreateXYZSprings(showSprings);

	GetShapeCorners();

	CreateShapeCornerSprings(showSprings);

	ConnectShapeCornersToAxisSprings();
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
		if ((joint->GetTransform().GetPosition() - basePosition).Length() < (tempJoint->GetTransform().GetPosition() - basePosition).Length()) {
			tempJoint = joint;
		}
	}
	smallAndLarge[0] = tempJoint;

	tempJoint = largestJoints[0];
	for (SoftBodyJoint* joint : largestJoints) {
		if ((joint->GetTransform().GetPosition() - basePosition).Length() < (tempJoint->GetTransform().GetPosition() - basePosition).Length()) {
			tempJoint = joint;
		}
	}
	smallAndLarge[1] = tempJoint;

	return smallAndLarge[0];
}

void SoftBodyObject::GetShapeCorners() {
	SoftBodyJoint* botBotLeft = GetShapeCorner(true, true, true);
	lowMaxJoints["botBotLeft"] = botBotLeft;

	SoftBodyJoint* botTopRight = GetShapeCorner(false, true, false);
	lowMaxJoints["botTopRight"] = botTopRight;

	SoftBodyJoint* botTopLeft = GetShapeCorner(true, true, false);
	lowMaxJoints["botTopLeft"] = botTopLeft;

	SoftBodyJoint* botBotRight = GetShapeCorner(false, true, true);
	lowMaxJoints["botBotRight"] = botBotRight;

	SoftBodyJoint* topBotLeft = GetShapeCorner(true, false, true);
	lowMaxJoints["topBotLeft"] = topBotLeft;

	SoftBodyJoint* topTopRight = GetShapeCorner(false, false, false);
	lowMaxJoints["topTopRight"] = topTopRight;

	SoftBodyJoint* topTopLeft = GetShapeCorner(true, false, false);
	lowMaxJoints["topTopLeft"] = topTopLeft;

	SoftBodyJoint* topBotRight = GetShapeCorner(false, false, true);
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

SoftBodyJoint* SoftBodyObject::GetShapeCorner(bool lowX, bool lowY, bool lowZ) {
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