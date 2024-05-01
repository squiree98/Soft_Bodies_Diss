#include "SoftBodyObject.h"
#include "GeneralUse.h"
#include "RenderObject.h"

#include "Debug.h"

SoftBodyObject::SoftBodyObject() {}

SoftBodyObject::SoftBodyObject(NCL::Mesh* mesh, GameWorld* world, NCL::Texture* texture, NCL::Shader* shader, Vector3 position, Vector3 scale, float springStrength, float particleSize)
{
	GetTransform().SetPosition(Vector3(0,0,0));
	CreateJoints(mesh, world, position, scale, particleSize, springStrength);
	SetRenderObject(new RenderObject(&GetTransform(), mesh, texture, shader));

	numberOfVertices = mesh->GetPositionData().size();

	world->AddGameObject(this);
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
	int counter = 0;
	Vector3 averagePosition = Vector3(0, 0, 0);
	Vector3 base = Vector3(0, 0, 0);
	for (SoftBodyJoint* joint : softBodyJoints)
	{
		base += joint->GetTransform().GetPosition();
		counter++;
	}
	averagePosition = base / counter;
	std::cout << averagePosition << std::endl;

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

SoftBodyJoint* SoftBodyObject::GetJointWithVertIndex(int index) {
	for (SoftBodyJoint* joint : softBodyJoints) {
		for (unsigned int x : joint->GetVertexIndices()) {
			if (x == index)
				return joint;
		}
	}
	return nullptr;
}
