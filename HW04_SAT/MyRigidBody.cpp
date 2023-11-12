#include "MyRigidBody.h"
using namespace BTX;
//Allocation
uint MyRigidBody::SAT(MyRigidBody* const a_pOther)
{
	// SAT algorithm from Real Time Collision Detection book
	// Some simplifications were made based on matrix multiplication, but the algorithm is mathematically equivalent to the one in the book.

	// Initialize variables used in the algorithm
	vector3 aHalfWidth = GetHalfWidth();
	vector3 bHalfWidth = a_pOther->GetHalfWidth();
	// Truncate the model matrix to a 3x3 rotation matrix
	matrix3 aMat(GetModelMatrix());
	matrix3 bMat(a_pOther->GetModelMatrix());

	// Precompute dot products that will be used in the SAT
	matrix3 rotMat;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			rotMat[i][j] = glm::dot(aMat[i], bMat[j]);
	matrix3 rotMatT = glm::transpose(rotMat);

	matrix3 absRotMat;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			absRotMat[i][j] = glm::abs(rotMat[i][j]) + glm::epsilon<float>();
	matrix3 absRotMatT = glm::transpose(absRotMat);

	// Compute the offset (translation vector) from the center of A to B, in A's local space
	vector3 offset = (a_pOther->GetCenterGlobal() - GetCenterGlobal()) * aMat;

	// Declare radii variables for A and B
	float ra, rb;

	// SAT tests - the first six are simplified using dot products

	// SAT_AX (L = A0)
	ra = aHalfWidth.x;
	rb = glm::dot(bHalfWidth, absRotMat[0]);
	if (glm::abs(offset.x) > ra + rb)
		return BTXs::eSATResults::SAT_AX;

	// SAT_AY (L = A1)
	ra = aHalfWidth.y;
	rb = glm::dot(bHalfWidth, absRotMat[1]);
	if (glm::abs(offset.y) > ra + rb)
		return BTXs::eSATResults::SAT_AY;

	// SAT_AZ (L = A2)
	ra = aHalfWidth.z;
	rb = glm::dot(bHalfWidth, absRotMat[2]);
	if (glm::abs(offset.z) > ra + rb)
		return BTXs::eSATResults::SAT_AZ;

	// SAT_BX (L = B0)
	ra = glm::dot(aHalfWidth, absRotMatT[0]);
	rb = bHalfWidth.x;
	if (glm::abs(glm::dot(offset, rotMatT[0])) > ra + rb)
		return BTXs::eSATResults::SAT_BX;

	// SAT_BY (L = B1)
	ra = glm::dot(aHalfWidth, absRotMatT[1]);
	rb = bHalfWidth.y;
	if (glm::abs(glm::dot(offset, rotMatT[1])) > ra + rb)
		return BTXs::eSATResults::SAT_BY;

	// SAT_BZ (L = B2)
	ra = glm::dot(aHalfWidth, absRotMatT[2]);
	rb = bHalfWidth.z;
	if (glm::abs(glm::dot(offset, rotMatT[2])) > ra + rb)
		return BTXs::eSATResults::SAT_BZ;

	// SAT_AXxBX (L = A0 x B0)
	ra = aHalfWidth.y * absRotMat[2][0] + aHalfWidth.z * absRotMat[1][0];
	rb = bHalfWidth.y * absRotMat[0][2] + bHalfWidth.z * absRotMat[0][1];
	if (glm::abs(offset.z * rotMat[1][0] - offset.y * rotMat[2][0]) > ra + rb)
		return BTXs::eSATResults::SAT_AXxBX;

	// SAT_AXxBY (L = A0 x B1)
	ra = aHalfWidth.y * absRotMat[2][1] + aHalfWidth.z * absRotMat[1][1];
	rb = bHalfWidth.x * absRotMat[0][2] + bHalfWidth.z * absRotMat[0][0];
	if (glm::abs(offset.z * rotMat[1][1] - offset.y * rotMat[2][1]) > ra + rb)
		return BTXs::eSATResults::SAT_AXxBY;

	// SAT_AXxBZ (L = A0 x B2)
	ra = aHalfWidth.y * absRotMat[2][2] + aHalfWidth.z * absRotMat[1][2];
	rb = bHalfWidth.x * absRotMat[0][1] + bHalfWidth.y * absRotMat[0][0];
	if (glm::abs(offset.z * rotMat[1][2] - offset.y * rotMat[2][2]) > ra + rb)
		return BTXs::eSATResults::SAT_AXxBZ;

	// SAT_AYxBX (L = A1 x B0)
	ra = aHalfWidth.x * absRotMat[2][0] + aHalfWidth.z * absRotMat[0][0];
	rb = bHalfWidth.y * absRotMat[1][2] + bHalfWidth.z * absRotMat[1][1];
	if (glm::abs(offset.x * rotMat[2][0] - offset.z * rotMat[0][0]) > ra + rb)
		return BTXs::eSATResults::SAT_AYxBX;

	// SAT_AYxBY (L = A1 x B1)
	ra = aHalfWidth.x * absRotMat[2][1] + aHalfWidth.z * absRotMat[0][1];
	rb = bHalfWidth.x * absRotMat[1][2] + bHalfWidth.z * absRotMat[1][0];
	if (glm::abs(offset.x * rotMat[2][1] - offset.z * rotMat[0][1]) > ra + rb)
		return BTXs::eSATResults::SAT_AYxBY;

	// SAT_AYxBZ (L = A1 x B2)
	ra = aHalfWidth.x * absRotMat[2][2] + aHalfWidth.z * absRotMat[0][2];
	rb = bHalfWidth.x * absRotMat[1][1] + bHalfWidth.y * absRotMat[1][0];
	if (glm::abs(offset.x * rotMat[2][2] - offset.z * rotMat[0][2]) > ra + rb)
		return BTXs::eSATResults::SAT_AYxBZ;

	// SAT_AZxBX (L = A2 x B0)
	ra = aHalfWidth.x * absRotMat[1][0] + aHalfWidth.y * absRotMat[0][0];
	rb = bHalfWidth.y * absRotMat[2][2] + bHalfWidth.z * absRotMat[2][1];
	if (glm::abs(offset.y * rotMat[0][0] - offset.x * rotMat[1][0]) > ra + rb)
		return BTXs::eSATResults::SAT_AZxBX;

	// SAT_AZxBY (L = A2 x B1)
	ra = aHalfWidth.x * absRotMat[1][1] + aHalfWidth.y * absRotMat[0][1];
	rb = bHalfWidth.x * absRotMat[2][2] + bHalfWidth.z * absRotMat[2][0];
	if (glm::abs(offset.y * rotMat[0][1] - offset.x * rotMat[1][1]) > ra + rb)
		return BTXs::eSATResults::SAT_AZxBY;

	// SAT_AZxBZ (L = A2 x B2)
	ra = aHalfWidth.x * absRotMat[1][2] + aHalfWidth.y * absRotMat[0][2];
	rb = bHalfWidth.x * absRotMat[2][1] + bHalfWidth.y * absRotMat[2][0];
	if (glm::abs(offset.y * rotMat[0][2] - offset.x * rotMat[1][2]) > ra + rb)
		return BTXs::eSATResults::SAT_AZxBZ;

	// If no plane could be found to separate the OBBs, return a NONE result
	return BTXs::eSATResults::SAT_NONE;
}
bool MyRigidBody::IsColliding(MyRigidBody* const a_pOther)
{
	//check if spheres are colliding
	bool bColliding = true;
	/*
	* We use Bounding Spheres or ARBB as a pre-test to avoid expensive calculations (SAT)
	* we default bColliding to true here to always fall in the need of calculating
	* SAT for the sake of the assignment.
	*/
	if (bColliding) //they are colliding with bounding sphere
	{
		uint nResult = SAT(a_pOther);
		bColliding = nResult == BTXs::eSATResults::SAT_NONE; // If no plane separating the two OBBs can be found, they are colliding

		if (bColliding) //The SAT shown they are colliding
		{
			this->AddCollisionWith(a_pOther);
			a_pOther->AddCollisionWith(this);
		}
		else //they are not colliding
		{
			this->RemoveCollisionWith(a_pOther);
			a_pOther->RemoveCollisionWith(this);

			// Draw a plane separating the two OBBs, with a normal and color defined by the plane that the SAT found.
			vector3 planeNormal;
			vector3 planeColor;
			switch (nResult)
			{
			case BTXs::eSATResults::SAT_AX:
				planeNormal = vector3(GetModelMatrix()[0]);
				planeColor = C_RED;
				break;
			case BTXs::eSATResults::SAT_AY:
				planeNormal = vector3(GetModelMatrix()[1]);
				planeColor = C_GREEN;
				break;
			case BTXs::eSATResults::SAT_AZ:
				planeNormal = vector3(GetModelMatrix()[2]);
				planeColor = C_BLUE;
				break;
			case BTXs::eSATResults::SAT_BX:
				planeNormal = vector3(a_pOther->GetModelMatrix()[0]);
				planeColor = C_RED;
				break;
			case BTXs::eSATResults::SAT_BY:
				planeNormal = vector3(a_pOther->GetModelMatrix()[1]);
				planeColor = C_GREEN;
				break;
			case BTXs::eSATResults::SAT_BZ:
				planeNormal = vector3(a_pOther->GetModelMatrix()[2]);
				planeColor = C_BLUE;
				break;
			case BTXs::eSATResults::SAT_AXxBX:
				planeNormal = glm::cross(vector3(GetModelMatrix()[0]), vector3(a_pOther->GetModelMatrix()[0]));
				planeColor = C_BLACK;
				break;
			case BTXs::eSATResults::SAT_AXxBY:
				planeNormal = glm::cross(vector3(GetModelMatrix()[0]), vector3(a_pOther->GetModelMatrix()[1]));
				planeColor = C_BROWN;
				break;
			case BTXs::eSATResults::SAT_AXxBZ:
				planeNormal = glm::cross(vector3(GetModelMatrix()[0]), vector3(a_pOther->GetModelMatrix()[2]));
				planeColor = C_CYAN;
				break;
			case BTXs::eSATResults::SAT_AYxBX:
				planeNormal = glm::cross(vector3(GetModelMatrix()[1]), vector3(a_pOther->GetModelMatrix()[0]));
				planeColor = C_MAGENTA;
				break;
			case BTXs::eSATResults::SAT_AYxBY:
				planeNormal = glm::cross(vector3(GetModelMatrix()[1]), vector3(a_pOther->GetModelMatrix()[1]));
				planeColor = C_ORANGE;
				break;
			case BTXs::eSATResults::SAT_AYxBZ:
				planeNormal = glm::cross(vector3(GetModelMatrix()[1]), vector3(a_pOther->GetModelMatrix()[2]));
				planeColor = C_PURPLE;
				break;
			case BTXs::eSATResults::SAT_AZxBX:
				planeNormal = glm::cross(vector3(GetModelMatrix()[2]), vector3(a_pOther->GetModelMatrix()[0]));
				planeColor = C_GRAY;
				break;
			case BTXs::eSATResults::SAT_AZxBY:
				planeNormal = glm::cross(vector3(GetModelMatrix()[2]), vector3(a_pOther->GetModelMatrix()[1]));
				planeColor = C_VIOLET;
				break;
			case BTXs::eSATResults::SAT_AZxBZ:
				planeNormal = glm::cross(vector3(GetModelMatrix()[2]), vector3(a_pOther->GetModelMatrix()[2]));
				planeColor = C_WHITE;
				break;
			}

			// Calculate the transformation matrix for this plane, and add it to the render list (rendering both front and back faces).
			vector3 center = (a_pOther->GetCenterGlobal() + GetCenterGlobal()) / 2; // The solution binary appears to use the middle of the two OBBs' centers
			matrix4 planeMat = glm::translate(center) * glm::toMat4(glm::rotation(vector3(0, 0, 1), planeNormal));
			planeMat = glm::scale(planeMat, vector3(5)); // Scale of 5 (similar to what was used in the solution binary)
			m_pModelMngr->AddPlaneToRenderList(planeMat, planeColor); // Front face
			m_pModelMngr->AddPlaneToRenderList(glm::rotate(planeMat, (float)PI, vector3(0, 1, 0)), planeColor); // Back face

			// Add a sphere to the render list with an inverted color (as is in the solution binary)
			m_pModelMngr->AddSphereToRenderList(glm::scale(glm::translate(center), vector3(0.1f)), C_WHITE - planeColor);
		}
	}
	else //they are not colliding with bounding sphere
	{
		this->RemoveCollisionWith(a_pOther);
		a_pOther->RemoveCollisionWith(this);
	}
	return bColliding;
}
void MyRigidBody::Init(void)
{
	m_pModelMngr = ModelManager::GetInstance();
	m_bVisibleBS = false;
	m_bVisibleOBB = true;
	m_bVisibleARBB = false;

	m_fRadius = 0.0f;

	m_v3ColorColliding = C_RED;
	m_v3ColorNotColliding = C_WHITE;

	m_v3Center = ZERO_V3;
	m_v3MinL = ZERO_V3;
	m_v3MaxL = ZERO_V3;

	m_v3MinG = ZERO_V3;
	m_v3MaxG = ZERO_V3;

	m_v3HalfWidth = ZERO_V3;
	m_v3ARBBSize = ZERO_V3;

	m_m4ToWorld = IDENTITY_M4;
}
void MyRigidBody::Swap(MyRigidBody& a_pOther)
{
	std::swap(m_pModelMngr, a_pOther.m_pModelMngr);
	std::swap(m_bVisibleBS, a_pOther.m_bVisibleBS);
	std::swap(m_bVisibleOBB, a_pOther.m_bVisibleOBB);
	std::swap(m_bVisibleARBB, a_pOther.m_bVisibleARBB);

	std::swap(m_fRadius, a_pOther.m_fRadius);

	std::swap(m_v3ColorColliding, a_pOther.m_v3ColorColliding);
	std::swap(m_v3ColorNotColliding, a_pOther.m_v3ColorNotColliding);

	std::swap(m_v3Center, a_pOther.m_v3Center);
	std::swap(m_v3MinL, a_pOther.m_v3MinL);
	std::swap(m_v3MaxL, a_pOther.m_v3MaxL);

	std::swap(m_v3MinG, a_pOther.m_v3MinG);
	std::swap(m_v3MaxG, a_pOther.m_v3MaxG);

	std::swap(m_v3HalfWidth, a_pOther.m_v3HalfWidth);
	std::swap(m_v3ARBBSize, a_pOther.m_v3ARBBSize);

	std::swap(m_m4ToWorld, a_pOther.m_m4ToWorld);

	std::swap(m_CollidingRBSet, a_pOther.m_CollidingRBSet);
}
void MyRigidBody::Release(void)
{
	m_pModelMngr = nullptr;
	ClearCollidingList();
}
//Accessors
bool MyRigidBody::GetVisibleBS(void) { return m_bVisibleBS; }
void MyRigidBody::SetVisibleBS(bool a_bVisible) { m_bVisibleBS = a_bVisible; }
bool MyRigidBody::GetVisibleOBB(void) { return m_bVisibleOBB; }
void MyRigidBody::SetVisibleOBB(bool a_bVisible) { m_bVisibleOBB = a_bVisible; }
bool MyRigidBody::GetVisibleARBB(void) { return m_bVisibleARBB; }
void MyRigidBody::SetVisibleARBB(bool a_bVisible) { m_bVisibleARBB = a_bVisible; }
float MyRigidBody::GetRadius(void) { return m_fRadius; }
vector3 MyRigidBody::GetColorColliding(void) { return m_v3ColorColliding; }
vector3 MyRigidBody::GetColorNotColliding(void) { return m_v3ColorNotColliding; }
void MyRigidBody::SetColorColliding(vector3 a_v3Color) { m_v3ColorColliding = a_v3Color; }
void MyRigidBody::SetColorNotColliding(vector3 a_v3Color) { m_v3ColorNotColliding = a_v3Color; }
vector3 MyRigidBody::GetCenterLocal(void) { return m_v3Center; }
vector3 MyRigidBody::GetMinLocal(void) { return m_v3MinL; }
vector3 MyRigidBody::GetMaxLocal(void) { return m_v3MaxL; }
vector3 MyRigidBody::GetCenterGlobal(void) { return vector3(m_m4ToWorld * vector4(m_v3Center, 1.0f)); }
vector3 MyRigidBody::GetMinGlobal(void) { return m_v3MinG; }
vector3 MyRigidBody::GetMaxGlobal(void) { return m_v3MaxG; }
vector3 MyRigidBody::GetHalfWidth(void) { return m_v3HalfWidth; }
matrix4 MyRigidBody::GetModelMatrix(void) { return m_m4ToWorld; }
void MyRigidBody::SetModelMatrix(matrix4 a_m4ModelMatrix)
{
	//to save some calculations if the model matrix is the same there is nothing to do here
	if (a_m4ModelMatrix == m_m4ToWorld)
		return;

	//Assign the model matrix
	m_m4ToWorld = a_m4ModelMatrix;

	//Calculate the 8 corners of the cube
	vector3 v3Corner[8];
	//Back square
	v3Corner[0] = m_v3MinL;
	v3Corner[1] = vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MinL.z);
	v3Corner[2] = vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MinL.z);
	v3Corner[3] = vector3(m_v3MaxL.x, m_v3MaxL.y, m_v3MinL.z);

	//Front square
	v3Corner[4] = vector3(m_v3MinL.x, m_v3MinL.y, m_v3MaxL.z);
	v3Corner[5] = vector3(m_v3MaxL.x, m_v3MinL.y, m_v3MaxL.z);
	v3Corner[6] = vector3(m_v3MinL.x, m_v3MaxL.y, m_v3MaxL.z);
	v3Corner[7] = m_v3MaxL;

	//Place them in world space
	for (uint uIndex = 0; uIndex < 8; ++uIndex)
	{
		v3Corner[uIndex] = vector3(m_m4ToWorld * vector4(v3Corner[uIndex], 1.0f));
	}

	//Identify the max and min as the first corner
	m_v3MaxG = m_v3MinG = v3Corner[0];

	//get the new max and min for the global box
	for (uint i = 1; i < 8; ++i)
	{
		if (m_v3MaxG.x < v3Corner[i].x) m_v3MaxG.x = v3Corner[i].x;
		else if (m_v3MinG.x > v3Corner[i].x) m_v3MinG.x = v3Corner[i].x;

		if (m_v3MaxG.y < v3Corner[i].y) m_v3MaxG.y = v3Corner[i].y;
		else if (m_v3MinG.y > v3Corner[i].y) m_v3MinG.y = v3Corner[i].y;

		if (m_v3MaxG.z < v3Corner[i].z) m_v3MaxG.z = v3Corner[i].z;
		else if (m_v3MinG.z > v3Corner[i].z) m_v3MinG.z = v3Corner[i].z;
	}

	//we calculate the distance between min and max vectors
	m_v3ARBBSize = m_v3MaxG - m_v3MinG;
}
//The big 3
MyRigidBody::MyRigidBody(std::vector<vector3> a_pointList)
{
	Init();
	//Count the points of the incoming list
	uint uVertexCount = a_pointList.size();

	//If there are none just return, we have no information to create the BS from
	if (uVertexCount == 0)
		return;

	//Max and min as the first vector of the list
	m_v3MaxL = m_v3MinL = a_pointList[0];

	//Get the max and min out of the list
	for (uint i = 1; i < uVertexCount; ++i)
	{
		if (m_v3MaxL.x < a_pointList[i].x) m_v3MaxL.x = a_pointList[i].x;
		else if (m_v3MinL.x > a_pointList[i].x) m_v3MinL.x = a_pointList[i].x;

		if (m_v3MaxL.y < a_pointList[i].y) m_v3MaxL.y = a_pointList[i].y;
		else if (m_v3MinL.y > a_pointList[i].y) m_v3MinL.y = a_pointList[i].y;

		if (m_v3MaxL.z < a_pointList[i].z) m_v3MaxL.z = a_pointList[i].z;
		else if (m_v3MinL.z > a_pointList[i].z) m_v3MinL.z = a_pointList[i].z;
	}

	//with model matrix being the identity, local and global are the same
	m_v3MinG = m_v3MinL;
	m_v3MaxG = m_v3MaxL;

	//with the max and the min we calculate the center
	m_v3Center = (m_v3MaxL + m_v3MinL) / 2.0f;

	//we calculate the distance between min and max vectors
	m_v3HalfWidth = (m_v3MaxL - m_v3MinL) / 2.0f;

	//Get the distance between the center and either the min or the max
	m_fRadius = glm::distance(m_v3Center, m_v3MinL);
}
MyRigidBody::MyRigidBody(MyRigidBody const& a_pOther)
{
	m_pModelMngr = a_pOther.m_pModelMngr;

	m_bVisibleBS = a_pOther.m_bVisibleBS;
	m_bVisibleOBB = a_pOther.m_bVisibleOBB;
	m_bVisibleARBB = a_pOther.m_bVisibleARBB;

	m_fRadius = a_pOther.m_fRadius;

	m_v3ColorColliding = a_pOther.m_v3ColorColliding;
	m_v3ColorNotColliding = a_pOther.m_v3ColorNotColliding;

	m_v3Center = a_pOther.m_v3Center;
	m_v3MinL = a_pOther.m_v3MinL;
	m_v3MaxL = a_pOther.m_v3MaxL;

	m_v3MinG = a_pOther.m_v3MinG;
	m_v3MaxG = a_pOther.m_v3MaxG;

	m_v3HalfWidth = a_pOther.m_v3HalfWidth;
	m_v3ARBBSize = a_pOther.m_v3ARBBSize;

	m_m4ToWorld = a_pOther.m_m4ToWorld;

	m_CollidingRBSet = a_pOther.m_CollidingRBSet;
}
MyRigidBody& MyRigidBody::operator=(MyRigidBody const& a_pOther)
{
	if (this != &a_pOther)
	{
		Release();
		Init();
		MyRigidBody temp(a_pOther);
		Swap(temp);
	}
	return *this;
}
MyRigidBody::~MyRigidBody() { Release(); };

//--- a_pOther Methods
void MyRigidBody::AddCollisionWith(MyRigidBody* a_pOther)
{
	/*
		check if the object is already in the colliding set, if
		the object is already there return with no changes
	*/
	auto element = m_CollidingRBSet.find(a_pOther);
	if (element != m_CollidingRBSet.end())
		return;
	// we couldn't find the object so add it
	m_CollidingRBSet.insert(a_pOther);
}
void MyRigidBody::RemoveCollisionWith(MyRigidBody* a_pOther)
{
	m_CollidingRBSet.erase(a_pOther);
}
void MyRigidBody::ClearCollidingList(void)
{
	m_CollidingRBSet.clear();
}

void MyRigidBody::AddToRenderList(void)
{
	if (m_bVisibleBS)
	{
		if (m_CollidingRBSet.size() > 0)
			m_pModelMngr->AddWireSphereToRenderList(glm::translate(m_m4ToWorld, m_v3Center) * glm::scale(vector3(m_fRadius)), C_BLUE_CORNFLOWER);
		else
			m_pModelMngr->AddWireSphereToRenderList(glm::translate(m_m4ToWorld, m_v3Center) * glm::scale(vector3(m_fRadius)), C_BLUE_CORNFLOWER);
	}
	if (m_bVisibleOBB)
	{
		if (m_CollidingRBSet.size() > 0)
			m_pModelMngr->AddWireCubeToRenderList(glm::translate(m_m4ToWorld, m_v3Center) * glm::scale(m_v3HalfWidth * 2.0f), m_v3ColorColliding);
		else
			m_pModelMngr->AddWireCubeToRenderList(glm::translate(m_m4ToWorld, m_v3Center) * glm::scale(m_v3HalfWidth * 2.0f), m_v3ColorNotColliding);
	}
	if (m_bVisibleARBB)
	{
		if (m_CollidingRBSet.size() > 0)
			m_pModelMngr->AddWireCubeToRenderList(glm::translate(GetCenterGlobal()) * glm::scale(m_v3ARBBSize), C_YELLOW);
		else
			m_pModelMngr->AddWireCubeToRenderList(glm::translate(GetCenterGlobal()) * glm::scale(m_v3ARBBSize), C_YELLOW);
	}
}