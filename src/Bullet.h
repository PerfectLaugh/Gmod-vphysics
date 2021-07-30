#ifndef VPHYSICS_BULLET_H_
#define VPHYSICS_BULLET_H_

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletCollision/CollisionShapes/btMaterial.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

// Implement this class to handle callbacks in your code.
class btGhostObjectCallback {
public:
	virtual void	addedOverlappingObject(btCollisionObject* object) {};
	virtual void	removedOverlappingObject(btCollisionObject* object) {};
};

class btGhostObjectWithCallback : public btGhostObject
{
public:
	virtual ~btGhostObjectWithCallback();

	///this method is mainly for expert/internal use only.
	virtual void addOverlappingObjectInternal(btBroadphaseProxy* otherProxy, btBroadphaseProxy* thisProxy = 0);
	///this method is mainly for expert/internal use only.
	virtual void removeOverlappingObjectInternal(btBroadphaseProxy* otherProxy, btDispatcher* dispatcher, btBroadphaseProxy* thisProxy = 0);

	btGhostObjectCallback* getCallback() const
	{
		return m_callback;
	}

	void setCallback(btGhostObjectCallback* callback)
	{
		m_callback = callback;
	}

protected:
	btGhostObjectCallback* m_callback;
};

class btConvexTriangleMeshShapeWithData : public btConvexTriangleMeshShape
{
public:
	btConvexTriangleMeshShapeWithData(btStridingMeshInterface * meshInterface, bool calcAabb = true) : btConvexTriangleMeshShape(meshInterface, calcAabb) {}
	virtual ~btConvexTriangleMeshShapeWithData() {}

	int getUserData()
	{
		return m_userData;
	}

	void setUserData(int data)
	{
		m_userData = data;
	}

protected:
	int m_userData;
};

#endif // VPHYSICS_BULLET_H_