#include "StdAfx.h"

btGhostObjectWithCallback::~btGhostObjectWithCallback() {}

void btGhostObjectWithCallback::addOverlappingObjectInternal(btBroadphaseProxy* otherProxy, btBroadphaseProxy* thisProxy)
{
	btCollisionObject* otherObject = (btCollisionObject*)otherProxy->m_clientObject;
	btAssert(otherObject);
	///if this linearSearch becomes too slow (too many overlapping objects) we should add a more appropriate data structure
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index == m_overlappingObjects.size())
	{
		//not found
		m_overlappingObjects.push_back(otherObject);

		if (m_callback)
			m_callback->addedOverlappingObject(otherObject);
	}
}

void btGhostObjectWithCallback::removeOverlappingObjectInternal(btBroadphaseProxy* otherProxy, btDispatcher* dispatcher, btBroadphaseProxy* thisProxy)
{
	btCollisionObject* otherObject = (btCollisionObject*)otherProxy->m_clientObject;
	btAssert(otherObject);
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index < m_overlappingObjects.size())
	{
		m_overlappingObjects[index] = m_overlappingObjects[m_overlappingObjects.size() - 1];
		m_overlappingObjects.pop_back();

		if (m_callback)
			m_callback->removedOverlappingObject(otherObject);
	}
}
