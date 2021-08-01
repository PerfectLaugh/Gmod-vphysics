#include "StdAfx.h"

#include "Physics_MotionController.h"
#include "Physics_Object.h"
#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

IPhysicsMotionController *CreateMotionController(CPhysicsEnvironment *pEnv, IMotionEvent *pHandler) {
	if (!pEnv) return NULL;
	return new CPhysicsMotionController(pHandler, pEnv);
}

/***********************************
* CLASS CPhysicsMotionController
***********************************/

// This class can totally be moved back to game code.

CPhysicsMotionController::CPhysicsMotionController(IMotionEvent *pHandler, CPhysicsEnvironment *pEnv) {
	m_handler = pHandler;
	m_pEnv = pEnv;
}

CPhysicsMotionController::~CPhysicsMotionController() {
	for (int i = m_objectList.size()-1; i >= 0; i--) {
		DetachObject(m_objectList[i]);
	}
}

void CPhysicsMotionController::Tick(float deltaTime) {
	if (!m_handler) return;

	for (int i = 0; i < m_objectList.size(); i++) {
		Vector speed;
		AngularImpulse rot;
		
		CPhysicsObject *pObject = m_objectList[i];
		IMotionEvent::simresult_e ret = m_handler->Simulate(this, pObject, deltaTime, speed, rot);

		speed *= deltaTime;
		rot *= deltaTime;

		Vector curVel, curAngVel;
		pObject->GetVelocity(&curVel, &curAngVel);

		switch (ret) {
			case IMotionEvent::SIM_NOTHING: {
				break;
			}
			case IMotionEvent::SIM_LOCAL_ACCELERATION: {
				// Convert velocity to world space
				Vector newVel;
				pObject->LocalToWorldVector(&newVel, speed);

				pObject->AddVelocity(&newVel, &rot); // Rotation already in local space.
				break;
			}
			case IMotionEvent::SIM_LOCAL_FORCE: {
				Vector newVel;
				pObject->LocalToWorldVector(&newVel, speed);

				pObject->ApplyForceCenter(newVel);
				pObject->ApplyTorqueCenter(rot);
				break;
			}
			case IMotionEvent::SIM_GLOBAL_ACCELERATION: {
				pObject->AddVelocity(&speed, &rot);
				break;
			}
			case IMotionEvent::SIM_GLOBAL_FORCE: {
				pObject->ApplyForceCenter(speed);
				pObject->ApplyTorqueCenter(rot);
				break;
			}
			default: {
				DevWarning("VPhysics: Invalid motion controller event type returned (%d)\n", ret);
			}
		}
	}
}

void CPhysicsMotionController::ObjectDestroyed(CPhysicsObject *pObject) {
	auto it = std::find(m_objectList.begin(), m_objectList.end(), pObject);
	if (it != m_objectList.end()) {
		m_objectList.erase(it);
	}
}

void CPhysicsMotionController::SetEventHandler(IMotionEvent *handler) {
	m_handler = handler;
}

void CPhysicsMotionController::AttachObject(IPhysicsObject *pObject, bool checkIfAlreadyAttached) {
	Assert(pObject);
	if (!pObject || pObject->IsStatic()) return;

	CPhysicsObject *pPhys = (CPhysicsObject *)pObject;

	auto it = std::find(m_objectList.begin(), m_objectList.end(), pPhys);
	if (it != m_objectList.end() && checkIfAlreadyAttached)
		return;

	pPhys->AttachedToController(this);
	pPhys->AttachEventListener(this);
	m_objectList.push_back(pPhys);
}

void CPhysicsMotionController::DetachObject(IPhysicsObject *pObject) {
	CPhysicsObject *pPhys = (CPhysicsObject *)pObject;

	auto it = std::find(m_objectList.begin(), m_objectList.end(), pPhys);
	if (it == m_objectList.end()) return;

	pPhys->DetachedFromController(this);
	pPhys->DetachEventListener(this);
	m_objectList.erase(it);
}

int CPhysicsMotionController::CountObjects() {
	return m_objectList.size();
}

void CPhysicsMotionController::GetObjects(IPhysicsObject **pObjectList) {
	if (!pObjectList) return;

	for (size_t i = 0; i < m_objectList.size(); i++) {
		pObjectList[i] = (IPhysicsObject *)m_objectList[i];
	}
}

void CPhysicsMotionController::ClearObjects() {
	m_objectList.clear();
}

void CPhysicsMotionController::WakeObjects() {
	for (size_t i = 0; i < m_objectList.size(); i++) {
		m_objectList[i]->GetObject()->setActivationState(ACTIVE_TAG);
	}
}

void CPhysicsMotionController::SetPriority(priority_t priority) {
	// IVP Controllers had a priority. Since bullet doesn't have controllers, this function is useless.
}
