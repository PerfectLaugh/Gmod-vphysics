#include "StdAfx.h"

#include "Physics_DragController.h"
#include "Physics_Object.h"
#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/******************************
* CLASS CPhysicsDragController
******************************/

CPhysicsDragController::CPhysicsDragController() {
	m_airDensity = 2.f; // default
}

void CPhysicsDragController::SetAirDensity(float d) {
	// Density is kg/in^3 from HL
	m_airDensity = d / CUBIC_METERS_PER_CUBIC_INCH;
}

float CPhysicsDragController::GetAirDensity() {
	return m_airDensity * CUBIC_METERS_PER_CUBIC_INCH;
}

void CPhysicsDragController::RemovePhysicsObject(CPhysicsObject *obj) {
	auto it = std::find(m_ents.begin(), m_ents.end(), obj);
	if (it == m_ents.end()) {
		return;
	}

	m_ents.erase(it);
}

void CPhysicsDragController::AddPhysicsObject(CPhysicsObject *obj) {
	if (!IsControlling(obj)) {
		m_ents.push_back(obj);
	}
}

bool CPhysicsDragController::IsControlling(const CPhysicsObject *obj) const {
	auto it = std::find(m_ents.begin(), m_ents.end(), obj);
	return it != m_ents.end();
}

void CPhysicsDragController::Tick(btScalar dt) {
	for (size_t i = 0; i < m_ents.size(); i++) {
		CPhysicsObject *pObject = (CPhysicsObject *)m_ents[i];
		btRigidBody *body = pObject->GetObject();
		if (body->getActivationState() == ISLAND_SLEEPING || body->getActivationState() == DISABLE_SIMULATION)
			continue;

		//------------------
		// LINEAR DRAG
		//------------------
		if (!btFuzzyZero(body->getLinearVelocity().length2())) {
			btVector3 vel(0, 0, 0);

			auto dragForce = -1.0 * pObject->GetDragInDirection(body->getLinearVelocity().normalized()) * (btScalar)m_airDensity * dt;
			if (dragForce < -1.0)
				dragForce = -1.0;

			// If the drag force actually drags
			if (dragForce < 0)
				vel = body->getLinearVelocity() * dragForce;

			body->setLinearVelocity(body->getLinearVelocity() + vel);
		}

		//------------------
		// ANGULAR DRAG
		//------------------
		if (!btFuzzyZero(body->getAngularVelocity().length2())) {
			btVector3 ang(0, 0, 0);

			auto angDragForce = -1.0 * pObject->GetAngularDragInDirection(body->getAngularVelocity().normalized()) * (btScalar)m_airDensity * dt;
			if (angDragForce < -1.0)
				angDragForce = -1.0;

			// If the drag force actually drags
			if (angDragForce < 0)
				ang = body->getAngularVelocity() * angDragForce;

			body->setAngularVelocity(body->getAngularVelocity() + ang);
		}
	}
}