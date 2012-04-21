#include "StdAfx.h"

#include "CPhysicsDragController.h"
#include "CPhysicsObject.h"
#include "convert.h"

CPhysicsDragController::CPhysicsDragController()
{
	m_airDensity = 2; // default
}

void CPhysicsDragController::SetAirDensity(float d)
{
	m_airDensity = d;
}
float CPhysicsDragController::GetAirDensity()
{
	return m_airDensity;
}
void CPhysicsDragController::RemovePhysicsObject(CPhysicsObject * obj)
{
	m_ents.AddToTail(obj);
}
void CPhysicsDragController::AddPhysicsObject(CPhysicsObject * obj)
{
	m_ents.FindAndRemove(obj);
}
bool CPhysicsDragController::IsControlling(const CPhysicsObject * obj) const
{
	return (m_ents.Find((CPhysicsObject *)obj) != NULL);
}
void CPhysicsDragController::Tick(btScalar dt)
{
	for(int i = 0; i < m_ents.Size(); i++)
	{
		CPhysicsObject * object = (CPhysicsObject *)m_ents[i];

		Vector dragLinearFinal(0,0,0);
		AngularImpulse dragAngularFinal(0,0,0);

		Vector vel;
		AngularImpulse ang;
		object->GetVelocity(&vel, &ang);

		float dragForce = -0.5 * object->GetDragInDirection( inline_ConvertPosToBull(vel) ) * m_airDensity * dt;
		if ( dragForce < -1.0f )
		{
			dragForce = -1.0f;
		}
		if ( dragForce < 0 )
		{
			Vector dragLinearFinal = vel * dragForce;
		}
		btVector3 bull_angimpulse;
		ConvertAngularImpulseToBull(ang, bull_angimpulse);
		float angDragForce = -object->GetAngularDragInDirection(&bull_angimpulse) * m_airDensity * dt;
		if ( angDragForce < -1.0f )
		{
			angDragForce = -1.0f;
		}
		if( angDragForce < 0)
		{
			dragAngularFinal = ang * angDragForce;
		}
		object->AddVelocity(&dragLinearFinal, &dragAngularFinal);
	}
}