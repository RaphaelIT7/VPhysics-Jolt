//=================================================================================================
//
// Air drag controller. Applies a velocity-scaling drag force to every registered
// awake body in OnStep (per Jolt collision substep), mirroring IVP's
// CDragController::do_simulation_controller, which runs per PSI.
//
//=================================================================================================

#include "cbase.h"

#include "vjolt_controller_drag.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

JoltPhysicsDragController::JoltPhysicsDragController()
{
}

JoltPhysicsDragController::~JoltPhysicsDragController()
{
	for ( JoltPhysicsObject *pObject : m_pObjects )
		pObject->RemoveDestroyedListener( this );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsDragController::RegisterObject( JoltPhysicsObject *pObject )
{
	if ( !pObject || pObject->IsStatic() )
		return;

	if ( VectorContains( m_pObjects, pObject ) )
		return;

	pObject->AddDestroyedListener( this );
	m_pObjects.push_back( pObject );
}

void JoltPhysicsDragController::UnregisterObject( JoltPhysicsObject *pObject )
{
	if ( !pObject || !VectorContains( m_pObjects, pObject ) )
		return;

	Erase( m_pObjects, pObject );
	pObject->RemoveDestroyedListener( this );
}

void JoltPhysicsDragController::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	Erase( m_pObjects, pObject );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsDragController::OnStep( const JPH::PhysicsStepListenerContext &inContext )
{
	if ( m_flAirDensity <= 0.0f )
		return;

	const float flDeltaTime = inContext.mDeltaTime;

	for ( JoltPhysicsObject *pObject : m_pObjects )
	{
		JPH::Body *pBody = pObject->GetBody();

		if ( !pBody->IsActive() )
			continue;

		const JPH::Vec3 vLinearVel = pBody->GetLinearVelocity();
		const JPH::Vec3 vAngularVelWorld = pBody->GetAngularVelocity();
		// GetAngularDragInDirection expects local-space; do the world->local transform here.
		const JPH::Vec3 vAngularVelLocal = pBody->GetRotation().Conjugated() * vAngularVelWorld;

		float flLinearDrag = -0.5f * pObject->GetDragInDirection( vLinearVel ) * m_flAirDensity * flDeltaTime;
		if ( flLinearDrag < -1.0f )
			flLinearDrag = -1.0f;
		if ( flLinearDrag < 0.0f )
			pBody->SetLinearVelocity( vLinearVel + vLinearVel * flLinearDrag );

		float flAngularDrag = -pObject->GetAngularDragInDirection( vAngularVelLocal ) * m_flAirDensity * flDeltaTime;
		if ( flAngularDrag < -1.0f )
			flAngularDrag = -1.0f;
		if ( flAngularDrag < 0.0f )
			pBody->SetAngularVelocity( vAngularVelWorld + vAngularVelWorld * flAngularDrag );
	}
}
