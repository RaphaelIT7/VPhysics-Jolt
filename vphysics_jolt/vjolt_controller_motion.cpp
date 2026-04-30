
#include "cbase.h"

#include "vjolt_controller_motion.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static ConVar vjolt_motion_debug( "vjolt_motion_debug", "0", FCVAR_NONE, "Log motion controller (physgun etc) values." );

//-------------------------------------------------------------------------------------------------

JoltPhysicsMotionController::JoltPhysicsMotionController( IMotionEvent *pHandler )
	: m_pMotionEvent( pHandler )
{
}

JoltPhysicsMotionController::~JoltPhysicsMotionController()
{
	for ( JoltPhysicsObject *pObject : m_pObjects )
		pObject->RemoveDestroyedListener( this );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsMotionController::SetEventHandler( IMotionEvent *pHandler )
{
	m_pMotionEvent = pHandler;
}

void JoltPhysicsMotionController::AttachObject( IPhysicsObject *pObject, bool bCheckIfAlreadyAttached )
{
	if ( !pObject || pObject->IsStatic() )
		return;

	JoltPhysicsObject *pPhysicsObject = static_cast< JoltPhysicsObject * >( pObject );
	if ( bCheckIfAlreadyAttached && VectorContains( m_pObjects, pPhysicsObject ) )
		return;

	pPhysicsObject->AddDestroyedListener( this );
	m_pObjects.push_back( pPhysicsObject );
}

void JoltPhysicsMotionController::DetachObject( IPhysicsObject *pObject )
{
	if ( !pObject )
		return;

	JoltPhysicsObject *pPhysicsObject = static_cast< JoltPhysicsObject * >( pObject );
	Erase( m_pObjects, pPhysicsObject );
	pPhysicsObject->RemoveDestroyedListener( this );
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsMotionController::CountObjects( void )
{
	return int( m_pObjects.size() );
}

void JoltPhysicsMotionController::GetObjects( IPhysicsObject **pObjectList )
{
	for ( size_t i = 0; i < m_pObjects.size(); i++ )
		pObjectList[ i ] = m_pObjects[ i ];
}

void JoltPhysicsMotionController::ClearObjects( void )
{
	m_pObjects.clear();
}

void JoltPhysicsMotionController::WakeObjects( void )
{
	for ( JoltPhysicsObject *pObject : m_pObjects )
		pObject->Wake();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsMotionController::SetPriority( priority_t priority )
{
	// Not relevant to us.
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsMotionController::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	JoltPhysicsObject *pPhysicsObject = static_cast< JoltPhysicsObject * >( pObject );
	Erase( m_pObjects, pPhysicsObject );
}

void JoltPhysicsMotionController::OnPreSimulate( float flDeltaTime )
{
	if ( !m_pMotionEvent )
		return;

	for ( JoltPhysicsObject *pObject : m_pObjects )
	{
		if ( !pObject->IsMoveable() )
			continue;

		Vector speed = vec3_origin;
		AngularImpulse rot = vec3_origin;
		IMotionEvent::simresult_e simResult = m_pMotionEvent->Simulate( this, pObject, flDeltaTime, speed, rot );

		if ( vjolt_motion_debug.GetBool() )
		{
			Log_Msg( LOG_VJolt, "Motion ctrl: result=%d lin=(%.1f,%.1f,%.1f) |lin|=%.1f ang=(%.1f,%.1f,%.1f) |ang|=%.1f dt=%.4f\n",
				int( simResult ),
				speed.x, speed.y, speed.z, speed.Length(),
				rot.x, rot.y, rot.z, rot.Length(),
				flDeltaTime );
		}

		// Per IVP contract, the angular component is always in object-local (core)
		// space regardless of the LOCAL/GLOBAL flag - the flag governs only the
		// linear component. Jolt body angular velocity is in world space, so we
		// always need to rotate the angular value into world space before applying.
		Vector worldLinear = speed;
		AngularImpulse worldAngular;
		pObject->LocalToWorldVector( &worldAngular, rot );

		if ( simResult == IMotionEvent::SIM_LOCAL_ACCELERATION || simResult == IMotionEvent::SIM_LOCAL_FORCE )
			pObject->LocalToWorldVector( &worldLinear, speed );

		switch ( simResult )
		{
			case IMotionEvent::SIM_NOTHING:
				break;

			case IMotionEvent::SIM_LOCAL_ACCELERATION:
			case IMotionEvent::SIM_GLOBAL_ACCELERATION:
			{
				// IVP: pCore->speed += accel * dt; pCore->rot_speed += rot_accel * dt.
				Vector deltaVel = worldLinear * flDeltaTime;
				AngularImpulse deltaAng = worldAngular * flDeltaTime;
				pObject->AddVelocity( &deltaVel, &deltaAng );
				break;
			}

			case IMotionEvent::SIM_LOCAL_FORCE:
			case IMotionEvent::SIM_GLOBAL_FORCE:
			{
				// IVP: center_push divides force*dt by mass; rot_push by inertia.
				// Jolt's AddImpulse / AddAngularImpulse do the same.
				Vector linearImpulse = worldLinear * flDeltaTime;
				AngularImpulse angularImpulse = worldAngular * flDeltaTime;
				pObject->ApplyForceCenter( linearImpulse );
				pObject->ApplyTorqueCenter( angularImpulse );
				break;
			}

			default:
				Log_Warning( LOG_VJolt, "Invalid motion event\n" );
				break;
		}
	}
}
