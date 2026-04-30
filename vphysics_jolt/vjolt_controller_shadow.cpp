
#include "cbase.h"

#include "vjolt_surfaceprops.h"
#include "vjolt_controller_shadow.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static ConVar vjolt_shadow_debug( "vjolt_shadow_debug", "0", FCVAR_NONE, "Log shadow controller parameters." );

//-------------------------------------------------------------------------------------------------

JoltPhysicsShadowController::JoltPhysicsShadowController( JoltPhysicsObject *pObject, bool allowTranslation, bool allowRotation )
	: m_pObject( pObject ), m_allowTranslation( allowTranslation ), m_allowRotation( allowRotation )
{
	JPH::Body *pBody = m_pObject->GetBody();

	m_savedMotionType = pBody->GetMotionType();
	pBody->SetMotionType( JPH::EMotionType::Kinematic );

	m_savedMaterialIndex = m_pObject->GetMaterialIndex();
	UseShadowMaterial( true );

	m_savedCallbackFlags = m_pObject->GetCallbackFlags();
	uint16 flags = m_savedCallbackFlags | CALLBACK_SHADOW_COLLISION;
	flags &= ~CALLBACK_GLOBAL_FRICTION;
	flags &= ~CALLBACK_GLOBAL_COLLIDE_STATIC;
	m_pObject->SetCallbackFlags( flags );
	m_pObject->EnableDrag( false );

	m_targetPosition = pBody->GetPosition();
	m_targetRotation = pBody->GetRotation();
}

JoltPhysicsShadowController::~JoltPhysicsShadowController()
{
	if ( !m_pObject )
		return;

	if ( !( m_pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
	{
		m_pObject->SetCallbackFlags( m_savedCallbackFlags );
		m_pObject->EnableDrag( true );
		UseShadowMaterial( false );

		JPH::Body *pBody = m_pObject->GetBody();
		pBody->SetMotionType( m_savedMotionType );
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsShadowController::Update( const Vector &position, const QAngle &angles, float timeOffset )
{
	JPH::Vec3 targetPosition = SourceToJolt::Distance( position );
	JPH::Quat targetRotation = SourceToJolt::Angle( angles );

	const bool bSamePos = targetPosition.IsClose( m_targetPosition, 1e-8f );
	const bool bSameRot = targetRotation.IsClose( m_targetRotation, 1e-8f );

	m_targetPosition = targetPosition;
	m_targetRotation = targetRotation;
	m_secondsToArrival = Max( timeOffset, 0.0f );
	m_enabled = true;

	if ( !bSamePos || !bSameRot )
		m_pObject->Wake();
}

void JoltPhysicsShadowController::MaxSpeed( float maxSpeed, float maxAngularSpeed )
{
	m_maxSpeed = maxSpeed;
	m_maxDampSpeed = maxSpeed;
	m_maxAngular = maxAngularSpeed;
	m_maxDampAngular = maxAngularSpeed;
}

void JoltPhysicsShadowController::StepUp( float height )
{
	if ( height == 0.0f )
		return;

	m_pObject->AddToPosition( JPH::Vec3( 0.0f, 0.0f, SourceToJolt::Distance( height ) ) );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsShadowController::SetTeleportDistance( float teleportDistance )
{
	m_teleportDistance = SourceToJolt::Distance( teleportDistance );
}

bool JoltPhysicsShadowController::AllowsTranslation()
{
	return m_allowTranslation;
}

bool JoltPhysicsShadowController::AllowsRotation()
{
	return m_allowRotation;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsShadowController::SetPhysicallyControlled( bool isPhysicallyControlled )
{
	m_isPhysicallyControlled = isPhysicallyControlled;
}

bool JoltPhysicsShadowController::IsPhysicallyControlled()
{
	return m_isPhysicallyControlled;
}

void JoltPhysicsShadowController::GetLastImpulse( Vector *pOut )
{
	*pOut = JoltToSource::Distance( m_lastImpulse );
}

void JoltPhysicsShadowController::UseShadowMaterial( bool bUseShadowMaterial )
{
	if ( !m_pObject )
		return;

	int current = m_pObject->GetMaterialIndex();
	int target = bUseShadowMaterial ? JoltPhysicsSurfaceProps::GetInstance().GetShadowMaterialIndex() : m_savedMaterialIndex;
	if ( target != current )
		m_pObject->SetMaterialIndex( target );
}

void JoltPhysicsShadowController::ObjectMaterialChanged( int materialIndex )
{
	if ( !m_pObject )
		return;

	m_savedMaterialIndex = materialIndex;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsShadowController::GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut )
{
	if ( pPositionOut )
		*pPositionOut = JoltToSource::Distance( m_targetPosition );
	if ( pAnglesOut )
		*pAnglesOut = JoltToSource::Angle( m_targetRotation );

	return m_secondsToArrival;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsShadowController::GetTeleportDistance()
{
	return JoltToSource::Distance( m_teleportDistance );
}

void JoltPhysicsShadowController::GetMaxSpeed( float *pMaxSpeedOut, float *pMaxAngularSpeedOut )
{
	if ( pMaxSpeedOut )
		*pMaxSpeedOut = m_maxSpeed;

	if ( pMaxAngularSpeedOut )
		*pMaxAngularSpeedOut = m_maxAngular;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsShadowController::OnPreSimulate( float flDeltaTime )
{
	if ( !m_enabled )
		return;

	JPH::BodyInterface &bodyInterface = m_pObject->GetJoltEnvironment()->GetPhysicsSystem()->GetBodyInterfaceNoLock();

	if ( m_secondsToArrival > 0.0f )
	{
		bodyInterface.MoveKinematic( m_pObject->GetBodyID(), m_targetPosition, m_targetRotation, m_secondsToArrival );
	}
	else
	{
		bodyInterface.SetPositionAndRotation( m_pObject->GetBodyID(), m_targetPosition, m_targetRotation, JPH::EActivation::Activate );
		bodyInterface.SetLinearAndAngularVelocity( m_pObject->GetBodyID(), JPH::Vec3::sZero(), JPH::Vec3::sZero() );
		m_enabled = false;
	}

	m_secondsToArrival = Max( m_secondsToArrival - flDeltaTime, 0.0f );
}
