
#include "cbase.h"

#include "vjolt_surfaceprops.h"
#include "vjolt_controller_shadow.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static ConVar vjolt_shadow_debug( "vjolt_shadow_debug", "0", FCVAR_NONE, "Log shadow controller parameters." );

//-------------------------------------------------------------------------------------------------

static void ComputeController( JPH::Vec3 &ioCurrentSpeed, JPH::Vec3Arg inDelta, float inMaxSpeed, float inMaxDampSpeed, float inScaleDelta, float inDamping, JPH::Vec3 *pOutImpulse )
{
	if ( ioCurrentSpeed.LengthSq() < 1e-6f )
		ioCurrentSpeed = JPH::Vec3::sZero();

	JPH::Vec3 acceleration = JPH::Vec3::sZero();
	if ( inMaxSpeed > 0.0f )
	{
		acceleration = inDelta * inScaleDelta;
		const float speed = acceleration.Length();
		if ( speed > inMaxSpeed )
			acceleration *= inMaxSpeed / speed;
	}

	JPH::Vec3 dampAccel = JPH::Vec3::sZero();
	if ( inMaxDampSpeed > 0.0f )
	{
		dampAccel = ioCurrentSpeed * -inDamping;
		const float speed = dampAccel.Length();
		if ( speed > inMaxDampSpeed )
			dampAccel *= inMaxDampSpeed / speed;
	}

	ioCurrentSpeed += dampAccel;
	ioCurrentSpeed += acceleration;

	if ( pOutImpulse )
		*pOutImpulse = acceleration;
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsShadowController::JoltPhysicsShadowController( JoltPhysicsObject *pObject, bool allowTranslation, bool allowRotation )
	: m_pObject( pObject ), m_allowTranslation( allowTranslation ), m_allowRotation( allowRotation )
{
	if ( vjolt_shadow_debug.GetBool() )
		Log_Msg( LOG_VJolt, "Shadow ctor: obj=%p allowT=%d allowR=%d\n", pObject, allowTranslation, allowRotation );

	JPH::Body *pBody = m_pObject->GetBody();
	JPH::MotionProperties *pMP = pBody->GetMotionProperties();

	if ( pMP )
	{
		m_savedInvMass = pMP->GetInverseMassUnchecked();
		m_savedInvInertiaDiagonal = pMP->GetInverseInertiaDiagonal();
		m_savedInertiaRotation = pMP->GetInertiaRotation();
		m_savedGravityFactor = pMP->GetGravityFactor();
		m_savedLinearDamping = pMP->GetLinearDamping();
		m_savedAngularDamping = pMP->GetAngularDamping();
		m_motionPropertiesSaved = true;

		if ( !allowTranslation )
		{
			pMP->SetInverseMass( 0.0f );
			pMP->SetGravityFactor( 0.0f );
		}
		if ( !allowRotation )
		{
			pMP->SetInverseInertia( JPH::Vec3::sZero(), JPH::Quat::sIdentity() );
		}

		pMP->SetLinearDamping( 0.0f );
		pMP->SetAngularDamping( 100.0f );
	}

	m_savedCallbackFlags = m_pObject->GetCallbackFlags();
	m_pObject->SetCallbackFlags( m_savedCallbackFlags | CALLBACK_SHADOW_COLLISION );

	m_targetPosition = SourceToJolt::Distance( JoltToSource::Distance( pBody->GetPosition() ) );
	m_targetRotation = pBody->GetRotation();
}

JoltPhysicsShadowController::~JoltPhysicsShadowController()
{
	if ( !m_pObject )
		return;

	if ( !( m_pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
	{
		m_pObject->SetCallbackFlags( m_savedCallbackFlags );

		if ( m_motionPropertiesSaved )
		{
			JPH::Body *pBody = m_pObject->GetBody();
			JPH::MotionProperties *pMP = pBody->GetMotionProperties();
			if ( pMP )
			{
				pMP->SetInverseMass( m_savedInvMass );
				pMP->SetInverseInertia( m_savedInvInertiaDiagonal, m_savedInertiaRotation );
				pMP->SetGravityFactor( m_savedGravityFactor );
				pMP->SetLinearDamping( m_savedLinearDamping );
				pMP->SetAngularDamping( m_savedAngularDamping );
			}
		}
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

	if ( vjolt_shadow_debug.GetBool() )
		Log_Msg( LOG_VJolt, "Shadow Update: pos=(%.1f,%.1f,%.1f) timeOffset=%.4f\n", position.x, position.y, position.z, timeOffset );

	if ( !bSamePos || !bSameRot )
		m_pObject->Wake();
}

void JoltPhysicsShadowController::MaxSpeed( float maxSpeed, float maxAngularSpeed )
{
	m_maxSpeed = maxSpeed;
	m_maxDampSpeed = maxSpeed;
	m_maxAngular = maxAngularSpeed;
	m_maxDampAngular = maxAngularSpeed;

	if ( vjolt_shadow_debug.GetBool() )
	{
		Log_Msg( LOG_VJolt, "Shadow MaxSpeed: lin=%.1f in/s ang=%.1f deg/s\n", maxSpeed, maxAngularSpeed );
	}
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
	if ( !m_enabled || flDeltaTime <= 0.0f )
		return;

	JPH::Body *pBody = m_pObject->GetBody();
	JPH::MotionProperties *pMP = pBody->GetMotionProperties();
	if ( !pMP )
		return;

	float fraction = 1.0f;
	if ( m_secondsToArrival > 0.0f )
	{
		fraction = flDeltaTime / m_secondsToArrival;
		if ( fraction > 1.0f )
			fraction = 1.0f;
	}

	m_secondsToArrival = Max( m_secondsToArrival - flDeltaTime, 0.0f );

	if ( fraction <= 0.0f )
		return;

	const float scaleDelta = fraction / flDeltaTime;
	const float dampFactor = 1.0f;

	if ( m_allowTranslation )
	{
		const JPH::Vec3 currentPos = pBody->GetPosition();
		JPH::Vec3 delta = m_targetPosition - currentPos;

		if ( m_teleportDistance > 0.0f && delta.LengthSq() > m_teleportDistance * m_teleportDistance )
		{
			JPH::BodyInterface &bodyInterface = m_pObject->GetJoltEnvironment()->GetPhysicsSystem()->GetBodyInterfaceNoLock();
			bodyInterface.SetPositionAndRotation( pBody->GetID(), m_targetPosition, m_targetRotation, JPH::EActivation::Activate );
			pMP->SetLinearVelocity( JPH::Vec3::sZero() );
			pMP->SetAngularVelocity( JPH::Vec3::sZero() );
			m_lastImpulse = JPH::Vec3::sZero();
			return;
		}

		const float maxSpeed = SourceToJolt::Distance( m_maxSpeed );
		const float maxDampSpeed = SourceToJolt::Distance( m_maxDampSpeed );

		JPH::Vec3 newVel = pMP->GetLinearVelocity();
		ComputeController( newVel, delta, maxSpeed, maxDampSpeed, scaleDelta, dampFactor, &m_lastImpulse );
		pMP->SetLinearVelocityClamped( newVel );

		if ( vjolt_shadow_debug.GetBool() )
		{
			Log_Msg( LOG_VJolt, "Shadow tick: delta=%.3f m newVel=%.2f m/s seek=%.1f m/s scaleDelta=%.1f sec=%.4f\n",
				delta.Length(), newVel.Length(), maxSpeed, scaleDelta, m_secondsToArrival );
		}
	}

	if ( m_allowRotation )
	{
		JPH::Quat deltaQuat = m_targetRotation * pBody->GetRotation().Conjugated();
		JPH::Vec3 axis;
		float angle;
		deltaQuat.GetAxisAngle( axis, angle );
		if ( angle > M_PI_F )
			angle -= 2.0f * M_PI_F;
		const JPH::Vec3 deltaAngles = axis * angle;

		const float maxAngular = DEG2RAD( m_maxAngular );
		const float maxDampAngular = DEG2RAD( m_maxDampAngular );

		JPH::Vec3 newAngVel = pMP->GetAngularVelocity();
		ComputeController( newAngVel, deltaAngles, maxAngular, maxDampAngular, scaleDelta, dampFactor, nullptr );
		pMP->SetAngularVelocityClamped( newAngVel );
	}

	if ( !pBody->IsActive() )
	{
		JPH::BodyInterface &bodyInterface = m_pObject->GetJoltEnvironment()->GetPhysicsSystem()->GetBodyInterfaceNoLock();
		const JPH::BodyID bodyId = pBody->GetID();
		bodyInterface.ActivateBodies( &bodyId, 1 );
	}
}
