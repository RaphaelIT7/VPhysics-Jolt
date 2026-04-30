
#pragma once

#include "vjolt_object.h"
#include "vjolt_environment.h"

class JoltPhysicsShadowController final : public IPhysicsShadowController, public IJoltPhysicsController
{
public:
	JoltPhysicsShadowController( JoltPhysicsObject *pObject, bool allowTranslation, bool allowRotation );
	~JoltPhysicsShadowController() override;

	void Update( const Vector &position, const QAngle &angles, float timeOffset ) override;
	void MaxSpeed( float maxSpeed, float maxAngularSpeed ) override;
	void StepUp( float height ) override;

	void SetTeleportDistance( float teleportDistance ) override;
	bool AllowsTranslation() override;
	bool AllowsRotation() override;

	void SetPhysicallyControlled( bool isPhysicallyControlled ) override;
	bool IsPhysicallyControlled() override;
	void GetLastImpulse( Vector *pOut ) override;
	void UseShadowMaterial( bool bUseShadowMaterial ) override;
	void ObjectMaterialChanged( int materialIndex ) override;

	float GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut ) override;

	float GetTeleportDistance() override;
	void GetMaxSpeed( float *pMaxSpeedOut, float *pMaxAngularSpeedOut ) override;

	// IJoltPhysicsController
	void OnPreSimulate( float flDeltaTime ) override;

private:
	JoltPhysicsObject *m_pObject = nullptr;

	JPH::Vec3 m_targetPosition = JPH::Vec3::sZero();
	JPH::Quat m_targetRotation = JPH::Quat::sIdentity();
	JPH::Vec3 m_lastImpulse = JPH::Vec3::sZero();
	float m_secondsToArrival = 0;

	float m_maxSpeed = 0.0f;
	float m_maxDampSpeed = 0.0f;
	float m_maxAngular = 0.0f;
	float m_maxDampAngular = 0.0f;
	float m_teleportDistance = 0.0f;
	bool m_isPhysicallyControlled = false;
	bool m_allowTranslation = false;
	bool m_allowRotation = false;

	bool m_enabled = false;

	uint16 m_savedMaterialIndex = 0;
	uint16 m_savedCallbackFlags = 0;

	float m_savedInvMass = 0.0f;
	JPH::Vec3 m_savedInvInertiaDiagonal = JPH::Vec3::sZero();
	JPH::Quat m_savedInertiaRotation = JPH::Quat::sIdentity();
	float m_savedGravityFactor = 1.0f;
	float m_savedLinearDamping = 0.0f;
	float m_savedAngularDamping = 0.0f;
	bool m_motionPropertiesSaved = false;
};
