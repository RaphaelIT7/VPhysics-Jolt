//=================================================================================================
//
// Air drag controller. One per environment. Mirrors IVP's CDragController.
//
// Hooks JPH::PhysicsStepListener so OnStep fires once per Jolt collision step (the
// closest analogue to an IVP PSI), receiving that step's dt. This matches IVP's
// per-PSI controller invocation semantics rather than the once-per-Simulate pattern
// the other vphysics_jolt controllers use.
//
//=================================================================================================

#pragma once

#include "vjolt_object.h"
#include "vjolt_environment.h"
#include "vjolt_internal_listeners.h"

class JoltPhysicsDragController final : public JPH::PhysicsStepListener, public IJoltObjectDestroyedListener
{
public:
	JoltPhysicsDragController();
	~JoltPhysicsDragController() override;

	void RegisterObject( JoltPhysicsObject *pObject );
	void UnregisterObject( JoltPhysicsObject *pObject );

	void SetAirDensity( float flDensity )	{ m_flAirDensity = flDensity; }
	float GetAirDensity() const				{ return m_flAirDensity; }

	void OnStep( const JPH::PhysicsStepListenerContext &inContext ) override;
	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) override;

private:
	// Matches IVP's AIR_DENSITY default in physics_environment.cpp.
	static constexpr float kDefaultAirDensity = 2.0f;

	float m_flAirDensity = kDefaultAirDensity;
	std::vector< JoltPhysicsObject * > m_pObjects;
};
