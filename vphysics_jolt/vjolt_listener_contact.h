
#pragma once

#include "vjolt_controller_fluid.h"
#include "vjolt_surfaceprops.h"

#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>

struct JoltPhysicsContactPair
{
	JoltPhysicsContactPair( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
		: pObject1(pObject1), pObject2(pObject2)
	{
	}

	JoltPhysicsObject *pObject1 = nullptr;
	JoltPhysicsObject *pObject2 = nullptr;
};

enum VPhysicsGameFlags : uint32
{
	FVPHYSICS_DMG_SLICE				= 0x0001,
	FVPHYSICS_CONSTRAINT_STATIC		= 0x0002,
	FVPHYSICS_PLAYER_HELD			= 0x0004,
	FVPHYSICS_PART_OF_RAGDOLL		= 0x0008,
	FVPHYSICS_MULTIOBJECT_ENTITY	= 0x0010,
	FVPHYSICS_HEAVY_OBJECT			= 0x0020,
	FVPHYSICS_PENETRATING			= 0x0040,
	FVPHYSICS_NO_PLAYER_PICKUP		= 0x0080,
	FVPHYSICS_WAS_THROWN			= 0x0100,
	FVPHYSICS_DMG_DISSOLVE			= 0x0200,
	FVPHYSICS_NO_IMPACT_DMG			= 0x0400,
	FVPHYSICS_NO_NPC_IMPACT_DMG		= 0x0800,
	FVPHYSICS_PUSH_PLAYER			= 0x1000,
	FVPHYSICS_NO_SELF_COLLISIONS	= 0x8000,
};

class JoltPhysicsContactListener final : public JPH::ContactListener
{
public:
	JoltPhysicsContactListener( JPH::PhysicsSystem &physicsSystem )
		: m_PhysicsSystem( physicsSystem )
	{
	}

	JPH::ValidateResult OnContactValidate( const JPH::Body &inBody1, const JPH::Body &inBody2, JPH::Vec3Arg inBaseOffset, const JPH::CollideShapeResult &inCollisionResult ) override
	{
		return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
	}

	void OnContactAdded( const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings ) override
	{	
		JoltPhysicsObject* pObject1 = reinterpret_cast<JoltPhysicsObject*>( inBody1.GetUserData() );
		JoltPhysicsObject* pObject2 = reinterpret_cast<JoltPhysicsObject*>( inBody2.GetUserData() );

		bool bShouldCollide = ShouldCollide( pObject1, pObject2 );
		// If the game says we shouldn't collide, we will treat this as a sensor
		// to satisfy the StartTouch/EndTouch events.
		ioSettings.mIsSensor = !bShouldCollide || ioSettings.mIsSensor;

		if ( !m_pGameListener )
			return;

		if ( pObject1->IsFluid() || pObject2->IsFluid() )
		{
			const uint32 uThreadId = GetThreadId();

			if ( pObject1->IsFluid() && ( pObject2->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_FluidStartTouchEvents.EmplaceBack( uThreadId, pObject1, pObject2 );

			if ( pObject2->IsFluid() && ( pObject1->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_FluidStartTouchEvents.EmplaceBack( uThreadId, pObject2, pObject1 );

			return;
		}

		if ( pObject1->IsTrigger() || pObject2->IsTrigger() )
		{
			const uint32 uThreadId = GetThreadId();

			if ( pObject1->IsTrigger() )
				m_EnterTriggerEvents.EmplaceBack( uThreadId, pObject1, pObject2 );
		
			if ( pObject2->IsTrigger() )
				m_EnterTriggerEvents.EmplaceBack( uThreadId, pObject2, pObject1 );
	
			return;
		}

		const bool bIsCollision			= bShouldCollide && JoltPhysicsCollisionEvent::IsCollision		( pObject1, pObject2 );
		const bool bIsShadowCollision	= bShouldCollide && JoltPhysicsCollisionEvent::IsShadowCollision( pObject1, pObject2 );

		if ( bIsCollision || bIsShadowCollision )
		{
			// Josh:
			// We know ahead of time what this is used for (playing sounds and such)
			// and it is not easily threadable
			// (unlike the StartTouch objects which we can get away as long as the objects themselves aren't concurrent it seems)
			// To avoid this causing locks and therefore lagging with many objects,
			// we can just know ahead of time what is going to cause a sound to play, which is
			// hardcoded at speed > 70.0f and deltaTime < 0.05 (the latter of which we don't track)
			// So we can just avoid sending these PreCollision in this case.

			const Vector vecCollideNormal = Vector( inManifold.mWorldSpaceNormal.GetX(), inManifold.mWorldSpaceNormal.GetY(), inManifold.mWorldSpaceNormal.GetZ() );
			const float flCollisionSpeed = JoltPhysicsCollisionEvent::GetCollisionSpeed( pObject1, pObject2, vecCollideNormal );

			// Skip impact reporting when either body is currently held by the player.
			// The physgun's shadow controller drives the held body into the contact
			// every tick; under IVP this would resolve to zero relative velocity at
			// the contact (run-to-convergence), but Jolt's fixed-iteration solver
			// leaves residual velocity that the engine reads as a "real" impact and
			// triggers camera shake / impact sounds for what is really player input.
			const bool bHeldByPlayer =
				( pObject1->GetGameFlags() & FVPHYSICS_PLAYER_HELD ) ||
				( pObject2->GetGameFlags() & FVPHYSICS_PLAYER_HELD );

			const bool bHasSound =
				flCollisionSpeed >= 70.0f &&
				!bHeldByPlayer &&
				pObject1->GetGameMaterialAllowsSounds() &&
				pObject2->GetGameMaterialAllowsSounds();

			const bool bSane = m_GlobalCollisionEventCount < MaxCollisionEvents;

			const bool bSendCollisionCallback = ( bHasSound && bSane ) || bIsShadowCollision;

			if ( bSendCollisionCallback )
			{
				m_CollisionEvents.EmplaceBack( GetThreadId(), JoltPhysicsCollisionInfo( pObject1, pObject2, inManifold ) );
				m_GlobalCollisionEventCount++;
			}
		}

		if ( ShouldTouchCallback( pObject1, pObject2 ) )
			m_StartTouchEvents.EmplaceBack( GetThreadId(), JoltPhysicsCollisionInfo( pObject1, pObject2, inManifold ) );
	}

	void OnContactPersisted( const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings ) override
	{
		JoltPhysicsObject* pObject1 = reinterpret_cast<JoltPhysicsObject*>( inBody1.GetUserData() );
		JoltPhysicsObject* pObject2 = reinterpret_cast<JoltPhysicsObject*>( inBody2.GetUserData() );

		bool bShouldCollide = ShouldCollide( pObject1, pObject2 );
		// If the game says we shouldn't collide, we will treat this as a sensor
		// to satisfy the StartTouch/EndTouch events.
		ioSettings.mIsSensor = !bShouldCollide || ioSettings.mIsSensor;

		// Sensors don't generate impulses or friction events.
		if ( ioSettings.mIsSensor )
			return;

		// Estimate the resolved contact and friction impulses for this manifold.
		// Jolt doesn't expose post-solve constraint impulses cheaply, so we run a few
		// iterations of the same impulse solver via EstimateCollisionResponse to get
		// per-tick estimates - used both for friction sound events and for
		// IPhysicsFrictionSnapshot::GetNormalForce / GetEnergyAbsorbed feedback.
		JPH::CollisionEstimationResult result;
		JPH::EstimateCollisionResponse( inBody1, inBody2, inManifold, result,
			ioSettings.mCombinedFriction, ioSettings.mCombinedRestitution,
			/*minVelocityForRestitution*/ 1.0f, /*numIterations*/ 4 );

		// Sum normal impulse across contact points and stash on each body for the
		// friction snapshot to read. IVP exposes get_vert_force per contact; this is
		// the closest equivalent we can produce in Jolt.
		float flNormalImpulseSum = 0.0f;
		for ( const JPH::CollisionEstimationResult::Impulse &imp : result.mImpulses )
			flNormalImpulseSum += imp.mContactImpulse;
		pObject1->AccumulateContactNormalImpulse( pObject2, flNormalImpulseSum );
		pObject2->AccumulateContactNormalImpulse( pObject1, flNormalImpulseSum );

		if ( !m_pGameListener )
			return;

		if ( !ShouldFrictionCallback( pObject1, pObject2 ) )
			return;

		// Sum friction impulse vectors across the manifold's contact points.
		JPH::Vec3 vFrictionImpulse = JPH::Vec3::sZero();
		for ( const JPH::CollisionEstimationResult::Impulse &imp : result.mImpulses )
			vFrictionImpulse += imp.mFrictionImpulse1 * result.mTangent1
			                  + imp.mFrictionImpulse2 * result.mTangent2;

		const float flFrictionImpulseMag = vFrictionImpulse.Length();
		if ( flFrictionImpulseMag < 1e-4f )
			return;

		// Relative tangent velocity at the contact (first point — sliding contacts
		// typically have one dominant manifold point).
		const JPH::Vec3 vWorldContact = inManifold.GetWorldSpaceContactPointOn1( 0 );
		const JPH::Vec3 vRel = inBody1.GetPointVelocity( vWorldContact ) - inBody2.GetPointVelocity( vWorldContact );
		const JPH::Vec3 vTangent = vRel - vRel.Dot( inManifold.mWorldSpaceNormal ) * inManifold.mWorldSpaceNormal;
		const float flTangentSpeed = vTangent.Length();

		// Skip near-stationary contacts to avoid event spam every tick.
		constexpr float kMinTangentSpeed = 0.5f; // ~50 cm/s
		if ( flTangentSpeed < kMinTangentSpeed )
			return;

		// IVP's friction event reports `eliminated_energy / dt / mass` (specific power, W/kg)
		// then converts to HL units with the same in²/m² factor we use here. We match that
		// so the engine's energy thresholds behave as they did under IVP. The friction
		// impulse from EstimateCollisionResponse is per-step; assume Source's typical
		// 60Hz physics step for the dt scaling.
		const float flMass1 = pObject1->IsStatic() ? 0.0f : pObject1->GetMass();
		const float flMass2 = pObject2->IsStatic() ? 0.0f : pObject2->GetMass();
		const float flMass = flMass1 > 0.0f ? flMass1 : flMass2;
		if ( flMass <= 0.0f )
			return;

		constexpr float kAssumedStepDt = 1.0f / 60.0f;
		const float flPowerPerMass = ( flFrictionImpulseMag * flTangentSpeed ) / kAssumedStepDt / flMass;
		const float flEnergy = JoltToSource::Energy( flPowerPerMass );

		// Snapshot also queries energy absorbed; track on both bodies.
		pObject1->AccumulateContactFrictionEnergy( pObject2, flEnergy );
		pObject2->AccumulateContactFrictionEnergy( pObject1, flEnergy );

		m_FrictionEvents.EmplaceBack( GetThreadId(),
			JoltPhysicsCollisionInfo( pObject1, pObject2, inManifold ),
			flEnergy );
	}

	void OnContactRemoved( const JPH::SubShapeIDPair &inSubShapePair )
	{
		if ( !m_pGameListener )
			return;

		// This is always called with all bodies locked.
		const JPH::BodyLockInterfaceNoLock &bodyInterface = m_PhysicsSystem.GetBodyLockInterfaceNoLock();

		JPH::Body *pBody1 = bodyInterface.TryGetBody( inSubShapePair.GetBody1ID() );
		JPH::Body *pBody2 = bodyInterface.TryGetBody( inSubShapePair.GetBody2ID() );

		// One of the bodies may have been deleted.
		// TODO(Josh): Handle calling end touch when we delete a body.
		if ( !pBody1 || !pBody2 )
			return;

		JoltPhysicsObject *pObject1 = reinterpret_cast< JoltPhysicsObject * >( pBody1->GetUserData() );
		JoltPhysicsObject *pObject2 = reinterpret_cast< JoltPhysicsObject * >( pBody2->GetUserData() );

		if ( pObject1->IsFluid() || pObject2->IsFluid() )
		{
			const uint32 uThreadId = GetThreadId();

			if ( pObject1->IsFluid() && ( pObject2->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_FluidEndTouchEvents.EmplaceBack( uThreadId, pObject1, pObject2 );

			if ( pObject2->IsFluid() && ( pObject1->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_FluidEndTouchEvents.EmplaceBack( uThreadId, pObject2, pObject1 );

			return;
		}

		if ( pObject1->IsTrigger() || pObject2->IsTrigger() )
		{
			const uint32 uThreadId = GetThreadId();

			if ( pObject1->IsTrigger() )
				m_LeaveTriggerEvents.EmplaceBack( uThreadId, pObject1, pObject2 );
		
			if ( pObject2->IsTrigger() )
				m_LeaveTriggerEvents.EmplaceBack( uThreadId, pObject2, pObject1 );
	
			return;
		}

		if ( !ShouldTouchCallback( pObject1, pObject2 ) )
			return;

		const uint32 uThreadId = GetThreadId();

		// Josh:
		// We don't have any collision data here
		// and caching it would be annoying and expensive.
		// 
		// Lucky for us though, the game simply just calls the stuff
		// to retrieve the contact point and normal, then just never uses it
		// so we can return anything we want and it will change *nothing*!
		m_EndTouchEvents.EmplaceBack( uThreadId, JoltPhysicsCollisionInfo( pObject1, pObject2 ) );
	}

	bool ShouldCollide( JoltPhysicsObject *pObject0, JoltPhysicsObject *pObject1 )
	{
		VJoltAssert( pObject0 != pObject1 );

		if ( !pObject0 || !pObject1 )
			return false;

		if ( ( pObject0->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION ) && ( pObject1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
			return false;

		if ( ( pObject1->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION ) && ( pObject0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
			return false;

		if ( !m_pGameSolver )
			return true;

		// Josh:
		// Do some work the game does ahead of time to
		// avoid needless locking.
		if ( !PreEmptGameShouldCollide( pObject0, pObject1 ) )
			return false;

		// Actually ask the game now, locking both bodies so they cannot have
		// concurrent ShouldCollide calls.
		//JoltPhysicsObjectPairLock lock( pObject0->GetCollisionTestLock(), pObject1->GetCollisionTestLock() );
		std::unique_lock lock( m_ShouldCollideLock );
		return m_pGameSolver->ShouldCollide( pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData() );
	}

	bool PreEmptGameShouldCollide( JoltPhysicsObject *pObject0, JoltPhysicsObject *pObject1 )
	{
		// This function pre-empts the result of the
		// game's ShouldCollide implementation to avoid needless locking.

		// Check if the entities are the same and self-collisions are disabled.
		if ( pObject0->GetGameData() == pObject1->GetGameData() )
		{
			if ( ( pObject0->GetGameFlags() | pObject1->GetGameFlags() ) & FVPHYSICS_NO_SELF_COLLISIONS )
				return false;
		}

		// If both of these are constrained to the world, they shouldn't collide.
		if ( pObject0->GetGameFlags() & pObject1->GetGameFlags() & FVPHYSICS_CONSTRAINT_STATIC )
			return false;

		// We do wheels separately, the IS_VEHICLE_WHEEL is just to have some dummy object to return to the game.
		if ( ( pObject0->GetCallbackFlags() & CALLBACK_IS_VEHICLE_WHEEL ) || ( pObject1->GetCallbackFlags() & CALLBACK_IS_VEHICLE_WHEEL ) )
			return false;

		// Two shadow controlled objects should not collide.
		if ( pObject0->GetShadowController() && pObject1->GetShadowController() )
			return false;

		return true;
	}

	bool ShouldFrictionCallback( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
	{
		if ( !( pObject1->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION ) )
			return false;

		if ( !( pObject2->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION ) )
			return false;

		return true;
	}

	bool ShouldTouchCallback( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
	{
		uint32 uFlags = 0;
		uFlags |= pObject1->GetCallbackFlags();
		uFlags |= pObject2->GetCallbackFlags();

		if ( !( uFlags & CALLBACK_GLOBAL_TOUCH ) )
			return false;

		if ( !( uFlags & CALLBACK_GLOBAL_TOUCH_STATIC ) && ( pObject1->IsStatic() || pObject2->IsStatic() ) )
			return false;

		return true;
	}

	IPhysicsCollisionEvent *GetGameListener()
	{
		return m_pGameListener;
	}

	void SetGameListener( IPhysicsCollisionEvent *pListener )
	{
		m_pGameListener = pListener;
	}

	IPhysicsCollisionSolver *GetGameSolver()
	{
		return m_pGameSolver;
	}

	void SetGameSolver( IPhysicsCollisionSolver *pSolver )
	{
		m_pGameSolver = pSolver;
	}

	void FlushCallbacks()
	{
		if ( !m_pGameListener )
			return;

		// Send PreCollision events
		//
		// Don't clear the collision events the first time around as we have
		// the post-collisde event to send too!
		m_CollisionEvents.ForEach< false >( [ this ]( JoltPhysicsCollisionEvent& event )
		{
			JoltPhysicsObject *pObj1 = event.m_Data.GetPair().pObject1;
			JoltPhysicsObject *pObj2 = event.m_Data.GetPair().pObject2;

			// Compute collisionSpeed from the post-resolution body velocities (matches
			// IVP's contact->speed semantics: relative velocity at the contact surface
			// after impulses have been applied). Reading pre-solver velocity here would
			// over-report the impact for any externally-driven body (e.g. physgun-held
			// props), causing spurious camera shake on sustained contact.
			Vector vel1, vel2, normal;
			pObj1->GetVelocity( &vel1, nullptr );
			pObj2->GetVelocity( &vel2, nullptr );
			event.m_Data.GetSurfaceNormal( normal );
			event.m_Event.collisionSpeed = fabsf( ( vel1 - vel2 ).Dot( normal ) );

			// Fake the velocities for the objects during the PreCollision callback so
			// we get a proper delta velocity between Pre/Post for damage callbacks to work.
			JPH::Vec3 object1Vel = pObj1->FakeJoltLinearVelocity( event.m_Data.GetObject1PreCollisionVelocity() );
			JPH::Vec3 object2Vel = pObj2->FakeJoltLinearVelocity( event.m_Data.GetObject2PreCollisionVelocity() );
			m_pGameListener->PreCollision( &event.m_Event );
			pObj1->RestoreJoltLinearVelocity( object1Vel );
			pObj2->RestoreJoltLinearVelocity( object2Vel );
		});

		// Send StartTouch events
		m_StartTouchEvents.ForEach< true >( [ this ]( JoltPhysicsCollisionData& event )
		{
			m_pGameListener->StartTouch( event.GetPair().pObject1, event.GetPair().pObject2, &event );
		});

		// Send EnterTrigger events
		m_EnterTriggerEvents.ForEach< true >( [ this ]( JoltPhysicsContactPair& event )
		{
			m_pGameListener->ObjectEnterTrigger( event.pObject1, event.pObject2 );
		});

		// Send FluidStartTouch events
		m_FluidStartTouchEvents.ForEach< true >( [ this ]( JoltPhysicsContactPair& event )
		{
			m_pGameListener->FluidStartTouch( event.pObject2, event.pObject1->GetFluidController() );
		});

		// Send PostCollision events
		//
		// Clear it this time as we are done with these!
		m_CollisionEvents.ForEach< true >( [ this ]( JoltPhysicsCollisionEvent& event )
		{
			m_pGameListener->PostCollision( &event.m_Event );
		});

		// Send Friction events. The game uses these to drive scrape sounds and
		// particle effects on sustained sliding contacts.
		m_FrictionEvents.ForEach< true >( [ this ]( JoltPhysicsFrictionEvent& event )
		{
			JoltPhysicsObject *pObj1 = event.m_Data.GetPair().pObject1;
			JoltPhysicsObject *pObj2 = event.m_Data.GetPair().pObject2;
			const int nMtl1 = pObj1->GetMaterialIndex();
			const int nMtl2 = pObj2->GetMaterialIndex();
		#if !defined( GAME_GMOD_64X )
			JoltPhysicsSurfaceProps &props = JoltPhysicsSurfaceProps::GetInstance();
			const int nMtl1Hit = props.RemapIVPMaterialIndex( nMtl2 );
			const int nMtl2Hit = props.RemapIVPMaterialIndex( nMtl1 );
		#else
			const int nMtl1Hit = nMtl2;
			const int nMtl2Hit = nMtl1;
		#endif
			m_pGameListener->Friction( pObj1, event.m_flEnergy, nMtl1, nMtl1Hit, &event.m_Data );
			m_pGameListener->Friction( pObj2, event.m_flEnergy, nMtl2, nMtl2Hit, &event.m_Data );
		});

		// Send EndTouch events
		m_EndTouchEvents.ForEach< true >( [ this ]( JoltPhysicsCollisionData& event )
		{
			m_pGameListener->EndTouch( event.GetPair().pObject1, event.GetPair().pObject2, &event );
		});

		// Send LeaveTrigger events
		m_LeaveTriggerEvents.ForEach< true >( [ this ]( JoltPhysicsContactPair& event )
		{
			m_pGameListener->ObjectLeaveTrigger( event.pObject1, event.pObject2 );
		});

		// Send FluidEndTouch events
		m_FluidEndTouchEvents.ForEach< true >( [ this ]( JoltPhysicsContactPair& event )
		{
			m_pGameListener->FluidEndTouch( event.pObject2, event.pObject1->GetFluidController() );
		});

		// Reset the collision event counter.
		m_GlobalCollisionEventCount = 0u;
	}

	void PostSimulationFrame()
	{
		if ( m_pGameListener )
			m_pGameListener->PostSimulationFrame();
	}

private:

	static uint32 GetThreadId()
	{
		extern thread_local uint32 s_ThreadId;
		static std::atomic< uint32 > s_ThreadCtr = { 0u };
		if ( s_ThreadId == ~0u )
			s_ThreadId = s_ThreadCtr++;
		return s_ThreadId;
	}

	const JPH::PhysicsSystem &m_PhysicsSystem;

	IPhysicsCollisionEvent	*m_pGameListener = nullptr;
	IPhysicsCollisionSolver *m_pGameSolver = nullptr;

	std::mutex m_ShouldCollideLock;

	class JoltPhysicsCollisionInfo
	{
	public:
		JoltPhysicsCollisionInfo( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
			: m_CollisionPair{ pObject1, pObject2 }
		{
		}

		JoltPhysicsCollisionInfo( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2, const JPH::ContactManifold &inManifold )
			: m_CollisionPair{ pObject1, pObject2 }
			// Slart: Note this negated vector, it is important, Portal 2 bouncy paint needs it negated otherwise things fly into the surface they hit
			, m_SurfaceNormal( -Vector( inManifold.mWorldSpaceNormal.GetX(), inManifold.mWorldSpaceNormal.GetY(), inManifold.mWorldSpaceNormal.GetZ() ) )
			, m_ContactPoint( JoltToSource::Distance( inManifold.GetWorldSpaceContactPointOn1( 0 ) ) )
			// Unused...
			, m_ContactSpeed( vec3_origin )
			, m_Velocity0( pObject1->GetBody()->GetLinearVelocity() )
			, m_Velocity1( pObject2->GetBody()->GetLinearVelocity() )
		{
		}

		JoltPhysicsContactPair m_CollisionPair;

		Vector m_SurfaceNormal = vec3_origin;
		Vector m_ContactPoint  = vec3_origin;
		Vector m_ContactSpeed  = vec3_origin;

		JPH::Vec3 m_Velocity0 = JPH::Vec3::sZero();
		JPH::Vec3 m_Velocity1 = JPH::Vec3::sZero();
	};

	class JoltPhysicsCollisionData final : public IPhysicsCollisionData
	{
	public:
		JoltPhysicsCollisionData( const JoltPhysicsCollisionInfo &info )
			: m_CollisionData{ info }
		{
		}

		void GetSurfaceNormal( Vector &out ) override
		{
			out = m_CollisionData.m_SurfaceNormal;
		}

		void GetContactPoint( Vector &out ) override
		{
			out = m_CollisionData.m_ContactPoint;
		}

		void GetContactSpeed( Vector &out ) override
		{
			out = m_CollisionData.m_ContactSpeed;
		}

		JoltPhysicsContactPair GetPair() const
		{
			return m_CollisionData.m_CollisionPair;
		}

		JPH::Vec3 GetObject1PreCollisionVelocity() const
		{
			return m_CollisionData.m_Velocity0;
		}

		JPH::Vec3 GetObject2PreCollisionVelocity() const
		{
			return m_CollisionData.m_Velocity1;
		}
	private:
		JoltPhysicsCollisionInfo m_CollisionData;
	};

	class JoltPhysicsCollisionEvent
	{
	public:
		JoltPhysicsCollisionEvent( const JoltPhysicsCollisionInfo &info )
			: m_Data{ info }
		{
			JoltPhysicsObject *pObject1 = m_Data.GetPair().pObject1;
			JoltPhysicsObject* pObject2 = m_Data.GetPair().pObject2;

			m_Event.pObjects[0]			= pObject1;
			m_Event.pObjects[1]			= pObject2;
			m_Event.surfaceProps[0]		= pObject1->GetMaterialIndex();
			m_Event.surfaceProps[1]		= pObject2->GetMaterialIndex();
			m_Event.isCollision			= IsCollision( pObject1, pObject2 );
			m_Event.isShadowCollision	= IsShadowCollision( pObject1, pObject2 );
			m_Event.deltaCollisionTime	= 100.0f;
			m_Event.collisionSpeed		= 0.0f;
			m_Event.pInternalData		= &m_Data;
		}

		JoltPhysicsCollisionEvent( const JoltPhysicsCollisionEvent &other )
			: m_Event( other.m_Event )
			, m_Data ( other.m_Data )
		{
			// Re-target the event's internal data pointer to our own structure.
			m_Event.pInternalData = &m_Data;
		}

		JoltPhysicsCollisionEvent( JoltPhysicsCollisionEvent &&other )
			: m_Event( std::move( other.m_Event ) )
			, m_Data ( std::move( other.m_Data ) )
		{
			// Re-target the event's internal data pointer to our own structure.
			m_Event.pInternalData = &m_Data;
		}

		static bool IsCollision( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
		{
			bool bIsCollision = ( pObject1->GetCallbackFlags() & pObject2->GetCallbackFlags() ) & CALLBACK_GLOBAL_COLLISION;

			if ( pObject1->IsStatic() && !( pObject2->GetCallbackFlags() & CALLBACK_GLOBAL_COLLIDE_STATIC ) )
				bIsCollision = false;
			if ( pObject2->IsStatic() && !( pObject1->GetCallbackFlags() & CALLBACK_GLOBAL_COLLIDE_STATIC ) )
				bIsCollision = false;

			return bIsCollision;
		}

		static bool IsShadowCollision( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
		{
			return ( pObject1->GetCallbackFlags() ^ pObject2->GetCallbackFlags() ) & CALLBACK_SHADOW_COLLISION;
		}

		static float GetCollisionSpeed( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2, Vector vecNormal )
		{
			const Vector vecCollisionSpeed = pObject1->GetVelocity() - pObject2->GetVelocity();
			return fabsf( vecCollisionSpeed.Dot( vecNormal ) );
		}

		vcollisionevent_t			m_Event = {};
		JoltPhysicsCollisionData	m_Data;
	};

	class JoltPhysicsFrictionEvent
	{
	public:
		JoltPhysicsFrictionEvent( const JoltPhysicsCollisionInfo &info, float flEnergy )
			: m_Data( info )
			, m_flEnergy( flEnergy )
		{
		}

		JoltPhysicsCollisionData	m_Data;
		float						m_flEnergy = 0.0f;
	};

	template < typename Data >
	struct JoltPhysicsEventTracker
	{
	public:
		template < typename... T >
		void EmplaceBack( uint32 uThreadId, T&&... val)
		{
			m_Mask |= 1ull << uThreadId;
			m_Events[ uThreadId ].emplace_back( std::forward< T >( val )... );
		}

		template < bool bClear, typename FuncType >
		void ForEach( FuncType func )
		{
			for ( uint32 thread = m_Mask; thread; thread &= thread - 1 )
			{
				const uint32 i = JPH::CountTrailingZeros( thread );
				for ( auto &event : m_Events[ i ] )
					func( event );

				if constexpr ( bClear )
					m_Events[ i ].clear();
			}

			if constexpr ( bClear )
				m_Mask = 0ull;
		}

	private:
		static constexpr uint32 kMaxThreads = 64;
		std::atomic< uint64_t >	m_Mask = { 0ull };
		std::vector< Data >		m_Events[ kMaxThreads ];
	};

	// The maximum number of sent collision events to send per-frame.
	// This is used to play stuff like sounds and physics fx.
	// This is quite expensive to do so, we rate-limit this quite aggressively.
	static constexpr uint32_t MaxCollisionEvents = 4;
	std::atomic< uint32 > m_GlobalCollisionEventCount = { 0u };

	JoltPhysicsEventTracker< JoltPhysicsCollisionEvent >	m_CollisionEvents;
	JoltPhysicsEventTracker< JoltPhysicsFrictionEvent >		m_FrictionEvents;

	JoltPhysicsEventTracker< JoltPhysicsCollisionData >		m_StartTouchEvents;
	JoltPhysicsEventTracker< JoltPhysicsCollisionData >		m_EndTouchEvents;

	JoltPhysicsEventTracker< JoltPhysicsContactPair >		m_EnterTriggerEvents;
	JoltPhysicsEventTracker< JoltPhysicsContactPair >		m_LeaveTriggerEvents;

	// For the fluid events:
	//   Object1 = the fluid
	//   Object2 = the object
	JoltPhysicsEventTracker< JoltPhysicsContactPair >		m_FluidStartTouchEvents;
	JoltPhysicsEventTracker< JoltPhysicsContactPair >		m_FluidEndTouchEvents;

};
