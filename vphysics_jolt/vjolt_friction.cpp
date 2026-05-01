
#include "cbase.h"

#include "vjolt_friction.h"
#include "vjolt_layers.h"
#include "vjolt_object.h"
#include "vjolt_environment.h"
#include "vjolt_listener_contact.h"
#include "vjolt_surfaceprops.h"

#include "tier0/memdbgon.h"

namespace
{
	class FrictionSnapshotCollector final : public JPH::CollideShapeCollector
	{
	public:
		FrictionSnapshotCollector( JoltPhysicsObject *pSelf, JPH::PhysicsSystem *pSystem,
			std::vector< JoltPhysicsFrictionSnapshot::Contact > &contacts )
			: m_pSelf( pSelf )
			, m_pSystem( pSystem )
			, m_contacts( contacts )
		{
		}

		void AddHit( const JPH::CollideShapeResult &inResult ) override
		{
			JPH::BodyLockRead lock( m_pSystem->GetBodyLockInterfaceNoLock(), inResult.mBodyID2 );
			if ( !lock.Succeeded() )
				return;

			const JPH::Body &body = lock.GetBody();
			JoltPhysicsObject *pOther = reinterpret_cast< JoltPhysicsObject * >( body.GetUserData() );
			if ( !pOther || pOther == m_pSelf )
				return;

			JoltPhysicsContactListener *pListener = m_pSelf->GetJoltEnvironment()->GetContactListener();
			if ( !pListener->ShouldCollide( m_pSelf, pOther ) )
				return;

			JPH::Vec3 axis = inResult.mPenetrationAxis.NormalizedOr( JPH::Vec3::sAxisZ() );

			JoltPhysicsFrictionSnapshot::Contact c;
			c.pOther = pOther;
			c.vNormal = Vector( axis.GetX(), axis.GetY(), axis.GetZ() );
			c.vContactPoint = JoltToSource::Distance( inResult.mContactPointOn2 );
			c.flPenetrationDepth = inResult.mPenetrationDepth;
			m_contacts.push_back( c );
		}

	private:
		JoltPhysicsObject *m_pSelf;
		JPH::PhysicsSystem *m_pSystem;
		std::vector< JoltPhysicsFrictionSnapshot::Contact > &m_contacts;
	};

	class FrictionSnapshotFilter final : public JPH::BodyFilter
	{
	public:
		explicit FrictionSnapshotFilter( JoltPhysicsObject *pSelf ) : m_pSelf( pSelf ) {}

		bool ShouldCollideLocked( const JPH::Body &inBody ) const override
		{
			JoltPhysicsObject *pObject = reinterpret_cast< JoltPhysicsObject * >( inBody.GetUserData() );
			return pObject != m_pSelf;
		}

	private:
		JoltPhysicsObject *m_pSelf;
	};
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsFrictionSnapshot::JoltPhysicsFrictionSnapshot( JoltPhysicsObject *pObject )
	: m_pSelf( pObject )
{
	if ( !pObject || !pObject->IsCollisionEnabled() )
		return;

	JPH::PhysicsSystem *pSystem = pObject->GetJoltEnvironment()->GetPhysicsSystem();

	JPH::DefaultBroadPhaseLayerFilter broadphase_layer_filter = pSystem->GetDefaultBroadPhaseLayerFilter( Layers::MOVING );
	JPH::DefaultObjectLayerFilter object_layer_filter = pSystem->GetDefaultLayerFilter( Layers::MOVING );

	JPH::Vec3 position;
	JPH::Quat rotation;
	JPH::BodyInterface &bi = pSystem->GetBodyInterfaceNoLock();
	bi.GetPositionAndRotation( pObject->GetBodyID(), position, rotation );
	JPH::Mat44 query_transform = JPH::Mat44::sRotationTranslation(
		rotation, position + rotation * pObject->GetBody()->GetShape()->GetCenterOfMass() );

	JPH::CollideShapeSettings settings;
	settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;
	settings.mActiveEdgeMovementDirection = bi.GetLinearVelocity( pObject->GetBodyID() );
	settings.mBackFaceMode = JPH::EBackFaceMode::IgnoreBackFaces;
	settings.mMaxSeparationDistance = 0.025f;

	FrictionSnapshotCollector collector( pObject, pSystem, m_contacts );
	FrictionSnapshotFilter filter( pObject );

	pSystem->GetNarrowPhaseQueryNoLock().CollideShape(
		pObject->GetBody()->GetShape(),
		JPH::Vec3::sReplicate( 1.0f ),
		query_transform,
		settings,
		JPH::Vec3::sZero(),
		collector,
		broadphase_layer_filter,
		object_layer_filter,
		filter );
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsFrictionSnapshot::IsValid()
{
	return m_index < m_contacts.size();
}

//-------------------------------------------------------------------------------------------------

IPhysicsObject *JoltPhysicsFrictionSnapshot::GetObject( int index )
{
	if ( !IsValid() )
		return nullptr;
	return ( index == 0 ) ? static_cast< IPhysicsObject * >( m_pSelf )
	                      : static_cast< IPhysicsObject * >( m_contacts[ m_index ].pOther );
}

int JoltPhysicsFrictionSnapshot::GetMaterial( int index )
{
	if ( !IsValid() )
		return 0;
	return ( index == 0 ) ? m_pSelf->GetMaterialIndex()
	                      : m_contacts[ m_index ].pOther->GetMaterialIndex();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::GetContactPoint( Vector &out )
{
	if ( !IsValid() )
	{
		out.Zero();
		return;
	}
	out = m_contacts[ m_index ].vContactPoint;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::GetSurfaceNormal( Vector &out )
{
	if ( !IsValid() )
	{
		out.Zero();
		return;
	}
	out = m_contacts[ m_index ].vNormal;
}

float JoltPhysicsFrictionSnapshot::GetNormalForce()
{
	return 0.0f;
}

float JoltPhysicsFrictionSnapshot::GetEnergyAbsorbed()
{
	return 0.0f;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::RecomputeFriction()
{
}

void JoltPhysicsFrictionSnapshot::ClearFrictionForce()
{
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::MarkContactForDelete()
{
}

void JoltPhysicsFrictionSnapshot::DeleteAllMarkedContacts( bool wakeObjects )
{
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::NextFrictionData()
{
	if ( m_index < m_contacts.size() )
		m_index++;
}

float JoltPhysicsFrictionSnapshot::GetFrictionCoefficient()
{
	if ( !IsValid() )
		return 0.0f;

	int matSelf = m_pSelf->GetMaterialIndex();
	int matOther = m_contacts[ m_index ].pOther->GetMaterialIndex();

	surfacedata_t *pSelf = JoltPhysicsSurfaceProps::GetInstance().GetSurfaceData( matSelf );
	surfacedata_t *pOther = JoltPhysicsSurfaceProps::GetInstance().GetSurfaceData( matOther );
	if ( !pSelf || !pOther )
		return 0.0f;

	return pSelf->physics.friction * pOther->physics.friction;
}
