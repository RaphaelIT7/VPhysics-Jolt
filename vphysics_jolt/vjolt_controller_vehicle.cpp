
#include "cbase.h"

#include "vjolt_layers.h"

#include "vjolt_controller_vehicle.h"
#include "vjolt_surfaceprops.h"

#include "cmodel.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//------------------------------------------------------------------------------------------------

static ConVar vjolt_vehicle_wheel_debug( "vjolt_vehicle_wheel_debug", "0", FCVAR_CHEAT );

static ConVar vjolt_vehicle_throttle_opposition_limit( "vjolt_vehicle_throttle_opposition_limit", "5", FCVAR_NONE,
	"Below what speed should we be attempting to drive/climb with handbrake on to avoid falling down." );

static ConVar vjolt_vehicle_disable_handbrakes( "vjolt_vehicle_disable_handbrakes", "0", FCVAR_NONE );

static ConVar vjolt_vehicle_disable_autobrake( "vjolt_vehicle_disable_autobrake", "0", FCVAR_NONE );
static ConVar vjolt_vehicle_disable_brake( "vjolt_vehicle_disable_brake", "0", FCVAR_NONE );

static ConVar vjolt_vehicle_throttle_override( "vjolt_vehicle_throttle_override", "-1.0", FCVAR_NONE );

static ConVar vjolt_airboat_debug( "vjolt_airboat_debug", "0", FCVAR_NONE );

//------------------------------------------------------------------------------------------------

static const JPH::Vec3 VehicleUpVector		= JPH::Vec3( 0, 0, 1 );
static const JPH::Vec3 VehicleForwardVector	= JPH::Vec3( 0, 1, 0 );

constexpr float MPH_TO_METERSPERSECOND = 0.44707f;
inline float MphToGameVel( float mph ) { return mph * MPH_TO_METERSPERSECOND * JoltToSource::Factor; }

#ifdef _X360
	#define AIRBOAT_STEERING_RATE_MIN			0.000225f
	#define AIRBOAT_STEERING_RATE_MAX			(10.0f * AIRBOAT_STEERING_RATE_MIN)
	#define AIRBOAT_STEERING_INTERVAL			1.5f
#else
	#define AIRBOAT_STEERING_RATE_MIN			0.00045f
	#define AIRBOAT_STEERING_RATE_MAX			(5.0f * AIRBOAT_STEERING_RATE_MIN)
	#define AIRBOAT_STEERING_INTERVAL			0.5f
#endif

#define AIRBOAT_ROT_DRAG					0.00004f
#define AIRBOAT_ROT_DAMPING					0.001f

#define AIRBOAT_THRUST_MAX					11.0f
#define AIRBOAT_THRUST_MAX_REVERSE			7.5f

#define AIRBOAT_WATER_DRAG_LEFT_RIGHT		0.6f
#define AIRBOAT_WATER_DRAG_FORWARD_BACK		0.005f
#define AIRBOAT_WATER_DRAG_UP_DOWN			0.0025f

#define AIRBOAT_GROUND_DRAG_LEFT_RIGHT		2.0f
#define AIRBOAT_GROUND_DRAG_FORWARD_BACK	1.0f
#define AIRBOAT_GROUND_DRAG_UP_DOWN			0.8f

#define AIRBOAT_DRY_FRICTION_SCALE			0.6f

#define AIRBOAT_RAYCAST_DIST				0.35f
#define AIRBOAT_RAYCAST_DIST_WATER_LOW		0.1f
#define AIRBOAT_RAYCAST_DIST_WATER_HIGH		0.35f

#define AIRBOAT_WATER_NOISE_MIN				0.01f
#define AIRBOAT_WATER_NOISE_MAX				0.03f
#define AIRBOAT_WATER_FREQ_MIN				1.5f
#define AIRBOAT_WATER_FREQ_MAX				1.5f
#define AIRBOAT_WATER_PHASE_MIN				0.0f
#define AIRBOAT_WATER_PHASE_MAX				1.5f

#define AIRBOAT_GRAVITY						9.81f

#define AIRBOAT_BUOYANCY_SCALAR				1.6f
#define AIRBOAT_PONTOON_AREA_2D				2.8f
#define AIRBOAT_PONTOON_HEIGHT				0.41f

static const char *VehicleTypeToName( unsigned int VehicleType )
{
	switch ( VehicleType )
	{
		case VEHICLE_TYPE_CAR_WHEELS:		return "Car Wheels";
		case VEHICLE_TYPE_CAR_RAYCAST:		return "Car Raycast";
		case VEHICLE_TYPE_JETSKI_RAYCAST:	return "Jetski Raycast";
		case VEHICLE_TYPE_AIRBOAT_RAYCAST:	return "Airboat Raycast";
		default:							return "Unknown";
	}
}

JPH::Ref< JPH::VehicleCollisionTester > CreateVehicleCollisionTester( unsigned int VehicleType, float LargestWheelRadius )
{
	switch ( VehicleType )
	{
	default:
		Log_Warning( LOG_VJolt, "Don't know how to make vehicle type: %s (%u).\n", VehicleTypeToName( VehicleType ), VehicleType );
		[[ fallthrough ]];
	case VEHICLE_TYPE_CAR_WHEELS:
		return new JPH::VehicleCollisionTesterCastSphere( Layers::MOVING, LargestWheelRadius, VehicleUpVector );
	}
}

//------------------------------------------------------------------------------------------------

JoltPhysicsVehicleController::JoltPhysicsVehicleController( JoltPhysicsEnvironment* pEnvironment, JPH::PhysicsSystem* pPhysicsSystem, JoltPhysicsObject* pVehicleBodyObject, const vehicleparams_t& params, unsigned int nVehicleType, IPhysicsGameTrace* pGameTrace )
	: m_pEnvironment( pEnvironment )
	, m_pPhysicsSystem( pPhysicsSystem )
	, m_pCarBodyObject( pVehicleBodyObject )
	, m_pGameTrace( pGameTrace )
	, m_VehicleType( nVehicleType )
	, m_VehicleParams( params )
{
	m_pCarBodyObject->AddDestroyedListener( this );

	VehicleDataReload();

	if ( IsAirboat() )
	{
		InitAirboat( params );
		return;
	}

	JPH::VehicleConstraintSettings vehicle;
	vehicle.mUp					= VehicleUpVector;
	vehicle.mForward			= VehicleForwardVector;
	vehicle.mDrawConstraintSize = 0.1f;
	CreateWheels( vehicle );
	vehicle.mController			= CreateVehicleController();

	m_Tester = CreateVehicleCollisionTester( nVehicleType, m_InternalState.LargestWheelRadius );

	m_VehicleConstraint = new JPH::VehicleConstraint( *m_pCarBodyObject->GetBody(), vehicle );
	m_pPhysicsSystem->AddConstraint( m_VehicleConstraint );
	m_pPhysicsSystem->AddStepListener( m_VehicleConstraint );
}

JoltPhysicsVehicleController::~JoltPhysicsVehicleController()
{
	DetachObject();

	for ( auto &wheel : m_Wheels )
		m_pEnvironment->DestroyObject( wheel.pObject );
	m_Wheels.clear();
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::Update( float dt, vehicle_controlparams_t &controls )
{
	m_ControlParams = controls;

	UpdateBooster( dt );

	HandleBoostKey();

	if ( IsAirboat() )
		AirboatUpdate( dt, controls );
}

const vehicle_operatingparams_t &JoltPhysicsVehicleController::GetOperatingParams()
{
	return m_OperatingParams;
}

const vehicleparams_t &JoltPhysicsVehicleController::GetVehicleParams()
{
	return m_VehicleParams;
}

vehicleparams_t &JoltPhysicsVehicleController::GetVehicleParamsForChange()
{
	return m_VehicleParams;
}

float JoltPhysicsVehicleController::UpdateBooster( float dt )
{
	m_InternalState.BoostDelay				= Max( m_InternalState.BoostDelay - dt, 0.0f );
	m_InternalState.BoosterRemainingTime	= Max( m_InternalState.BoosterRemainingTime - dt, 0.0f );

	return m_InternalState.BoostDelay;
}

int JoltPhysicsVehicleController::GetWheelCount()
{
	if ( IsAirboat() )
		return m_nAirboatPontoons;
	return int( m_Wheels.size() );
}

IPhysicsObject *JoltPhysicsVehicleController::GetWheel( int index )
{
	if ( IsAirboat() )
		return nullptr;

	if ( index >= int( m_Wheels.size() ) )
		return nullptr;

	return m_Wheels[ index ].pObject;
}

bool JoltPhysicsVehicleController::GetWheelContactPoint( int index, Vector *pContactPoint, int *pSurfaceProps )
{
	if ( IsAirboat() )
	{
		if ( index < 0 || index >= m_nAirboatPontoons )
		{
			if ( pContactPoint ) *pContactPoint = vec3_origin;
			if ( pSurfaceProps ) *pSurfaceProps = 0;
			return false;
		}

		const JoltAirboatImpact &impact = m_AirboatImpacts[ index ];
		if ( impact.bImpact )
		{
			if ( pContactPoint ) *pContactPoint = impact.vecImpactPointWS;
			if ( pSurfaceProps ) *pSurfaceProps = impact.nSurfaceProps;
			return true;
		}

		if ( pContactPoint ) *pContactPoint = vec3_origin;
		if ( pSurfaceProps ) *pSurfaceProps = 0;
		return false;
	}

	if ( index < int( m_Wheels.size() ) && m_VehicleConstraint->GetWheels()[ index ]->HasContact() )
	{
		if ( pContactPoint )
			*pContactPoint = JoltToSource::Distance( m_VehicleConstraint->GetWheels()[ index ]->GetContactPosition() );

		// TODO(Josh): This!
		if ( pSurfaceProps )
			*pSurfaceProps = 0;

		return true;
	}
	else
	{
		if ( pContactPoint )
			*pContactPoint = vec3_origin;

		if ( pSurfaceProps )
			*pSurfaceProps = 0;

		return false;
	}
}

void JoltPhysicsVehicleController::SetSpringLength( int wheelIndex, float length )
{

}

void JoltPhysicsVehicleController::SetWheelFriction( int wheelIndex, float friction )
{
	if ( IsAirboat() )
	{
		if ( wheelIndex >= 0 && wheelIndex < m_nAirboatPontoons )
			m_AirboatPontoons[ wheelIndex ].friction_of_wheel = friction;
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::OnVehicleEnter()
{
	// Undo any damping we may have set to slow the boat when
	// we got out.
	if ( m_VehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST )
	{
		float flDampSpeed = 0.0f;
		float flDampRotSpeed = 0.0f;
		m_pCarBodyObject->SetDamping( &flDampSpeed, &flDampRotSpeed );
	}
}

void JoltPhysicsVehicleController::OnVehicleExit()
{
	// If we are an airboat, set a bunch of damping to slow us down.
	if ( m_VehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST )
	{
		float flDampSpeed = 1.0f;
		float flDampRotSpeed = 1.0f;
		m_pCarBodyObject->SetDamping( &flDampSpeed, &flDampRotSpeed );
	}

	SetEngineDisabled( false );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::SetEngineDisabled( bool bDisable )
{
	m_InternalState.EngineDisabled = bDisable;
}

bool JoltPhysicsVehicleController::IsEngineDisabled()
{
	return m_InternalState.EngineDisabled;
}

void JoltPhysicsVehicleController::GetCarSystemDebugData( vehicle_debugcarsystem_t &debugCarSystem )
{

}

void JoltPhysicsVehicleController::VehicleDataReload()
{
	m_VehicleParams.engine.maxSpeed		= MphToGameVel( m_VehicleParams.engine.maxSpeed );
	m_VehicleParams.engine.maxRevSpeed	= MphToGameVel( m_VehicleParams.engine.maxRevSpeed );
	m_VehicleParams.engine.boostMaxSpeed = MphToGameVel( m_VehicleParams.engine.boostMaxSpeed );
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsVehicleController::GetSpeed()
{
	const Vector orientation = GetColumn( GetBodyMatrix(), MatrixAxis::Left );
	return orientation.Dot( m_pCarBodyObject->GetVelocity() );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::HandleBoostKey()
{
	// Handle triggering boosting if the key is pressed and we aren't currently boosting or in cooldown.
	if ( m_ControlParams.boost && !m_InternalState.BoostDelay && !m_InternalState.BoosterRemainingTime )
	{
		m_InternalState.BoosterRemainingTime	= m_VehicleParams.engine.boostDuration;
		m_InternalState.BoostDelay				= m_VehicleParams.engine.boostDuration + m_VehicleParams.engine.boostDelay;
	}
}

void JoltPhysicsVehicleController::HandleBoostDecay()
{
	// Decay the boost time if we are currently boosting or have a delay.
	if ( m_VehicleParams.engine.boostDuration || m_VehicleParams.engine.boostDelay )
	{
		m_OperatingParams.boostTimeLeft = m_InternalState.BoostDelay
			? 100.0f - ( 100.0f * ( m_InternalState.BoostDelay / ( m_VehicleParams.engine.boostDuration + m_VehicleParams.engine.boostDelay ) ) )
			: 100.0f;
	}
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::OnPreSimulate( float flDeltaTime )
{
	if ( IsAirboat() )
	{
		AirboatOnPreSimulate( flDeltaTime );
		return;
	}

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	// With any user input, assure that the car is active
	if ( m_ControlParams.steering != 0.0f || m_ControlParams.throttle != 0.0f || m_ControlParams.brake != 0.0f || m_ControlParams.handbrake )
		bodyInterface.ActivateBody( m_pCarBodyObject->GetBodyID() );

	bool bHandbrake = m_ControlParams.handbrake && !vjolt_vehicle_disable_handbrakes.GetBool();

	// Don't throttle when holding handbrake (like Source)
	float flThrottle = bHandbrake ? 0.0f : m_ControlParams.throttle;

	if ( vjolt_vehicle_throttle_override.GetFloat() > 0.0f )
		flThrottle = vjolt_vehicle_throttle_override.GetFloat();

	// Apply a little brake without throttle to stop the vehicle from coasting (like Source).
	const bool bCoasting = flThrottle == 0.0f && m_ControlParams.brake == 0.0f && !bHandbrake;
	float flBrake = bCoasting && !vjolt_vehicle_disable_autobrake.GetBool() ? 0.1f : m_ControlParams.brake;

	if ( vjolt_vehicle_disable_brake.GetBool() )
		flBrake = 0.0f;

	const float ThrottleOpositionSpeed = vjolt_vehicle_throttle_opposition_limit.GetFloat();

	// Enable the handbrake when going at low speeds to avoid slipping when going up hill.
	if ( ( flThrottle < 0.0f && m_OperatingParams.speed > ThrottleOpositionSpeed ) ||
		( flThrottle > 0.0f && m_OperatingParams.speed < -ThrottleOpositionSpeed ) )
		bHandbrake = !vjolt_vehicle_disable_handbrakes.GetBool();

	// Are we boosting?
	float flTotalTorqueMultiplier = 1.0f;
	if ( m_InternalState.BoosterRemainingTime != 0.0f )
	{
		GetWheeledVehicleController()->GetEngine().SetCurrentRPM(m_VehicleParams.engine.maxRPM);
		// Slam the throttle to 1, neeeowm!
		m_ControlParams.throttle = 1.0f;
		flThrottle = 1.0f;

		const float flSpeedFactor = RemapValClamped( fabsf( m_OperatingParams.speed ), 0, m_VehicleParams.engine.maxSpeed, 0.1f, 1.0f );
		const float flTurnFactor = 1.0f - ( fabsf( m_ControlParams.steering ) * 0.95f );
		// Josh: * 2 as the original torque stuff in Source was based around 0.5 being the max, and 1.0 being boost.
		const float flDampedBoost = 2.0f * m_VehicleParams.engine.boostForce * flSpeedFactor * flTurnFactor;

		if ( flDampedBoost > flTotalTorqueMultiplier )
			flTotalTorqueMultiplier = flDampedBoost;
	}

	// Update the torque factors as we may be boosting and be > 1.
	// TODO(Josh): More than 2 wheels per axle.
	VJoltAssert( m_VehicleParams.wheelsPerAxle == 2 );
	for ( int i = 0; i < m_VehicleParams.axleCount; i++ )
		GetWheeledVehicleController()->GetDifferentials()[i].mEngineTorqueRatio = flTotalTorqueMultiplier * m_VehicleParams.axles[i].torqueFactor;

	// Pass the input on to the constraint
	GetWheeledVehicleController()->SetDriverInput( flThrottle, m_ControlParams.steering, flBrake, bHandbrake ? 1.0f : 0.0f );

	// Set the collision tester
	m_VehicleConstraint->SetVehicleCollisionTester( m_Tester );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::OnPostSimulate( float flDeltaTime )
{
	if ( IsAirboat() )
	{
		m_OperatingParams.speed = GetSpeed();
		m_OperatingParams.steeringAngle = RAD2DEG( m_AirboatState.m_SteeringAngle );
		m_OperatingParams.wheelsInContact = AirboatCountSurfaceContacts( m_AirboatImpacts );
		m_OperatingParams.wheelsNotInContact = m_nAirboatPontoons - m_OperatingParams.wheelsInContact;
		m_OperatingParams.boostDelay = m_InternalState.BoostDelay;
		HandleBoostDecay();
		return;
	}

	// Draw our wheels (this needs to be done in the pre update since we draw the bodies too in the state before the step)

	float flSteeringAngle = 0.0f;
	m_OperatingParams.wheelsInContact = 0;
	m_OperatingParams.wheelsNotInContact = 0;
	for ( int w = 0; w < GetWheelCount(); w++ )
	{
		const JPH::WheelSettings *settings = m_VehicleConstraint->GetWheels()[w]->GetSettings();
		// The cyclinder we draw is aligned with Y so we specify that as rotational axis
		JPH::Mat44 wheelTransform = m_VehicleConstraint->GetWheelWorldTransform( w, JPH::Vec3( 1, 0, 0 ),  JPH::Vec3( 0, 0, 1 ) );

		// Find our greatest steering angle.
		float flWheelSteeringAngle = JoltToSource::Angle( m_VehicleConstraint->GetWheels()[w]->GetSteerAngle() );
		if ( fabsf( flWheelSteeringAngle ) > fabsf( flSteeringAngle ) )
			flSteeringAngle = flWheelSteeringAngle;

		Vector newPos = JoltToSource::Distance( wheelTransform.GetTranslation() );
		// TODO(Josh): This triggers JPH_ASSERT(mCol[3] == Vec4(0, 0, 0, 1));
		// what to do about that?..
		// We just want the local rotation, and this seems to work (?)
		QAngle newQuat = JoltToSource::Angle( wheelTransform.GetQuaternion() );
		m_Wheels[ w ].pObject->EnableCollisions( false );
		// Set dummy wheel object pos/angles so the game code can update pose positions for wheels.
		m_Wheels[ w ].pObject->SetPosition( newPos, newQuat, true );
		// Wake it up so that the game bothers to do pose positions.
		m_Wheels[ w ].pObject->Wake();

		if ( m_VehicleConstraint->GetWheels()[w]->HasContact() )
			m_OperatingParams.wheelsInContact++;
		else
			m_OperatingParams.wheelsNotInContact++;

		IVJoltDebugOverlay *pDebugOverlay = JoltPhysicsInterface::GetInstance().GetDebugOverlay();
		if ( vjolt_vehicle_wheel_debug.GetBool() && pDebugOverlay )
		{
			const Vector vecWheelPos = JoltToSource::Distance( wheelTransform.GetTranslation() );
			const Vector vecWheelSize = JoltToSource::Distance( JPH::Vec3( settings->mWidth / 2.0f, settings->mRadius, settings->mRadius ) );

			pDebugOverlay->AddBoxOverlay(
				vecWheelPos,
				-vecWheelSize, vecWheelSize,
				newQuat,
				255, 0, 255, 100,
				-1.0f );
		}
	}

	m_OperatingParams.gear			= GetWheeledVehicleController()->GetTransmission().GetCurrentGear();
	m_OperatingParams.engineRPM		= GetWheeledVehicleController()->GetEngine().GetCurrentRPM();
	m_OperatingParams.speed			= GetSpeed();
	m_OperatingParams.steeringAngle = -flSteeringAngle;
	m_OperatingParams.boostDelay	= m_InternalState.BoostDelay;
	HandleBoostDecay();
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::CreateWheel( JPH::VehicleConstraintSettings &vehicleSettings, matrix3x4_t& bodyMatrix, int axleIdx, int wheelIdx )
{
	const vehicle_axleparams_t &axle = m_VehicleParams.axles[ axleIdx ];

	const Vector wheelPositionLocal = axle.offset +
		( ( wheelIdx % 2 == 1 ) ? axle.wheelOffset : -axle.wheelOffset );

	Vector wheelPositionWorld;
	VectorTransform( wheelPositionLocal, bodyMatrix, wheelPositionWorld );

	// Josh: Good enough heuristic.
	const float wheelRadius = axle.wheels.radius;
	const float wheelWidth = wheelRadius / 2.0f;

	// Josh: Area of a cylinder = π.h.r^2
	// Using radius in terms of Source units as we pass this to CreateSphereObject.
	const float wheelVolume = M_PI * wheelWidth * Cube( wheelRadius );

	{
		objectparams_t wheelParams =
		{
			.mass		= axle.wheels.mass,
			.inertia	= axle.wheels.inertia,
			.damping	= axle.wheels.damping,
			.rotdamping	= axle.wheels.rotdamping,
			.pName		= "VehicleWheel",
			.pGameData	= m_pCarBodyObject->GetGameData(),
			.volume		= wheelVolume,
		};
		IPhysicsObject *pWheelObject = m_pEnvironment->CreateSphereObject(
			wheelRadius, axle.wheels.materialIndex,
			wheelPositionWorld, QAngle(),
			&wheelParams, false );

		JoltPhysicsObject *pJoltWheelObject = static_cast< JoltPhysicsObject * >( pWheelObject );

		pJoltWheelObject->SetGameFlags( m_pCarBodyObject->GetGameFlags() );
		pJoltWheelObject->SetCallbackFlags( CALLBACK_IS_VEHICLE_WHEEL );
		// Josh: The wheel is a fake object, so disable collisions on it.
		pJoltWheelObject->EnableCollisions( false );

		m_Wheels.push_back( JoltPhysicsWheel{ .pObject = pJoltWheelObject } );
	}

	const float steeringAngle = DEG2RAD( Max( m_VehicleParams.steering.degreesSlow, m_VehicleParams.steering.degreesFast ) );
	const float additionalLength = SourceToJolt::Distance( axle.wheels.springAdditionalLength );

	Vector gravity;
	m_pEnvironment->GetGravity( &gravity );

	JPH::WheelSettingsWV *wheelSettings = new JPH::WheelSettingsWV;
	wheelSettings->mPosition			= SourceToJolt::Distance( wheelPositionLocal );
	wheelSettings->mSuspensionDirection = JPH::Vec3( 0, 0, -1 );
	wheelSettings->mSteeringAxis		= JPH::Vec3( 0, 0, 1 );
	wheelSettings->mWheelUp				= JPH::Vec3( 0, 0, 1 );
	wheelSettings->mWheelForward		= JPH::Vec3( 0, 1, 0 );
	wheelSettings->mAngularDamping		= axle.wheels.rotdamping;
	// TODO(Josh): What about more than 4 wheels?
	wheelSettings->mMaxSteerAngle		= axleIdx == 0 ? steeringAngle : 0.0f;
	wheelSettings->mRadius				= SourceToJolt::Distance( axle.wheels.radius );
	wheelSettings->mWidth				= SourceToJolt::Distance( wheelWidth );
	wheelSettings->mInertia				= 0.5f * axle.wheels.mass * ( wheelSettings->mRadius * wheelSettings->mRadius );
	wheelSettings->mSuspensionMinLength = 0;
	wheelSettings->mSuspensionMaxLength = additionalLength;
	wheelSettings->mSuspensionSpring.mMode = JPH::ESpringMode::StiffnessAndDamping;
	// Source has these divided by the mass of the vehicle for some reason.
	// Convert these to a stiffness of k, in N/m...
	wheelSettings->mSuspensionSpring.mStiffness = axle.suspension.springConstant * m_pCarBodyObject->GetMass();
	wheelSettings->mSuspensionSpring.mDamping = axle.suspension.springDamping * m_pCarBodyObject->GetMass();
	if ( axle.wheels.frictionScale )
	{
		wheelSettings->mLateralFriction.AddPoint( 1.0f, axle.wheels.frictionScale );
		wheelSettings->mLongitudinalFriction.AddPoint( 1.0f, axle.wheels.frictionScale );
	}

	// TODO: We may want to update this every pre-simulation to account for changing gravity.
	wheelSettings->mMaxBrakeTorque =
		0.5f *
		SourceToJolt::Distance( gravity.Length() ) *
		( m_pCarBodyObject->GetMass() + m_TotalWheelMass ) *
		axle.brakeFactor *
		SourceToJolt::Distance( axle.wheels.radius );

	vehicleSettings.mWheels.push_back( wheelSettings );
	m_InternalState.LargestWheelRadius = Max( m_InternalState.LargestWheelRadius, SourceToJolt::Distance( wheelWidth ) );
}

void JoltPhysicsVehicleController::CreateWheels( JPH::VehicleConstraintSettings &vehicleSettings )
{
	matrix3x4_t carBodyMtx = GetBodyMatrix();

	m_Wheels.reserve( m_VehicleParams.axleCount * m_VehicleParams.wheelsPerAxle );
	vehicleSettings.mAntiRollBars.reserve( m_VehicleParams.axleCount );

	m_TotalWheelMass = 0.0f;
	for ( int axle = 0; axle < m_VehicleParams.axleCount; axle++ )
		m_TotalWheelMass += m_VehicleParams.axles[ axle ].wheels.mass * m_VehicleParams.wheelsPerAxle;

	for ( int axle = 0; axle < m_VehicleParams.axleCount; axle++ )
	{
		for ( int wheel = 0; wheel < m_VehicleParams.wheelsPerAxle; wheel++ )
			CreateWheel( vehicleSettings, carBodyMtx, axle, wheel );

		// TODO(Josh): More than 2 wheels per axle.
		VJoltAssert( m_VehicleParams.wheelsPerAxle == 2 );
		JPH::VehicleAntiRollBar rollbar;
		rollbar.mLeftWheel	= ( axle * m_VehicleParams.wheelsPerAxle );
		rollbar.mRightWheel	= ( axle * m_VehicleParams.wheelsPerAxle ) + 1;
		vehicleSettings.mAntiRollBars.push_back( rollbar );
	}
}

JPH::WheeledVehicleControllerSettings *JoltPhysicsVehicleController::CreateVehicleController()
{
	static constexpr float HorsePowerToWatts = 745.7f;

	JPH::WheeledVehicleControllerSettings *pController = new JPH::WheeledVehicleControllerSettings;
	// Josh:
	// T = ( 745.7 * P ) / ( 2 * PI * ( RPM / 60 ) )
	pController->mEngine.mMaxTorque = ( HorsePowerToWatts * m_VehicleParams.engine.horsepower ) / ( 2.0f * M_PI * ( m_VehicleParams.engine.maxRPM / 60.0f ) );
	// Josh: Fudge
	pController->mEngine.mMinRPM = Max( m_VehicleParams.engine.shiftDownRPM - 300, 0.0f );
	pController->mEngine.mMaxRPM = m_VehicleParams.engine.maxRPM;
	pController->mEngine.mAngularDamping = 0.0f;

	// Some vehicle scripts define just a single gear and have Autotransmission set to 0, in jolt the gear 0 is
	// neutral so it never moves, also some data like MaxSpeed doesn't really align, this has to do.
	pController->mTransmission.mMode = JPH::ETransmissionMode::Auto;
	pController->mTransmission.mGearRatios.clear();
	for ( int i = 0; i < m_VehicleParams.engine.gearCount; i++ )
		pController->mTransmission.mGearRatios.push_back( m_VehicleParams.engine.gearRatio[ i ] );

	pController->mTransmission.mReverseGearRatios.clear();
	pController->mTransmission.mReverseGearRatios.push_back( -m_VehicleParams.engine.gearRatio[0] );

	pController->mTransmission.mShiftUpRPM = m_VehicleParams.engine.shiftUpRPM;
	pController->mTransmission.mShiftDownRPM = m_VehicleParams.engine.shiftDownRPM;

	pController->mDifferentials.reserve( m_VehicleParams.axleCount );
	for ( int i = 0; i < m_VehicleParams.axleCount; i++ )
	{
		// TODO(Josh): More than 2 wheels per axle.
		VJoltAssert( m_VehicleParams.wheelsPerAxle == 2 );
		JPH::VehicleDifferentialSettings differential;
		differential.mLeftWheel			= ( i * m_VehicleParams.wheelsPerAxle );
		differential.mRightWheel		= ( i * m_VehicleParams.wheelsPerAxle ) + 1;
		differential.mEngineTorqueRatio = m_VehicleParams.axles[ i ].torqueFactor;

		pController->mDifferentials.push_back( differential );
	}

	return pController;
}

JPH::WheeledVehicleController *JoltPhysicsVehicleController::GetWheeledVehicleController()
{
	return static_cast<JPH::WheeledVehicleController *>( m_VehicleConstraint->GetController() );
}

//------------------------------------------------------------------------------------------------

matrix3x4_t JoltPhysicsVehicleController::GetBodyMatrix() const
{
	matrix3x4_t value;
	m_pCarBodyObject->GetPositionMatrix( &value );
	return value;
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	if ( m_pCarBodyObject == pObject )
		DetachObject();
}

void JoltPhysicsVehicleController::DetachObject()
{
	if ( m_pCarBodyObject )
	{
		m_pCarBodyObject->RemoveDestroyedListener( this );

		// Remove the listeners and constraint now, we can never
		// attach to another body.
		if ( m_VehicleConstraint != nullptr )
		{
			m_pPhysicsSystem->RemoveConstraint( m_VehicleConstraint );
			m_pPhysicsSystem->RemoveStepListener( m_VehicleConstraint );
		}

		m_pCarBodyObject = nullptr;
	}
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::ApplyImpulseAtPointMetric( const Vector &vImpulseMetric, const Vector &vWorldPosMetric )
{
	if ( !m_pCarBodyObject || !m_pCarBodyObject->GetBody() )
		return;

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.AddImpulse(
		m_pCarBodyObject->GetBodyID(),
		JPH::Vec3( vImpulseMetric.x, vImpulseMetric.y, vImpulseMetric.z ),
		JPH::Vec3( vWorldPosMetric.x, vWorldPosMetric.y, vWorldPosMetric.z ) );
}

void JoltPhysicsVehicleController::ApplyImpulseCenterMetric( const Vector &vImpulseMetric )
{
	if ( !m_pCarBodyObject || !m_pCarBodyObject->GetBody() )
		return;

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.AddImpulse(
		m_pCarBodyObject->GetBodyID(),
		JPH::Vec3( vImpulseMetric.x, vImpulseMetric.y, vImpulseMetric.z ) );
}

void JoltPhysicsVehicleController::ApplyAngularImpulseMetric( const Vector &vAngularImpulseMetric )
{
	if ( !m_pCarBodyObject || !m_pCarBodyObject->GetBody() )
		return;

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.AddAngularImpulse(
		m_pCarBodyObject->GetBodyID(),
		JPH::Vec3( vAngularImpulseMetric.x, vAngularImpulseMetric.y, vAngularImpulseMetric.z ) );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::InitAirboat( const vehicleparams_t &params )
{
	const float bodyMass = m_pCarBodyObject->GetMass();

	const int nAxles = Min( params.axleCount, 2 );
	const int nWheelsPerAxle = Min( params.wheelsPerAxle, 2 );
	m_nAirboatPontoons = Min( nAxles * nWheelsPerAxle, JOLT_AIRBOAT_MAX_PONTOONS );

	matrix3x4_t bodyMatrix = GetBodyMatrix();

	int idx = 0;
	for ( int axleIdx = 0; axleIdx < nAxles; ++axleIdx )
	{
		const vehicle_axleparams_t &axle = params.axles[ axleIdx ];

		for ( int wheelIdx = 0; wheelIdx < nWheelsPerAxle; ++wheelIdx, ++idx )
		{
			const Vector wheelOffsetSign = ( wheelIdx & 1 ) ? axle.wheelOffset : -axle.wheelOffset;
			const Vector traceOffsetSign = ( wheelIdx & 1 ) ? axle.raytraceOffset : -axle.raytraceOffset;

			const Vector wheelPositionLocal = axle.offset + wheelOffsetSign;
			const Vector tracePositionLocal = axle.raytraceCenterOffset + traceOffsetSign;

			JoltAirboatPontoon &pontoon = m_AirboatPontoons[ idx ];
			pontoon.hp_cs				= wheelPositionLocal;
			pontoon.raycast_start_cs	= tracePositionLocal;
			pontoon.raycast_dir_cs		= Vector( 0.0f, 0.0f, -1.0f );
			pontoon.raycast_length		= AIRBOAT_RAYCAST_DIST;
			pontoon.spring_constant		= axle.suspension.springConstant * bodyMass;
			pontoon.spring_damp_relax	= axle.suspension.springDamping * bodyMass;
			pontoon.spring_damp_compress = axle.suspension.springDampingCompression * bodyMass;
			pontoon.friction_of_wheel	= 1.0f;
			pontoon.wheel_radius		= SourceToJolt::Distance( axle.wheels.radius );
			pontoon.raycast_dist		= AIRBOAT_RAYCAST_DIST;
			pontoon.wheel_is_fixed		= true;
		}
	}

	m_pCarBodyObject->EnableGravity( false );
	m_pCarBodyObject->SetCallbackFlags( m_pCarBodyObject->GetCallbackFlags() & ~CALLBACK_DO_FLUID_SIMULATION );

	(void)bodyMatrix;
}

void JoltPhysicsVehicleController::AirboatUpdate( float dt, vehicle_controlparams_t &controls )
{
	JoltAirboatState &state = m_AirboatState;

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	if ( controls.steering != 0.0f || controls.throttle != 0.0f || fabsf( state.m_flThrust ) > 0.01f )
		bodyInterface.ActivateBody( m_pCarBodyObject->GetBodyID() );

	float flThrottle = controls.throttle;
	const float flAbsSpeed = fabsf( m_OperatingParams.speed );
	const float flMaxSpeed = Max( m_VehicleParams.engine.maxSpeed, 1.0f );
	if ( flThrottle > 0.0f && flAbsSpeed > flMaxSpeed )
	{
		const float flFrac = flAbsSpeed / flMaxSpeed;
		if ( flFrac > m_VehicleParams.engine.autobrakeSpeedGain )
			flThrottle = 0.0f;
		flThrottle *= 0.1f;
	}
	if ( flThrottle < 0.0f && flAbsSpeed > m_VehicleParams.engine.maxRevSpeed )
		flThrottle *= 0.1f;

	if ( fabsf( flThrottle ) < 0.01f )
		state.m_flThrust = 0.0f;
	else if ( flThrottle > 0.0f )
		state.m_flThrust = AIRBOAT_THRUST_MAX * flThrottle;
	else
		state.m_flThrust = AIRBOAT_THRUST_MAX_REVERSE * flThrottle;

	state.m_bAnalogSteering	= controls.bAnalogSteering;
	state.m_SteeringAngle	= DEG2RAD( controls.steering * Max( m_VehicleParams.steering.degreesSlow, m_VehicleParams.steering.degreesFast ) );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::AirboatOnPreSimulate( float flDeltaTime )
{
	if ( flDeltaTime <= 0.0f )
		return;

	{
		const float bodyMass = m_pCarBodyObject->GetMass();
		Vector vGravityImpulse( 0.0f, 0.0f, -AIRBOAT_GRAVITY * bodyMass * flDeltaTime );
		ApplyImpulseCenterMetric( vGravityImpulse );
	}

	matrix3x4_t matWorldFromCore = GetBodyMatrix();

	JPH::Vec3 jLinearMetric = m_pCarBodyObject->GetBody()->GetLinearVelocity();
	Vector vLinearMetric( jLinearMetric.GetX(), jLinearMetric.GetY(), jLinearMetric.GetZ() );
	m_AirboatState.m_flSpeed = vLinearMetric.Length();
	m_pCarBodyObject->WorldToLocalVector( &m_AirboatState.m_vecLocalVelocity, vLinearMetric );

	JoltAirboatImpact *pImpacts = m_AirboatImpacts;
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
		pImpacts[i] = JoltAirboatImpact{};

	AirboatPreRaycasts( pImpacts, matWorldFromCore );
	AirboatDoRaycasts( pImpacts );
	if ( !AirboatPostRaycasts( pImpacts, matWorldFromCore ) )
		return;

	AirboatUpdateAirborne( pImpacts, flDeltaTime );

	AirboatDoPontoons( pImpacts, flDeltaTime );
	AirboatDoDrag( pImpacts, flDeltaTime, matWorldFromCore );
	AirboatDoTurbine( flDeltaTime, matWorldFromCore );
	AirboatDoSteering( flDeltaTime );
	AirboatDoKeepUprightPitch( pImpacts, flDeltaTime, matWorldFromCore );
	AirboatDoKeepUprightRoll( pImpacts, flDeltaTime, matWorldFromCore );
}

//------------------------------------------------------------------------------------------------

float JoltPhysicsVehicleController::AirboatComputeFrontPontoonWaveNoise( int nIndex, float flSpeedRatio, float flCurrentTime )
{
	const float flNoiseScale = RemapValClamped( 1.0f - flSpeedRatio, 0.0f, 1.0f, AIRBOAT_WATER_NOISE_MIN, AIRBOAT_WATER_NOISE_MAX );

	float flPhaseShift = 0.0f;
	if ( flSpeedRatio < 0.3f )
		flPhaseShift = float( nIndex ) * AIRBOAT_WATER_PHASE_MAX;

	const float flFrequency = RemapValClamped( flSpeedRatio, 0.0f, 1.0f, AIRBOAT_WATER_FREQ_MIN, AIRBOAT_WATER_FREQ_MAX );
	return flNoiseScale * sinf( flFrequency * ( flCurrentTime + flPhaseShift ) );
}

void JoltPhysicsVehicleController::AirboatPreRaycasts( JoltAirboatImpact *pImpacts, const matrix3x4_t &matWorldFromCore )
{
	(void)matWorldFromCore;
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		JoltAirboatPontoon &pontoon = m_AirboatPontoons[ i ];
		pontoon.raycast_length = AIRBOAT_RAYCAST_DIST;
	}
}

void JoltPhysicsVehicleController::AirboatDoRaycasts( JoltAirboatImpact *pImpacts )
{
	if ( !m_pGameTrace )
		return;

	IPhysicsObject *pPhysAirboat = static_cast< IPhysicsObject * >( m_pCarBodyObject );

	const float flForwardSpeedRatioRaw = clamp( m_AirboatState.m_vecLocalVelocity.y / 10.0f, 0.0f, 1.0f );
	const float flSpeedRatio           = clamp( m_AirboatState.m_flSpeed / 15.0f, 0.0f, 1.0f );
	const float flForwardSpeedRatio    = m_AirboatState.m_flThrust ? flForwardSpeedRatioRaw : flForwardSpeedRatioRaw * 0.5f;
	const float flCurrentTime          = float( Plat_FloatTime() );

	matrix3x4_t matWorldFromCore = GetBodyMatrix();

	struct PontoonRayInfo
	{
		Vector start;
		Vector dirWorldSrc;
		float lengthMetric;
	};

	PontoonRayInfo rayInfo[ JOLT_AIRBOAT_MAX_PONTOONS ];

	int nFrontPontoonsInWater = 0;
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		JoltAirboatPontoon &pontoon = m_AirboatPontoons[ i ];

		Vector startWorldSrc;
		VectorTransform( pontoon.raycast_start_cs, matWorldFromCore, startWorldSrc );

		Vector dirWorldSrc;
		VectorRotate( pontoon.raycast_dir_cs, matWorldFromCore, dirWorldSrc );

		pImpacts[i].raycast_dir_ws = dirWorldSrc;
		pImpacts[i].bInWater = m_pGameTrace->VehiclePointInWater( startWorldSrc );
		if ( pImpacts[i].bInWater )
			dirWorldSrc.Negate();

		float lengthMetric = pontoon.raycast_length;
		Vector endWorldSrc = startWorldSrc + dirWorldSrc * ( lengthMetric * JoltToSource::Factor );

		if ( m_pGameTrace->VehiclePointInWater( endWorldSrc ) )
		{
			lengthMetric = AIRBOAT_RAYCAST_DIST_WATER_LOW;
			if ( i < 2 )
			{
				++nFrontPontoonsInWater;
				lengthMetric += AirboatComputeFrontPontoonWaveNoise( i, flSpeedRatio, flCurrentTime );
			}
			endWorldSrc = startWorldSrc + dirWorldSrc * ( lengthMetric * JoltToSource::Factor );
		}

		rayInfo[i].start = startWorldSrc;
		rayInfo[i].dirWorldSrc = dirWorldSrc;
		rayInfo[i].lengthMetric = lengthMetric;
	}

	if ( nFrontPontoonsInWater == 2 )
	{
		for ( int i = 0; i < 2; ++i )
		{
			float lengthMetric = RemapValClamped( flForwardSpeedRatio, 0.0f, 1.0f, AIRBOAT_RAYCAST_DIST_WATER_LOW, AIRBOAT_RAYCAST_DIST_WATER_HIGH );
			lengthMetric += AirboatComputeFrontPontoonWaveNoise( i, flSpeedRatio, flCurrentTime );
			rayInfo[i].lengthMetric = lengthMetric;
		}
	}

	trace_t trace;
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		JoltAirboatPontoon &pontoon = m_AirboatPontoons[ i ];
		JoltAirboatImpact &impact = pImpacts[ i ];

		pontoon.raycast_length = rayInfo[i].lengthMetric;

		Vector endWorldSrc = rayInfo[i].start + rayInfo[i].dirWorldSrc * ( rayInfo[i].lengthMetric * JoltToSource::Factor );

		Ray_t ray;
		ray.Init( rayInfo[i].start, endWorldSrc );

		if ( impact.bInWater )
		{
			m_pGameTrace->VehicleTraceRay( ray, pPhysAirboat->GetGameData(), &trace );

			Ray_t waterRay;
			Vector vecUp = rayInfo[i].start;
			vecUp.z += 1000.0f;
			waterRay.Init( rayInfo[i].start, vecUp );

			trace_t waterTrace;
			m_pGameTrace->VehicleTraceRayWithWater( waterRay, pPhysAirboat->GetGameData(), &waterTrace );
			impact.flDepth = 1000.0f * waterTrace.fractionleftsolid;

			if ( vjolt_airboat_debug.GetBool() )
				Log_Msg( LOG_VJolt, "Airboat[%d] inWater=1 startSolid=%d depthIn=%.2f upHit=%d\n",
					i, int( waterTrace.startsolid ), impact.flDepth, int( waterTrace.fraction != 1.0f ) );
		}
		else
		{
			m_pGameTrace->VehicleTraceRayWithWater( ray, pPhysAirboat->GetGameData(), &trace );

			if ( vjolt_airboat_debug.GetBool() )
				Log_Msg( LOG_VJolt, "Airboat[%d] inWater=0 hit=%d frac=%.3f isWater=%d startZ=%.1f\n",
					i, int( trace.fraction != 1.0f ), trace.fraction,
					int( ( trace.contents & MASK_WATER ) != 0 ), rayInfo[i].start.z );
		}

		impact.bImpact = false;
		impact.bImpactWater = false;

		if ( trace.fraction != 1.0f )
		{
			impact.bImpact = true;
			impact.flDepth = 0.0f;
			if ( trace.contents & MASK_WATER )
				impact.bImpactWater = true;

			impact.vecImpactPointWS = trace.endpos;
			impact.vecImpactNormalWS = trace.plane.normal;

			impact.nSurfaceProps = trace.surface.surfaceProps;

			const surfacedata_t *pSurface = JoltPhysicsSurfaceProps::GetInstance().GetSurfaceData( trace.surface.surfaceProps );
			if ( pSurface )
			{
				impact.flDampening = pSurface->physics.dampening;
				impact.flFriction = pSurface->physics.friction;
			}
		}
	}
}

bool JoltPhysicsVehicleController::AirboatPostRaycasts( JoltAirboatImpact *pImpacts, const matrix3x4_t &matWorldFromCore )
{
	bool bReturn = true;

	matrix3x4_t bodyMatrix = matWorldFromCore;

	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		JoltAirboatPontoon &pontoon = m_AirboatPontoons[ i ];
		JoltAirboatImpact &impact = pImpacts[ i ];

		if ( impact.bInWater )
			impact.raycast_dir_ws.Negate();

		Vector startWorldSrc;
		VectorTransform( pontoon.raycast_start_cs, bodyMatrix, startWorldSrc );

		if ( impact.bImpact )
		{
			Vector deltaSrc = impact.vecImpactPointWS - startWorldSrc;
			pontoon.raycast_dist = deltaSrc.Length() * SourceToJolt::Factor;

			float dotND = fabsf( impact.raycast_dir_ws.Dot( impact.vecImpactNormalWS ) );
			impact.inv_normal_dot_dir = 1.1f / ( dotND + 0.1f );
			impact.friction_value = impact.flFriction * pontoon.friction_of_wheel;
		}
		else
		{
			pontoon.raycast_dist = pontoon.raycast_length;
			impact.inv_normal_dot_dir = 1.0f;
			impact.vecImpactNormalWS = -impact.raycast_dir_ws;
			impact.friction_value = 1.0f;
			impact.vecImpactPointWS = startWorldSrc + impact.raycast_dir_ws * ( pontoon.raycast_dist * JoltToSource::Factor );
		}

		Vector surfaceSpeedSrc;
		m_pCarBodyObject->GetVelocityAtPoint( impact.vecImpactPointWS, &surfaceSpeedSrc );
		impact.surface_speed_wheel_ws = surfaceSpeedSrc * SourceToJolt::Factor;

		Vector n = impact.vecImpactNormalWS;
		float ns = n.Dot( impact.surface_speed_wheel_ws );
		impact.projected_surface_speed_wheel_ws = impact.surface_speed_wheel_ws - n * ns;
	}

	return bReturn;
}

//------------------------------------------------------------------------------------------------

int JoltPhysicsVehicleController::AirboatCountSurfaceContacts( JoltAirboatImpact *pImpacts )
{
	int nContacts = 0;
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		if ( pImpacts[i].bImpact )
			++nContacts;
	}
	return nContacts;
}

void JoltPhysicsVehicleController::AirboatUpdateAirborne( JoltAirboatImpact *pImpacts, float flDeltaTime )
{
	JoltAirboatState &state = m_AirboatState;
	int nCount = AirboatCountSurfaceContacts( pImpacts );
	if ( !nCount )
	{
		if ( !state.m_bAirborne )
		{
			state.m_bAirborne = true;
			state.m_flAirTime = 0;
			if ( state.m_flSpeed < 11.0f )
				state.m_bWeakJump = true;
		}
		else
		{
			state.m_flAirTime += flDeltaTime;
		}
	}
	else
	{
		state.m_bAirborne = false;
		state.m_bWeakJump = false;
	}
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::AirboatDoPontoons( JoltAirboatImpact *pImpacts, float flDeltaTime )
{
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		JoltAirboatPontoon &pontoon = m_AirboatPontoons[ i ];
		JoltAirboatImpact &impact = pImpacts[ i ];

		if ( impact.bImpact )
			AirboatDoPontoonGround( &pontoon, &impact, flDeltaTime );
		else if ( impact.bInWater )
			AirboatDoPontoonWater( &pontoon, &impact, flDeltaTime );
	}
}

void JoltPhysicsVehicleController::AirboatDoPontoonGround( JoltAirboatPontoon *pPontoon, JoltAirboatImpact *pImpact, float flDeltaTime )
{
	const float flDiff = pPontoon->raycast_dist - pPontoon->raycast_length;
	if ( flDiff >= 0.0f )
		return;

	float flForce = -flDiff * pPontoon->spring_constant;
	const float flInvNormalDotDir = clamp( pImpact->inv_normal_dot_dir, 0.0f, 3.0f );
	flForce *= flInvNormalDotDir;

	Vector vecSpeedDelta = pImpact->projected_surface_speed_wheel_ws - pImpact->surface_speed_wheel_ws;
	const float flSpeed = vecSpeedDelta.Dot( pImpact->raycast_dir_ws );
	if ( flSpeed > 0.0f )
		flForce -= pPontoon->spring_damp_relax * flSpeed;
	else
		flForce -= pPontoon->spring_damp_compress * flSpeed;

	if ( flForce < 0.0f )
		flForce = 0.0f;

	const float flImpulse = flForce * flDeltaTime;
	Vector vImpulseWS = pImpact->vecImpactNormalWS * flImpulse;

	ApplyImpulseAtPointMetric( vImpulseWS, pImpact->vecImpactPointWS * SourceToJolt::Factor );
}

void JoltPhysicsVehicleController::AirboatDoPontoonWater( JoltAirboatPontoon *pPontoon, JoltAirboatImpact *pImpact, float flDeltaTime )
{
	(void)pPontoon;

	const float flDepthMetric = clamp( pImpact->flDepth * SourceToJolt::Factor, 0.0f, AIRBOAT_PONTOON_HEIGHT );

	const float flSubmergedVolume = AIRBOAT_PONTOON_AREA_2D * flDepthMetric;
	const float bodyMass = m_pCarBodyObject->GetMass();
	const float flForce = AIRBOAT_BUOYANCY_SCALAR * 0.25f * bodyMass * flSubmergedVolume * 1000.0f;
	const float flImpulse = flForce * flDeltaTime;

	Vector vImpulseWS( 0.0f, 0.0f, flImpulse );
	ApplyImpulseAtPointMetric( vImpulseWS, pImpact->vecImpactPointWS * SourceToJolt::Factor );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::AirboatDoDrag( JoltAirboatImpact *pImpacts, float flDeltaTime, const matrix3x4_t &matWorldFromCore )
{
	const float flSpeed = m_AirboatState.m_flSpeed;

	int nPointsInWater = 0;
	int nPointsOnGround = 0;
	float flGroundFriction = 0.0f;
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		const JoltAirboatImpact &impact = pImpacts[i];
		if ( !impact.bImpact )
			continue;

		if ( impact.bImpactWater )
		{
			++nPointsInWater;
		}
		else
		{
			++nPointsOnGround;
			flGroundFriction += impact.flFriction;
		}
	}

	const float bodyMass = m_pCarBodyObject->GetMass();

	if ( nPointsInWater )
	{
		Vector negDirLS = -m_AirboatState.m_vecLocalVelocity;

		Vector vDragLS(
			AIRBOAT_WATER_DRAG_LEFT_RIGHT  * negDirLS.x,
			AIRBOAT_WATER_DRAG_FORWARD_BACK * negDirLS.y,
			AIRBOAT_WATER_DRAG_UP_DOWN     * negDirLS.z );

		vDragLS *= flSpeed * bodyMass * flDeltaTime;

		Vector vDragWS;
		VectorRotate( vDragLS, matWorldFromCore, vDragWS );
		ApplyImpulseCenterMetric( vDragWS );
	}

	if ( nPointsOnGround && flSpeed > 0.0f )
	{
		flGroundFriction /= float( nPointsOnGround );

		float flFrictionDrag = bodyMass * AIRBOAT_GRAVITY * AIRBOAT_DRY_FRICTION_SCALE * flGroundFriction;
		flFrictionDrag /= flSpeed;

		Vector negDirLS = -m_AirboatState.m_vecLocalVelocity;
		Vector vDragLS(
			AIRBOAT_GROUND_DRAG_LEFT_RIGHT  * negDirLS.x,
			AIRBOAT_GROUND_DRAG_FORWARD_BACK * negDirLS.y,
			AIRBOAT_GROUND_DRAG_UP_DOWN     * negDirLS.z );

		vDragLS *= flFrictionDrag * flDeltaTime;

		Vector vDragWS;
		VectorRotate( vDragLS, matWorldFromCore, vDragWS );
		ApplyImpulseCenterMetric( vDragWS );
	}
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::AirboatDoTurbine( float flDeltaTime, const matrix3x4_t &matWorldFromCore )
{
	float flThrust = m_AirboatState.m_flThrust;
	if ( m_AirboatState.m_bWeakJump || ( m_AirboatState.m_bAirborne && flThrust < 0.0f ) )
		flThrust *= 0.5f;

	Vector vForwardLocal( 0.0f, 1.0f, 0.0f );
	Vector vForwardWS;
	VectorRotate( vForwardLocal, matWorldFromCore, vForwardWS );

	if ( vForwardWS.z > 0.5f && flThrust > 0.0f )
	{
		float flFactor = 1.0f - vForwardWS.z;
		flThrust *= flFactor;
	}
	else if ( vForwardWS.z < -0.5f && flThrust < 0.0f )
	{
		float flFactor = 1.0f + vForwardWS.z;
		flThrust *= flFactor;
	}

	const float bodyMass = m_pCarBodyObject->GetMass();
	Vector vImpulse = vForwardWS * ( flThrust * bodyMass * flDeltaTime );
	ApplyImpulseCenterMetric( vImpulse );

	if ( vjolt_airboat_debug.GetBool() )
		Log_Msg( LOG_VJolt, "Airboat turbine: thrust=%.2f mass=%.1f |I|=%.2f fwdZ=%.2f\n", flThrust, bodyMass, vImpulse.Length(), vForwardWS.z );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::AirboatDoSteering( float flDeltaTime )
{
	JoltAirboatState &state = m_AirboatState;

	if ( state.m_SteeringAngle == 0.0f || state.m_flThrust != 0.0f )
	{
		if ( !state.m_bAnalogSteering )
		{
			if ( state.m_flThrust < 0.0f )
				state.m_bSteeringReversed = true;
			else if ( state.m_flThrust > 0.0f || state.m_vecLocalVelocity.y > 0.0f )
				state.m_bSteeringReversed = false;
		}
		else
		{
			if ( state.m_flThrust < -2.0f )
				state.m_bSteeringReversed = true;
			else if ( state.m_flThrust > 2.0f || state.m_vecLocalVelocity.y > 0.0f )
				state.m_bSteeringReversed = false;
		}
	}

	const float bodyMass = m_pCarBodyObject->GetMass();
	const float invDt = ( flDeltaTime > 0.0f ) ? ( 1.0f / flDeltaTime ) : 0.0f;

	float flForceSteering = 0.0f;
	if ( fabsf( state.m_SteeringAngle ) > 0.01f )
	{
		float flSteeringSign = state.m_SteeringAngle < 0.0f ? -1.0f : 1.0f;
		if ( state.m_bSteeringReversed )
			flSteeringSign *= -1.0f;

		float flPrevSign = state.m_flPrevSteeringAngle < 0.0f ? -1.0f : 1.0f;
		if ( fabsf( state.m_flPrevSteeringAngle ) < 0.01f || flSteeringSign != flPrevSign )
			state.m_flSteerTime = 0.0f;

		float flSteerScale = 0.0f;
		if ( !state.m_bAnalogSteering )
			flSteerScale = RemapValClamped( state.m_flSteerTime, 0.0f, AIRBOAT_STEERING_INTERVAL, AIRBOAT_STEERING_RATE_MIN, AIRBOAT_STEERING_RATE_MAX );
		else
			flSteerScale = RemapValClamped( fabsf( state.m_SteeringAngle ), 0.0f, AIRBOAT_STEERING_INTERVAL, AIRBOAT_STEERING_RATE_MIN, AIRBOAT_STEERING_RATE_MAX );

		flForceSteering = flSteerScale * bodyMass * invDt;
		flForceSteering *= -flSteeringSign;

		state.m_flSteerTime += flDeltaTime;
	}

	state.m_flPrevSteeringAngle = state.m_SteeringAngle * ( state.m_bSteeringReversed ? -1.0f : 1.0f );

	JPH::Vec3 jAngWorld = m_pCarBodyObject->GetBody()->GetAngularVelocity();
	Vector vAngWorld( jAngWorld.GetX(), jAngWorld.GetY(), jAngWorld.GetZ() );
	Vector vAngLocal;
	m_pCarBodyObject->WorldToLocalVector( &vAngLocal, vAngWorld );
	const float yawRate = -vAngLocal.z;
	const float yawSign = yawRate < 0.0f ? -1.0f : 1.0f;

	const float flRotDrag = AIRBOAT_ROT_DRAG * yawRate * yawRate * bodyMass * invDt * yawSign;
	const float flRotDamp = AIRBOAT_ROT_DAMPING * fabsf( yawRate ) * bodyMass * invDt * yawSign;

	const float flForceRotational = flForceSteering + flRotDrag + flRotDamp;

	Vector vAngImpLocal( 0.0f, 0.0f, flForceRotational );
	Vector vAngImpWorld;
	matrix3x4_t bodyMatrix = GetBodyMatrix();
	VectorRotate( vAngImpLocal, bodyMatrix, vAngImpWorld );
	ApplyAngularImpulseMetric( vAngImpWorld );

	if ( vjolt_airboat_debug.GetBool() )
		Log_Msg( LOG_VJolt, "Airboat steer: ang=%.3f rev=%d steerForce=%.2f rotDrag=%.2f rotDamp=%.2f\n",
			state.m_SteeringAngle, int( state.m_bSteeringReversed ), flForceSteering, flRotDrag, flRotDamp );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::AirboatDoKeepUprightPitch( JoltAirboatImpact *pImpacts, float flDeltaTime, const matrix3x4_t &matWorldFromCore )
{
	JoltAirboatState &state = m_AirboatState;
	if ( state.m_bWeakJump )
		return;

	if ( flDeltaTime <= 0.0f )
		return;

	const float kCos10 = cosf( DEG2RAD( 10.0f ) );
	const float kSin10 = sinf( DEG2RAD( 10.0f ) );
	Vector vUpCS( 0.0f, kSin10, kCos10 );

	Vector vGoalAxisWS( 0.0f, 0.0f, 1.0f );

	Vector vGoalAxisCS;
	VectorIRotate( vGoalAxisWS, matWorldFromCore, vGoalAxisCS );

	vGoalAxisCS.x = vUpCS.x;
	VectorNormalize( vGoalAxisCS );

	Vector vRotAxisCS = vUpCS.Cross( vGoalAxisCS );
	const float cosine = vUpCS.Dot( vGoalAxisCS );
	float sine = VectorNormalize( vRotAxisCS );
	const float angle = atan2f( sine, cosine );

	if ( AirboatCountSurfaceContacts( pImpacts ) > 0 )
	{
		state.m_flPitchErrorPrev = angle;
		return;
	}

	const float bodyMass = m_pCarBodyObject->GetMass();
	const float invDt = 1.0f / flDeltaTime;
	Vector vAngImpCS = vRotAxisCS * ( bodyMass * ( 0.1f * angle + 0.04f * invDt * ( angle - state.m_flPitchErrorPrev ) ) );
	state.m_flPitchErrorPrev = angle;

	float len = VectorNormalize( vAngImpCS );
	const float maxLen = DEG2RAD( 1.5f ) * bodyMass;
	if ( len > maxLen )
		len = maxLen;
	vAngImpCS *= len;

	Vector vAngImpWS;
	VectorRotate( vAngImpCS, matWorldFromCore, vAngImpWS );
	ApplyAngularImpulseMetric( vAngImpWS );
}

void JoltPhysicsVehicleController::AirboatDoKeepUprightRoll( JoltAirboatImpact *pImpacts, float flDeltaTime, const matrix3x4_t &matWorldFromCore )
{
	JoltAirboatState &state = m_AirboatState;
	if ( flDeltaTime <= 0.0f )
		return;

	const float kCos10 = cosf( DEG2RAD( 10.0f ) );
	const float kSin10 = sinf( DEG2RAD( 10.0f ) );
	Vector vUpCS( 0.0f, kSin10, kCos10 );

	Vector vGoalAxisWS( 0.0f, 0.0f, 1.0f );

	Vector vGoalAxisCS;
	VectorIRotate( vGoalAxisWS, matWorldFromCore, vGoalAxisCS );

	vGoalAxisCS.z = vUpCS.z;
	VectorNormalize( vGoalAxisCS );

	Vector vRotAxisCS = vUpCS.Cross( vGoalAxisCS );
	const float cosine = vUpCS.Dot( vGoalAxisCS );
	float sine = VectorNormalize( vRotAxisCS );
	const float angle = atan2f( sine, cosine );

	if ( AirboatCountSurfaceContacts( pImpacts ) > 0 )
	{
		state.m_flRollErrorPrev = angle;
		return;
	}

	if ( fabsf( angle ) < DEG2RAD( 10.0f ) )
	{
		state.m_flRollErrorPrev = angle;
		return;
	}

	const float bodyMass = m_pCarBodyObject->GetMass();
	const float invDt = 1.0f / flDeltaTime;
	Vector vAngImpCS = vRotAxisCS * ( bodyMass * ( 0.2f * angle + 0.3f * invDt * ( angle - state.m_flRollErrorPrev ) ) );
	state.m_flRollErrorPrev = angle;

	float len = VectorNormalize( vAngImpCS );
	const float maxLen = DEG2RAD( 2.0f ) * bodyMass;
	if ( len > maxLen )
		len = maxLen;
	vAngImpCS *= len;

	Vector vAngImpWS;
	VectorRotate( vAngImpCS, matWorldFromCore, vAngImpWS );
	ApplyAngularImpulseMetric( vAngImpWS );
}
