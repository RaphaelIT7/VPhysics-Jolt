
#pragma once

#include "vjolt_object.h" // IJoltObjectDestroyedListener
#include "vjolt_environment.h" // IJoltPhysicsController

struct JoltPhysicsWheel
{
	JoltPhysicsObject* pObject = nullptr;
	bool InWater = false;
	float Depth = 0.0f;
};

struct JoltPhysicsInternalVehicleState
{
	bool  EngineDisabled = false;
	float BoostDelay = 0.0f;
	float BoosterRemainingTime = 0.0f;
	float LargestWheelRadius = 0.0f;
};

static constexpr int JOLT_AIRBOAT_MAX_PONTOONS = 4;

struct JoltAirboatPontoon
{
	Vector	hp_cs;
	Vector	raycast_start_cs;
	Vector	raycast_dir_cs;
	float	raycast_length		= 0.0f;
	float	spring_constant		= 0.0f;
	float	spring_damp_relax	= 0.0f;
	float	spring_damp_compress = 0.0f;
	float	friction_of_wheel	= 1.0f;
	float	wheel_radius		= 0.0f;
	float	raycast_dist		= 0.0f;
	bool	wheel_is_fixed		= true;
};

struct JoltAirboatImpact
{
	bool	bImpact			= false;
	bool	bImpactWater	= false;
	bool	bInWater		= false;
	Vector	vecImpactPointWS;
	Vector	vecImpactNormalWS;
	Vector	raycast_dir_ws;
	Vector	surface_speed_wheel_ws;
	Vector	projected_surface_speed_wheel_ws;
	float	flDepth			= 0.0f;
	float	flFriction		= 1.0f;
	float	flDampening		= 0.0f;
	float	friction_value	= 1.0f;
	float	inv_normal_dot_dir = 1.0f;
	int		nSurfaceProps	= 0;
};

struct JoltAirboatState
{
	float	m_flThrust			= 0.0f;
	float	m_SteeringAngle		= 0.0f;
	bool	m_bAnalogSteering	= false;
	bool	m_bSteeringReversed	= false;
	float	m_flPrevSteeringAngle = 0.0f;
	float	m_flSteerTime		= 0.0f;
	bool	m_bAirborne			= false;
	float	m_flAirTime			= 0.0f;
	bool	m_bWeakJump			= false;
	float	m_flPitchErrorPrev	= 0.0f;
	float	m_flRollErrorPrev	= 0.0f;
	float	m_flSpeed			= 0.0f;
	Vector	m_vecLocalVelocity;
};

class JoltPhysicsVehicleController final : public IPhysicsVehicleController, public IJoltObjectDestroyedListener, public IJoltPhysicsController
{
public:
	static constexpr int MaxWheels = VEHICLE_MAX_WHEEL_COUNT;

	JoltPhysicsVehicleController( JoltPhysicsEnvironment *pEnvironment, JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace );
	~JoltPhysicsVehicleController() override;

	void				Update( float dt, vehicle_controlparams_t &controls ) override;
	const vehicle_operatingparams_t & GetOperatingParams() override;
	const vehicleparams_t & GetVehicleParams() override;
	vehicleparams_t &	GetVehicleParamsForChange() override;
	float				UpdateBooster( float dt ) override;
	int					GetWheelCount( void ) override;
	IPhysicsObject *	GetWheel( int index ) override;
	bool				GetWheelContactPoint( int index, Vector *pContactPoint, int *pSurfaceProps ) override;
	void				SetSpringLength( int wheelIndex, float length ) override;
	void				SetWheelFriction( int wheelIndex, float friction ) override;

	void				OnVehicleEnter( void ) override;
	void				OnVehicleExit( void ) override;

	void				SetEngineDisabled( bool bDisable ) override;
	bool				IsEngineDisabled( void ) override;

	void				GetCarSystemDebugData( vehicle_debugcarsystem_t &debugCarSystem ) override;
	void				VehicleDataReload() override;

public:
	// IJoltObjectDestroyedListener
	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) override;

	float GetSpeed();

	// IJoltPhysicsController
	void OnPreSimulate( float flDeltaTime ) override;
	void OnPostSimulate( float flDeltaTime ) override;

private:

	bool IsAirboat() const { return m_VehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST; }

	void HandleBoostKey();
	void HandleBoostDecay();

	void CreateWheel( JPH::VehicleConstraintSettings &vehicleSettings, matrix3x4_t &bodyMatrix, int axleIdx, int wheelIdx );
	void CreateWheels( JPH::VehicleConstraintSettings& vehicleSettings );

	JPH::WheeledVehicleControllerSettings *CreateVehicleController();
	JPH::WheeledVehicleController *GetWheeledVehicleController();

	matrix3x4_t GetBodyMatrix() const;

	void DetachObject();

	// Airboat
	void InitAirboat( const vehicleparams_t &params );
	void AirboatUpdate( float dt, vehicle_controlparams_t &controls );
	void AirboatOnPreSimulate( float flDeltaTime );
	void AirboatPreRaycasts( JoltAirboatImpact *pImpacts, const matrix3x4_t &matWorldFromCore );
	void AirboatDoRaycasts( JoltAirboatImpact *pImpacts );
	bool AirboatPostRaycasts( JoltAirboatImpact *pImpacts, const matrix3x4_t &matWorldFromCore );
	void AirboatUpdateAirborne( JoltAirboatImpact *pImpacts, float flDeltaTime );
	int  AirboatCountSurfaceContacts( JoltAirboatImpact *pImpacts );
	float AirboatComputeFrontPontoonWaveNoise( int nIndex, float flSpeedRatio, float flCurrentTime );
	void AirboatDoPontoons( JoltAirboatImpact *pImpacts, float flDeltaTime );
	void AirboatDoPontoonGround( JoltAirboatPontoon *pPontoon, JoltAirboatImpact *pImpact, float flDeltaTime );
	void AirboatDoPontoonWater( JoltAirboatPontoon *pPontoon, JoltAirboatImpact *pImpact, float flDeltaTime );
	void AirboatDoDrag( JoltAirboatImpact *pImpacts, float flDeltaTime, const matrix3x4_t &matWorldFromCore );
	void AirboatDoTurbine( float flDeltaTime, const matrix3x4_t &matWorldFromCore );
	void AirboatDoSteering( float flDeltaTime );
	void AirboatDoKeepUprightPitch( JoltAirboatImpact *pImpacts, float flDeltaTime, const matrix3x4_t &matWorldFromCore );
	void AirboatDoKeepUprightRoll( JoltAirboatImpact *pImpacts, float flDeltaTime, const matrix3x4_t &matWorldFromCore );

	void ApplyImpulseAtPointMetric( const Vector &vImpulseMetric, const Vector &vWorldPosMetric );
	void ApplyImpulseCenterMetric( const Vector &vImpulseMetric );
	void ApplyAngularImpulseMetric( const Vector &vAngularImpulseMetric );

	JoltPhysicsEnvironment					*m_pEnvironment = nullptr;
	JPH::PhysicsSystem						*m_pPhysicsSystem = nullptr;
	JoltPhysicsObject						*m_pCarBodyObject = nullptr;
	IPhysicsGameTrace						*m_pGameTrace = nullptr;
	vehicleparams_t							m_VehicleParams = {};
	unsigned int							m_VehicleType = 0u;

	vehicle_operatingparams_t				m_OperatingParams = {};
	vehicle_controlparams_t					m_ControlParams = {};

	std::vector< JoltPhysicsWheel >			m_Wheels;

	float									m_TotalWheelMass = 0.0f;
	JoltPhysicsInternalVehicleState			m_InternalState;

	JPH::Ref< JPH::VehicleConstraint >		m_VehicleConstraint;
	JPH::Ref< JPH::VehicleCollisionTester >	m_Tester;

	// Airboat
	int										m_nAirboatPontoons	= 0;
	JoltAirboatPontoon						m_AirboatPontoons[ JOLT_AIRBOAT_MAX_PONTOONS ];
	JoltAirboatImpact						m_AirboatImpacts[ JOLT_AIRBOAT_MAX_PONTOONS ];
	JoltAirboatState						m_AirboatState;
};
