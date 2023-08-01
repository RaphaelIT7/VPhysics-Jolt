#include "vphysics_interface.h"
#include <isaverestore.h>
#include <utlsymbol.h>
#include <UtlStringMap.h>
#include <KeyValues.h>
#include "branch_overrides.h"
#include "vjolt_surfaceprops.h"

class JoltPhysicsSurfaceProps_Bridge final : public IPhysicsSurfaceProps
{
public:
	JoltPhysicsSurfaceProps_Bridge();

	int					ParseSurfaceData(const char* pFilename, const char* pTextfile) override;
	int					SurfacePropCount(void) const override;

	int					GetSurfaceIndex(const char* pSurfacePropName) const override;
	void				GetPhysicsProperties(int surfaceDataIndex, float* density, float* thickness, float* friction, float* elasticity) const override;

	surfacedata_t* GetSurfaceData(int surfaceDataIndex) override;
	const char* GetString(unsigned short stringTableIndex) const override;

	const char* GetPropName(int surfaceDataIndex) const override;

	void				SetWorldMaterialIndexTable(int* pMapArray, int mapSize) override;

	void				GetPhysicsParameters(int surfaceDataIndex, surfacephysicsparams_t* pParamsOut) const override;

	ISaveRestoreOps* GetMaterialIndexDataOps() const override_portal2;

	// GMod-specific internal gubbins that was exposed in the public interface.
	void* GetIVPMaterial(int nIndex) override_gmod;
	int					GetIVPMaterialIndex(const void* pMaterial) const override_gmod;
	void* GetIVPManager(void) override_gmod;
	int					RemapIVPMaterialIndex(int nIndex) const override_gmod;
	const char* GetReservedMaterialName(int nMaterialIndex) const override_gmod;
public:
	static JoltPhysicsSurfaceProps_Bridge& GetInstance() { return s_PhysicsSurfaceProps; }

private:
	CSysModule* s_pActualPhysicsSurfacePropsModule;
	JoltPhysicsSurfaceProps* s_ActualPhysicsSurfaceProps;
	static JoltPhysicsSurfaceProps_Bridge s_PhysicsSurfaceProps;
};