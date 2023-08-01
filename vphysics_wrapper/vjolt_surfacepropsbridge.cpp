#include "vjolt_surfacepropsbridge.h"
#include "vjolt_utils.h"

JoltPhysicsSurfaceProps_Bridge::JoltPhysicsSurfaceProps_Bridge()
{
	const char* pModuleName = GetModuleFromCPULevel(GetCPULevel());

	if (!Sys_LoadInterface(pModuleName, VPHYSICS_SURFACEPROPS_INTERFACE_VERSION, &s_pActualPhysicsSurfacePropsModule, (void**)&s_ActualPhysicsSurfaceProps))
		return;
}

int JoltPhysicsSurfaceProps_Bridge::ParseSurfaceData(const char* pFilename, const char* pTextfile)
{
	return s_ActualPhysicsSurfaceProps->ParseSurfaceData(pFilename, pTextfile);
}

int JoltPhysicsSurfaceProps_Bridge::SurfacePropCount(void) const
{
	return s_ActualPhysicsSurfaceProps->SurfacePropCount();
}

int JoltPhysicsSurfaceProps_Bridge::GetSurfaceIndex(const char* pSurfacePropName) const
{
	return s_ActualPhysicsSurfaceProps->GetSurfaceIndex(pSurfacePropName);
}

void JoltPhysicsSurfaceProps_Bridge::GetPhysicsProperties(int surfaceDataIndex, float* density, float* thickness, float* friction, float* elasticity) const
{
	s_ActualPhysicsSurfaceProps->GetPhysicsProperties(surfaceDataIndex, density, thickness, friction, elasticity);
}

surfacedata_t* JoltPhysicsSurfaceProps_Bridge::GetSurfaceData(int surfaceDataIndex)
{
	return s_ActualPhysicsSurfaceProps->GetSurfaceData(surfaceDataIndex);
}

const char* JoltPhysicsSurfaceProps_Bridge::GetString(unsigned short stringTableIndex) const
{
	return s_ActualPhysicsSurfaceProps->GetString(stringTableIndex);
}

const char* JoltPhysicsSurfaceProps_Bridge::GetPropName(int surfaceDataIndex) const
{
	return s_ActualPhysicsSurfaceProps->GetPropName(surfaceDataIndex);
}

void JoltPhysicsSurfaceProps_Bridge::SetWorldMaterialIndexTable(int* pMapArray, int mapSize)
{
	return s_ActualPhysicsSurfaceProps->SetWorldMaterialIndexTable(pMapArray, mapSize);
}

void JoltPhysicsSurfaceProps_Bridge::GetPhysicsParameters(int surfaceDataIndex, surfacephysicsparams_t* pParamsOut) const
{
	return s_ActualPhysicsSurfaceProps->GetPhysicsParameters(surfaceDataIndex, pParamsOut);
}

ISaveRestoreOps* JoltPhysicsSurfaceProps_Bridge::GetMaterialIndexDataOps() const
{
	return s_ActualPhysicsSurfaceProps->GetMaterialIndexDataOps();
}

void* JoltPhysicsSurfaceProps_Bridge::GetIVPMaterial(int nIndex)
{
	return s_ActualPhysicsSurfaceProps->GetIVPMaterial(nIndex);
}

int JoltPhysicsSurfaceProps_Bridge::GetIVPMaterialIndex(const void* pMaterial) const
{
	return s_ActualPhysicsSurfaceProps->GetIVPMaterialIndex(pMaterial);
}

void* JoltPhysicsSurfaceProps_Bridge::GetIVPManager()
{
	return s_ActualPhysicsSurfaceProps->GetIVPManager();
}

int JoltPhysicsSurfaceProps_Bridge::RemapIVPMaterialIndex(int nIndex) const
{
	return s_ActualPhysicsSurfaceProps->RemapIVPMaterialIndex(nIndex);
}

const char* JoltPhysicsSurfaceProps_Bridge::GetReservedMaterialName(int nMaterialIndex) const
{
	return s_ActualPhysicsSurfaceProps->GetReservedMaterialName(nMaterialIndex);
}