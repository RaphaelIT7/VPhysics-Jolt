#include "tier0/basetypes.h"
#include "vjolt_utils.h"

#ifdef _WIN32
#include <intrin.h>
#else
#include <cpuid.h>
#endif

static void GetCPUID( int *pInfo, int func, int subfunc )
{
#ifdef _WIN32
	__cpuidex( pInfo, func, subfunc );
#else
	__cpuid_count( func, subfunc, pInfo[0], pInfo[1], pInfo[2], pInfo[3] );
#endif
}

static CPULevel_t GetCPULevel()
{
	int cpuInfo[4];
	CPULevel_t cpuLevel = CPU_HAS_SSE2;

	GetCPUID( cpuInfo, 0, 0 ); // Get the number of functions
	const int numFuncs = cpuInfo[0];
	if ( numFuncs >= 7 )
	{
		GetCPUID( cpuInfo, 7, 0 ); // Call function 7
		bool hasAVX2 = cpuInfo[1] & ( 1 << 5 ); // 5 is the AVX2 bit
		if ( hasAVX2 )
			cpuLevel = CPU_HAS_AVX2;
	}
	else
	{
		GetCPUID( cpuInfo, 1, 0 ); // Call function 1
		bool hasSSE42 = cpuInfo[2] & ( 1 << 20 ); // 20 is the SSE42 bit
		if ( hasSSE42 )
			cpuLevel = CPU_HAS_SSE42;
	}

	return cpuLevel;
}

static const char *GetModuleFromCPULevel( CPULevel_t level )
{
	switch ( level )
	{
		case CPU_HAS_AVX2:		return "vphysics_jolt_avx2" DLL_EXT_STRING;
		case CPU_HAS_SSE42:		return "vphysics_jolt_sse42" DLL_EXT_STRING;
		default:				return "vphysics_jolt_sse2" DLL_EXT_STRING;
	}
}