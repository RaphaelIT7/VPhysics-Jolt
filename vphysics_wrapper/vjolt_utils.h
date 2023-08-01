#pragma once

enum CPULevel_t
{
	CPU_HAS_SSE2,
	CPU_HAS_SSE42,
	CPU_HAS_AVX2,
};

extern CPULevel_t GetCPULevel();
extern const char* GetModuleFromCPULevel( CPULevel_t level );