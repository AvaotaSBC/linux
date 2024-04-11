/*************************************************************************/ /*!
@File
@Title          System Description Header
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    This header provides system-specific declarations and macros
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#if !defined(__SYSCONFIG_H__)
#define __SYSCONFIG_H__

#include "pvrsrv_device.h"
#include "rgxdevice.h"

/* Valid values for TC_MEMORY_CONFIG configuration option */
#define TC_MEMORY_LOCAL			(1)
#define TC_MEMORY_HOST			(2)
#define TC_MEMORY_HYBRID		(3)

#if defined(SUPPORT_DISPLAY_CLASS)
/* Memory reserved for use by the PDP DC. */
#define RGX_TC_RESERVE_DC_MEM_SIZE	((TC_DISPLAY_MEM_SIZE) * 1024 * 1024)
#endif

#define SYS_RGX_ACTIVE_POWER_LATENCY_MS (10)

static PVRSRV_DEVICE_CONFIG gsDevices[];

static RGX_TIMING_INFORMATION gsRGXTimingInfo =
{
	/* ui32CoreClockSpeed */
	0,	/* Initialize to 0, real value will be set in PCIInitDev() */
	/* bEnableActivePM */
	IMG_TRUE,
	/* bEnableRDPowIsland */
	IMG_FALSE,
	/* ui32ActivePMLatencyms */
	SYS_RGX_ACTIVE_POWER_LATENCY_MS
};

static RGX_DATA gsRGXData =
{
	/* psRGXTimingInfo */
	&gsRGXTimingInfo
};

#if (TC_MEMORY_CONFIG == TC_MEMORY_LOCAL)
static void TCLocalCpuPAddrToDevPAddr(IMG_HANDLE hPrivData,
					  IMG_UINT32 ui32NumOfAddr,
					  IMG_DEV_PHYADDR *psDevPAddr,
					  IMG_CPU_PHYADDR *psCpuPAddr);

static void TCLocalDevPAddrToCpuPAddr(IMG_HANDLE hPrivData,
					  IMG_UINT32 ui32NumOfAddr,
					  IMG_CPU_PHYADDR *psCpuPAddr,
					  IMG_DEV_PHYADDR *psDevPAddr);

#elif (TC_MEMORY_CONFIG == TC_MEMORY_HOST)
static void TCSystemCpuPAddrToDevPAddr(IMG_HANDLE hPrivData,
					   IMG_UINT32 ui32NumOfAddr,
					   IMG_DEV_PHYADDR *psDevPAddr,
					   IMG_CPU_PHYADDR *psCpuPAddr);

static void TCSystemDevPAddrToCpuPAddr(IMG_HANDLE hPrivData,
					   IMG_UINT32 ui32NumOfAddr,
					   IMG_CPU_PHYADDR *psCpuPAddr,
					   IMG_DEV_PHYADDR *psDevPAddr);

#elif (TC_MEMORY_CONFIG == TC_MEMORY_HYBRID)
static void TCHybridCpuPAddrToDevPAddr(IMG_HANDLE hPrivData,
					  IMG_UINT32 ui32NumOfAddr,
					  IMG_DEV_PHYADDR *psDevPAddr,
					  IMG_CPU_PHYADDR *psCpuPAddr);

static void TCHybridDevPAddrToCpuPAddr(IMG_HANDLE hPrivData,
					  IMG_UINT32 ui32NumOfAddr,
					  IMG_CPU_PHYADDR *psCpuPAddr,
					  IMG_DEV_PHYADDR *psDevPAddr);

#endif /* (TC_MEMORY_CONFIG == TC_MEMORY_HYBRID) */

#if (TC_MEMORY_CONFIG == TC_MEMORY_LOCAL)
static PHYS_HEAP_FUNCTIONS gsLocalPhysHeapFuncs =
{
	/* pfnCpuPAddrToDevPAddr */
	TCLocalCpuPAddrToDevPAddr,
	/* pfnDevPAddrToCpuPAddr */
	TCLocalDevPAddrToCpuPAddr,
};

static PHYS_HEAP_CONFIG	gsPhysHeapConfig[] =
{
	{
		/* eType */
		PHYS_HEAP_TYPE_LMA,
		/* pszPDumpMemspaceName */
		"LMA",
		/* psMemFuncs */
		&gsLocalPhysHeapFuncs,
		/* sStartAddr */
		{0},
		/* sCardBase */
		{0},
		/* uiSize */
		0,
		/* hPrivData */
		(IMG_HANDLE)&gsDevices[0],
		/* ui32UsageFlags */
		PHYS_HEAP_USAGE_GPU_LOCAL,
	},
#if defined(SUPPORT_DISPLAY_CLASS)
	{
		/* eType */
		PHYS_HEAP_TYPE_LMA,
		/* pszPDumpMemspaceName */
		"LMA",
		/* psMemFuncs */
		&gsLocalPhysHeapFuncs,
		/* sStartAddr */
		{0},
		/* sCardBase */
		{0},
		/* uiSize */
		0,
		/* hPrivData */
		(IMG_HANDLE)&gsDevices[0],
		/* ui32UsageFlags */
		PHYS_HEAP_USAGE_DISPLAY,
	},
#endif
};
#elif (TC_MEMORY_CONFIG == TC_MEMORY_HOST)
static PHYS_HEAP_FUNCTIONS gsSystemPhysHeapFuncs =
{
	/* pfnCpuPAddrToDevPAddr */
	TCSystemCpuPAddrToDevPAddr,
	/* pfnDevPAddrToCpuPAddr */
	TCSystemDevPAddrToCpuPAddr,
};

static PHYS_HEAP_CONFIG	gsPhysHeapConfig[] =
{
	{
		/* eType */
		PHYS_HEAP_TYPE_UMA,
		/* pszPDumpMemspaceName */
		"SYSMEM",
		/* psMemFuncs */
		&gsSystemPhysHeapFuncs,
		/* sStartAddr */
		{0},
		/* sCardBase */
		{0},
		/* uiSize */
		0,
		/* hPrivData */
		(IMG_HANDLE)&gsDevices[0],
		/* ui32UsageFlags */
		PHYS_HEAP_USAGE_GPU_LOCAL,
	}
};
#elif (TC_MEMORY_CONFIG == TC_MEMORY_HYBRID)
static PHYS_HEAP_FUNCTIONS gsHybridPhysHeapFuncs =
{
	/* pfnCpuPAddrToDevPAddr */
	TCHybridCpuPAddrToDevPAddr,
	/* pfnDevPAddrToCpuPAddr */
	TCHybridDevPAddrToCpuPAddr,
};

static PHYS_HEAP_CONFIG	gsPhysHeapConfig[] =
{
	{
		/* eType */
		PHYS_HEAP_TYPE_LMA,
		/* pszPDumpMemspaceName */
		"LMA",
		/* psMemFuncs */
		&gsHybridPhysHeapFuncs,
		/* sStartAddr */
		{0},
		/* sCardBase */
		{0},
		/* uiSize */
		0,
		/* hPrivData */
		(IMG_HANDLE)&gsDevices[0],
		/* ui32UsageFlags */
		PHYS_HEAP_USAGE_GPU_LOCAL,
	},
#if defined(SUPPORT_DISPLAY_CLASS)
	{
		/* eType */
		PHYS_HEAP_TYPE_LMA,
		/* pszPDumpMemspaceName */
		"LMA",
		/* psMemFuncs */
		&gsHybridPhysHeapFuncs,
		/* sStartAddr */
		{0},
		/* sCardBase */
		{0},
		/* uiSize */
		0,
		/* hPrivData */
		(IMG_HANDLE)&gsDevices[0],
		/* ui32UsageFlags */
		PHYS_HEAP_USAGE_DISPLAY,
	},
#endif
	{
		/* eType */
		PHYS_HEAP_TYPE_UMA,
		/* pszPDumpMemspaceName */
		"SYSMEM",
		/* psMemFuncs */
		&gsHybridPhysHeapFuncs,
		/* sStartAddr */
		{0},
		/* sCardBase */
		{0},
		/* uiSize */
		0,
		/* hPrivData */
		(IMG_HANDLE)&gsDevices[0],
		/* ui32UsageFlags */
		PHYS_HEAP_USAGE_CPU_LOCAL,
	}
};
#else
#error "TC_MEMORY_CONFIG not valid"
#endif

static PVRSRV_DEVICE_CONFIG gsDevices[] =
{
	{
		.pvOSDevice		= NULL,
		.psDevNode		= NULL,
		.pszName		= "apollo",
		.pszVersion		= NULL,

		/* Device setup information */
		.sRegsCpuPBase		= { 0 },
		.ui32RegsSize		= 0,
		.ui32IRQ			= 0,

		.eCacheSnoopingMode	= PVRSRV_DEVICE_SNOOP_NONE,

		.hDevData			= &gsRGXData,
		.hSysData			= NULL,

		.bHasNonMappableLocalMemory	= IMG_FALSE,

		/* Physical memory heaps */
		.pasPhysHeaps		= &gsPhysHeapConfig[0],
		.ui32PhysHeapCount	= ARRAY_SIZE(gsPhysHeapConfig),

		.pfnPrePowerState	= NULL,
		.pfnPostPowerState	= NULL,
		.bHasFBCDCVersion31	= IMG_FALSE,

		/* Only required for LMA but having this always set shouldn't be a problem */
		.bDevicePA0IsValid	= IMG_TRUE,

		.pfnClockFreqGet	= NULL,

		.pfnCheckMemAllocSize = NULL,

#if defined(SUPPORT_TRUSTED_DEVICE)
		.pfnTDSendFWImage		= NULL,
		.pfnTDSetPowerParams	= NULL,
		.pfnTDRGXStart			= NULL,
		.pfnTDRGXStop			= NULL,
#endif
		.pfnSysDevFeatureDepInit	= NULL
	}
};

/*****************************************************************************
 * system specific data structures
 *****************************************************************************/

#endif /* !defined(__SYSCONFIG_H__) */
