//*****************************************************************************
//
//! @file hello_world.c
//!
//! @brief A simple "Hello World" example.
//!
//! Purpose: This example prints a "Hello World" message with some device info.
//!
//! Printing takes place over the ITM at 1M Baud.
//!
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2021, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk_3_0_0-742e5ac27c of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

void dump_info0(void)
{
	uint32_t *idx= (uint32_t *)0x50020000;
	for(int i = 0; i < (8*1024)/4; i++)
	{
		if(i%4==0)
			am_util_stdio_printf("\n[0x%08X] ",i*4);
		am_util_stdio_printf("0x%08X ", *(idx+i));
	}


	// unlock info0 access w4 0x40030078 1             
	// unlock the info0 secured portion w4 0x40030080 DEADBEEF DEADBEEF DEADBEEF DEADBEEF   // write the unlock key		
	// this should be replaced with actual customer key!!!
	// mem32 0x4003007C 1          
	// dump the lock status for check

	*((uint32_t *)0x40030078) = 1;
	*((uint32_t *)0x40030080) = 0xDEADBEEF;
	*((uint32_t *)0x40030084) = 0xDEADBEEF;
	*((uint32_t *)0x40030088) = 0xDEADBEEF;
	*((uint32_t *)0x4003008C) = 0xDEADBEEF;

	am_util_stdio_printf("\nlock status : 0x%08X ", *((uint32_t *)0x4003007C));

	for(int i = 0; i < (8*1024)/4; i++)
	{
		if(i%4==0)
			am_util_stdio_printf("\n[0x%08X] ",i*4);
		am_util_stdio_printf("0x%08X ", *(idx+i));
	}

	//w4 0x40030080 FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFF   // lock the secured portion
	*((uint32_t *)0x40030080) = 0xFFFFFFFF;
	*((uint32_t *)0x40030084) = 0xFFFFFFFF;
	*((uint32_t *)0x40030088) = 0xFFFFFFFF;
	*((uint32_t *)0x4003008C) = 0xFFFFFFFF;

	am_util_stdio_printf("\nlock status : 0x%08X ", *((uint32_t *)0x4003007C));

	
}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
    am_util_id_t sIdDevice;
    uint32_t ui32StrBuf;
    //
    // Set the clock frequency.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Initialize the printf interface for ITM output
    //
    am_bsp_itm_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Hello World!\n\n");

    //
    // Print the device info.
    //
    am_util_id_device(&sIdDevice);
    am_util_stdio_printf("Vendor Name: %s\n", sIdDevice.pui8VendorName);
    am_util_stdio_printf("Device type: %s\n", sIdDevice.pui8DeviceName);


    am_util_stdio_printf("Qualified: %s\n",
                         sIdDevice.sMcuCtrlDevice.ui32Qualified ?
                         "Yes" : "No");

    am_util_stdio_printf("Device Info:\n"
                         "\tPart number: 0x%08X\n"
                         "\tChip ID0:    0x%08X\n"
                         "\tChip ID1:    0x%08X\n"
                         "\tRevision:    0x%08X (Rev%c%c)\n",
                         sIdDevice.sMcuCtrlDevice.ui32ChipPN,
                         sIdDevice.sMcuCtrlDevice.ui32ChipID0,
                         sIdDevice.sMcuCtrlDevice.ui32ChipID1,
                         sIdDevice.sMcuCtrlDevice.ui32ChipRev,
                         sIdDevice.ui8ChipRevMaj, sIdDevice.ui8ChipRevMin );

    //
    // If not a multiple of 1024 bytes, append a plus sign to the KB.
    //
    ui32StrBuf = ( sIdDevice.sMcuCtrlDevice.ui32FlashSize % 1024 ) ? '+' : 0;
    am_util_stdio_printf("\tFlash size:  %7d (%d KB%s)\n",
                         sIdDevice.sMcuCtrlDevice.ui32FlashSize,
                         sIdDevice.sMcuCtrlDevice.ui32FlashSize / 1024,
                         &ui32StrBuf);

    ui32StrBuf = ( sIdDevice.sMcuCtrlDevice.ui32SRAMSize % 1024 ) ? '+' : 0;
    am_util_stdio_printf("\tSRAM size:   %7d (%d KB%s)\n\n",
                         sIdDevice.sMcuCtrlDevice.ui32SRAMSize,
                         sIdDevice.sMcuCtrlDevice.ui32SRAMSize / 1024,
                         &ui32StrBuf);
    //
    // Print the compiler version.
    //
    am_util_stdio_printf("App Compiler:    %s\n", COMPILER_VERSION);
#if defined(AM_PART_APOLLO3)  || defined(AM_PART_APOLLO3P)
    am_util_stdio_printf("HAL Compiler:    %s\n", g_ui8HALcompiler);
    am_util_stdio_printf("HAL SDK version: %d.%d.%d\n",
                         g_ui32HALversion.s.Major,
                         g_ui32HALversion.s.Minor,
                         g_ui32HALversion.s.Revision);
    am_util_stdio_printf("HAL compiled with %s-style registers\n",
                         g_ui32HALversion.s.bAMREGS ? "AM_REG" : "CMSIS");

    am_hal_security_info_t secInfo;
    char sINFO[32];
    uint32_t ui32Status;
    ui32Status = am_hal_security_get_info(&secInfo);
    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        if ( secInfo.bInfo0Valid )
        {
            am_util_stdio_sprintf(sINFO, "INFO0 valid, ver 0x%X", secInfo.info0Version);
        }
        else
        {
            am_util_stdio_sprintf(sINFO, "INFO0 invalid");
        }

        am_util_stdio_printf("SBL ver: 0x%x - 0x%x, %s\n",
            secInfo.sblVersion, secInfo.sblVersionAddInfo, sINFO);
    }
    else
    {
        am_util_stdio_printf("am_hal_security_get_info failed 0x%X\n", ui32Status);
    }
#endif // AM_PART_APOLLO3

	dump_info0();

    //
    // We are done printing.
    // Disable debug printf messages on ITM.
    //
    am_bsp_debug_printf_disable();

    //
    // Loop forever while sleeping.
    //
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
