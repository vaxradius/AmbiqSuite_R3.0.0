//*****************************************************************************
//
//! @file deepsleep.c
//!
//! @brief Example demonstrating how to enter deepsleep.
//!
//! Purpose: This example configures the device to go into a deep sleep mode. Once in
//! sleep mode the device has no ability to wake up. This example is merely to
//! provide the opportunity to measure deepsleep current without interrupts
//! interfering with the measurement.
//!
//! The example begins by printing out a banner announcement message through
//! the UART, which is then completely disabled for the remainder of execution.
//!
//! Text is output to the UART at 115,200 BAUD, 8 bit, no parity.
//! Please note that text end-of-line is a newline (LF) character only.
//! Therefore, the UART terminal must be set to simulate a CR/LF.
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

#include <string.h>

// Define
#define DEBUG_GPIO			41
#define SRAM_START_ADDR	0x10010000
#define SRAM_SIZE			0x50000


//**************************************
// Timer configuration.
//**************************************
am_hal_ctimer_config_t g_cTimer0 =
{
    // Don't link timers.
    0,

    // Set up Timer0A.
    (AM_HAL_CTIMER_FN_REPEAT    |
     AM_HAL_CTIMER_INT_ENABLE   |
     AM_HAL_CTIMER_XT_16_384KHZ), // 1/16384 = 61uS
    // No configuration for Timer0B.
    0,
};

uint32_t interval = 40; //start from 61uS * 40 

//*****************************************************************************
    //
// Timer Interrupt Service Routine (ISR)
    //
//*****************************************************************************
void
am_ctimer_isr(void)
{
	
	interval = (interval+1); //increase 61us 
	if(interval > 500)
		interval = 4;//start from 61uS * 4
	am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, interval,
	                     (interval >> 1));
    //
	// Clear TimerA0 Interrupt (write to clear).
    //
	am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);

} // am_ctimer_isr()



void 
ctimer_init(void)
{
	
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);

	//
	// Set up timer A0.
	//
	am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA);
	am_hal_ctimer_config(0, &g_cTimer0);

	am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, interval,
	                 (interval >> 1));

	//
	// Clear the timer Interrupt
	//
	am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);

	//
	// Enable the timer Interrupt.
	//
	am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);
	//NVIC_SetPriority(CTIMER_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);

	//
	// Enable the timer interrupt in the NVIC.
	//
	NVIC_EnableIRQ(CTIMER_IRQn);

}


int
main(void)
{  

	am_hal_gpio_state_write(DEBUG_GPIO, AM_HAL_GPIO_OUTPUT_SET);
	am_hal_gpio_pinconfig(DEBUG_GPIO, g_AM_HAL_GPIO_OUTPUT);
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
	// Configure the board for low power.
	//
	am_bsp_low_power_init();

	ctimer_init();

	//
	// Enable interrupts to the core.
	//
	am_hal_interrupt_master_enable();

	//print_test_patterns();

	//Test SRAM retention with pattern "0xA5"
	//384K SRAM retained
	am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_384K);
	am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_CACHE);

	while (1)
	{
		memset((void *)(SRAM_START_ADDR),0xA5,SRAM_SIZE);
		am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);
		am_hal_gpio_state_write(DEBUG_GPIO, AM_HAL_GPIO_OUTPUT_CLEAR);
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
		//am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
		am_hal_gpio_state_write(DEBUG_GPIO, AM_HAL_GPIO_OUTPUT_SET);
		am_hal_ctimer_stop(0, AM_HAL_CTIMER_TIMERA);

		for(int i = SRAM_START_ADDR; i < (SRAM_START_ADDR+SRAM_SIZE); )
		{
			if(*((uint32_t *)i) != 0xA5A5A5A5)
			{
				am_bsp_itm_printf_enable();
				am_util_stdio_terminal_clear();
				am_util_stdio_printf(" [%x] %x != 0xA5A5A5A5 !=\n",i , *((uint32_t *)i));
				while(1);
			}

			i+=4;
		}

	}

}
