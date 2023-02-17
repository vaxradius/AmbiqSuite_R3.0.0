//*****************************************************************************
//
//! @file led_task.c
//!
//! @brief Task to handle LED operation.
//!
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

//*****************************************************************************
//
// Global includes for this project.
//
//*****************************************************************************
#include "freertos_lowpower.h"

//*****************************************************************************
//
// LED task handle.
//
//*****************************************************************************
TaskHandle_t led_task_handle;
//*****************************************************************************
//
// Handle for LED-related events.
//
//*****************************************************************************
EventGroupHandle_t xLedEventHandle;


//**************************************
// Timer configuration.
//**************************************
am_hal_ctimer_config_t g_sTimer0 =
{
    // Don't link timers.
    0,

    // Set up Timer0A.
    (AM_HAL_CTIMER_FN_REPEAT    |
     AM_HAL_CTIMER_INT_ENABLE   |
     AM_HAL_CTIMER_XT_32_768KHZ),
    // No configuration for Timer0B.
    0,
};

//*****************************************************************************
//
// Interrupt handler for the Buttons
//
//*****************************************************************************
void
timer_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;
    //
    // Send an event to the main LED task
    //
    xHigherPriorityTaskWoken = pdFALSE;

    xResult = xEventGroupSetBitsFromISR(xLedEventHandle, (1 << 0),
                                        &xHigherPriorityTaskWoken);

    //
    // If the LED task is higher-priority than the context we're currently
    // running from, we should yield now and run the radio task.
    //
    if (xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

//*****************************************************************************
    //
// Timer Interrupt Service Routine (ISR)
    //
//*****************************************************************************
void
am_ctimer_isr(void)
{
	static uint32_t interval = 2;
	interval = (interval+1)%500;
	if(interval == 0)
		interval = 1;
	am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, interval,
	                     (interval >> 1));
    //
	// Clear TimerA0 Interrupt (write to clear).
    //
	am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);

	timer_handler();
} // am_ctimer_isr()

//*****************************************************************************
//
// Perform initial setup for the LED task.
//
//*****************************************************************************
void
LedTaskSetup(void)
{
	uint32_t ui32Period;

	//
	// Create an event handle for our wake-up events.
	//
	xLedEventHandle = xEventGroupCreate();

	//
	// Make sure we actually allocated space for the events we need.
	//
	while (xLedEventHandle == NULL);

	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);

	//
	// Set up timer A0.
	//
	am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA);
	am_hal_ctimer_config(0, &g_sTimer0);

	ui32Period = 1;
	am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, ui32Period,
	                 (ui32Period >> 1));

	//
	// Clear the timer Interrupt
	//
	am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);

	//
	// Enable the timer Interrupt.
	//
	am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);
	NVIC_SetPriority(CTIMER_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);

	//
	// Enable the timer interrupt in the NVIC.
	//
	NVIC_EnableIRQ(CTIMER_IRQn);

	//
	// Enable interrupts to the core.
	//
	am_hal_interrupt_master_enable();

	//
	// Start timer A0
	//
	am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);
}

uint32_t
prime_number(int32_t i32n)
{
    uint32_t ui32Total, ui32Prime;
    int32_t ix, jx;

    ui32Total = 0;

    for ( ix = 2; ix <= i32n; ix++ )
    {
        ui32Prime = 1;
        for ( jx = 2; jx < ix; jx++ )
        {
            if ( (ix % jx) == 0 )
            {
                ui32Prime = 0;
                break;
            }
        }
        ui32Total += ui32Prime;
    }

    return ui32Total;
}

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************
void
LedTask(void *pvParameters)
{
	uint32_t bitSet;
	volatile uint32_t u32_pNumber = 0;

	u32_pNumber = prime_number(500);
	am_hal_gpio_pinconfig(41, g_AM_HAL_GPIO_OUTPUT);

	while (1)
	{
		bitSet = xEventGroupWaitBits(xLedEventHandle, 0x7, pdTRUE,
									pdFALSE, portMAX_DELAY);
		if (bitSet != 0)
		{
			if (bitSet & (1 << 0))
			{
				am_hal_gpio_state_write(41, AM_HAL_GPIO_OUTPUT_SET);
				if(u32_pNumber != prime_number(500))
					while(1);
				am_hal_gpio_state_write(41, AM_HAL_GPIO_OUTPUT_CLEAR);
			}
		}
	}
}

