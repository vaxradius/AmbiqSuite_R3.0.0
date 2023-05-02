////////////////////////////////////////////////////////////////////////////////
//
// Test the Apollo3 Cache Retention
// mainly for GAR-140, 149 RMA
//
// Author: Sam & Jeremy 
//
////////////////////////////////////////////////////////////////////////////////

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "flash_matrix.h"
// uint32_t __attribute__((section (".myBufSection"))) flash_matrix[] --> 32KB
//m_data (rx) : ORIGIN = 0x10000, LENGTH = 32K --> start from page 8 @64KB

#include <string.h>

// Define
#define CACHE_SZIE (16*1024)
#define DEBUG_GPIO 41

// Global Variables
uint32_t readbuffer0[CACHE_SZIE>>2];
uint32_t readbuffer1[CACHE_SZIE>>2];

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

void dump_readbuffer(void)
{
	am_bsp_itm_printf_enable();
	am_util_stdio_terminal_clear();
	am_util_stdio_printf("dump_readbuffer\n");

	for(int i = 0;i < (CACHE_SZIE/4);i++)
	{
		if(readbuffer0[i] != readbuffer1[i])
			am_util_stdio_printf("[%0X8] %0X8 : %0X8\n",i ,  readbuffer0[i], readbuffer1[i]);
	}

	while(1);
}

void print_test_patterns(void)
{
	am_bsp_itm_printf_enable();
	am_util_stdio_terminal_clear();

	am_util_stdio_printf("const uint32_t flash_matrix1[] = {\n");

	for(int i = 1;i <= (CACHE_SZIE/4);i++)
	{
		am_util_stdio_printf("0xA5A5A5A5");
		if(i < (CACHE_SZIE/4))
			am_util_stdio_printf(",");
		if(i%8 == 0)
			am_util_stdio_printf("\n");
	}
	am_util_stdio_printf(" };\n");
	
	am_util_stdio_printf("const uint32_t flash_matrix2[] = {\n");

	for(int i = 1;i <= (CACHE_SZIE/4);i++)
	{
		am_util_stdio_printf("0x5A5A5A5A");
		if(i < (CACHE_SZIE/4))
			am_util_stdio_printf(",");
		if(i%8 == 0)
			am_util_stdio_printf("\n");
	}
	am_util_stdio_printf(" };\n");

	while(1);
}

#if 0
/*am_hal_cachectrl_defaults in /mcu/apollo3/hal/am_hal_cachectrl.c*/
const am_hal_cachectrl_config_t am_hal_cachectrl_defaults =
{
    .bLRU                       = 0,
    .eDescript                  = AM_HAL_CACHECTRL_DESCR_1WAY_128B_1024E,
    .eMode                      = AM_HAL_CACHECTRL_CONFIG_MODE_INSTR_DATA,
};
#else
const am_hal_cachectrl_config_t my_cachectrl =
{
    .bLRU                       = 0,
    .eDescript                  = AM_HAL_CACHECTRL_DESCR_2WAY_128B_512E,
    .eMode                      = AM_HAL_CACHECTRL_CONFIG_MODE_DATA,
};
#endif

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
	am_hal_cachectrl_config(&my_cachectrl);
	am_hal_cachectrl_enable();

	//
	// Configure the board for low power.
	//
	am_bsp_low_power_init();


	// For optimal Deep Sleep current, configure cache to be powered-down in deepsleep:
	//am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_CACHE);

	//
	// Power down SRAM, only 32K SRAM retained
	//
	//am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_SRAM_MAX);
	am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_384K);

	ctimer_init();

	//
	// Enable interrupts to the core.
	//
	am_hal_interrupt_master_enable();

	//print_test_patterns();

	//
	// Start timer A0
	//
	

	while (1)
	{  
		int ret = 0;
		/*0xA5A5A5A5*/
		memcpy((void *)readbuffer0, (void *)flash_matrix1,CACHE_SZIE);
		
		am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);
		am_hal_gpio_state_write(DEBUG_GPIO, AM_HAL_GPIO_OUTPUT_CLEAR);
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
		am_hal_gpio_state_write(DEBUG_GPIO, AM_HAL_GPIO_OUTPUT_SET);
		am_hal_ctimer_stop(0, AM_HAL_CTIMER_TIMERA);
		
		memcpy((void *)readbuffer1, (void *)flash_matrix1,CACHE_SZIE);
		ret = memcmp((const void *)readbuffer0, (const void *)readbuffer1, CACHE_SZIE);
		if(ret)
		{
			while(1);
		}

		/*0x5A5A5A5A*/
		memcpy((void *)readbuffer0, (void *)flash_matrix2,CACHE_SZIE);
		
		am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);
		am_hal_gpio_state_write(DEBUG_GPIO, AM_HAL_GPIO_OUTPUT_CLEAR);
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
		am_hal_gpio_state_write(DEBUG_GPIO, AM_HAL_GPIO_OUTPUT_SET);
		am_hal_ctimer_stop(0, AM_HAL_CTIMER_TIMERA);
		
		memcpy((void *)readbuffer1, (void *)flash_matrix2,CACHE_SZIE);
		ret = memcmp((const void *)readbuffer0, (const void *)readbuffer1, CACHE_SZIE);
		if(ret)
		{
			while(1);
		}
			
			
	}
}



