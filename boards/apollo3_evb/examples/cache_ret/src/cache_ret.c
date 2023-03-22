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

#include "flash_matrix.h"//m_data (rx) : ORIGIN = 0x10000, LENGTH = 32K --> start from page 8 @64KB

// Define
#define CACHE_SZIE (16*1024)

// Global Variables
uint32_t readbuffer[CACHE_SZIE>>2];

int check_flash_instr ( bool inverse,uint32_t inst, uint32_t page) 
{

	uint32_t flash_addr;
	uint32_t i;

	//MSG(("Checking Inst%d, Page%d\n",inst,page)); 
	flash_addr = (inst *64 *  AM_HAL_FLASH_PAGE_SIZE) + page * AM_HAL_FLASH_PAGE_SIZE;
	//vl_am_hal_itm_disable();
	//AM_BFW(CTIMER, STCFG, FREEZE, 1);
	//AM_BFW(CTIMER, STCFG, CLEAR, 1);
	//AM_BFW(CTIMER, STCFG, CLEAR, 0);
	for ( i = 0; i < (8 * 1024 / 4); i ++ ) 
	{
		readbuffer[i] = *(uint32_t*)(flash_addr + i*4);       
	} 
	//AM_BFW(CTIMER, STCFG, FREEZE, 0);
	//vl_PutCoreToSleep(VLC_CORESLEEP_DEEPSLEEP);
	am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
	for ( i = 0; i < (8 * 1024 / 4); i ++) 
	{
		readbuffer[i] = *(uint32_t*)(flash_addr + i*4);       
	}  
	//vl_am_hal_itm_enable(true);
	for ( i = 0; i < (8 * 1024 / 4); i ++ ) 
	{
		if (inverse)
		{
			if ( readbuffer[i] != ~(flash_addr+i*4) )
			{
				//err_flag++;
				//MSG(("Inverse Error, inst%d, page%d,addr %x, value%x\n",inst,page,flash_addr+(i*4),readbuffer[i]));
			}
		}
		else
		{
			if ( readbuffer[i] != (flash_addr+(i*4)) )
			{
				//err_flag++;
				//MSG(("Error, inst%d, page%d,addr %x, value%x\n",inst,page,flash_addr+(i*4),readbuffer[i]));
			}
		}
	}
	return 0;
}

// Override the default interrupt handler
void am_stimer_cmpr0_isr(void)
{
    //AM_BFW(CTIMER, STCFG, FREEZE, 1);
    //AM_REG(CTIMER, STMINTCLR) = 0xFFFFFFFF;
    //AM_BFW(CTIMER, STCFG, CLEAR, 1);
    //AM_BFW(CTIMER, STCFG, CLEAR, 0);
}


int
main(void)
{  

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
	// Initialize the printf interface for UART output.
	//
	am_bsp_uart_printf_enable();

	//
	// Print the banner.
	//
	am_util_stdio_terminal_clear();
	am_util_stdio_printf("Retain Cache in Deepsleep\n");

	//
	// To minimize power during the run, disable the UART.
	//
	am_bsp_uart_printf_disable();

	//
	// Configure the board for low power.
	//
	am_bsp_low_power_init();

	//
	// Turn OFF unneeded flash
	//
	if ( am_hal_pwrctrl_memory_enable(AM_HAL_PWRCTRL_MEM_FLASH_MIN) )
	{
		while(1);
	}

	// For optimal Deep Sleep current, configure cache to be powered-down in deepsleep:
	am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_CACHE);

	//
	// Power down SRAM, only 32K SRAM retained
	//
	am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_SRAM_MAX);
	am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_32K_DTCM);

	while (1)
	{
		for(uint32_t flash_inst=0; flash_inst<1; flash_inst++) {
			for(uint32_t page=8; page<10; page++) {
				check_flash_instr(false,flash_inst,page);
			}
		}

		for(uint32_t flash_inst=0; flash_inst<1; flash_inst++) {
			for(uint32_t page=10; page<12; page++) {
				check_flash_instr(true,flash_inst,page);
			}
		}
	}
}



