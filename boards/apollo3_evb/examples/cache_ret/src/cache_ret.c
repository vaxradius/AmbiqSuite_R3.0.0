////////////////////////////////////////////////////////////////////////////////
//
// Test the Apollo3 Cache Retention
// mainly for GAR-140, 149 RMA
//
// Author: Sam & Jeremy 
//
////////////////////////////////////////////////////////////////////////////////

#include "am_mcu_apollo.h"
#include "val_lib.h"
#include "flash_lib.h"
#include <stdlib.h>   // Prototype for exit()
#include "gpio_lib.h"
#include "power_lib.h"
#include "ctimer_lib.h"
#include "core_cm4.h"
#include "am_hal_flash.h"
#include "../bootloader/src/bl_flash_helpers_INTERNAL_ONLY.h"
#include "flash_matrix.h"

// Define
#define GPIO_PASS 16
#define GPIO_TEST_START 2
#define GPIO_HARDFAULT 3
#define CACHE_SZIE (16*1024)

// Global Variables
uint32_t err_flag = 0;
uint32_t itr_count = 0;
uint32_t program_buffer[FLASH_DATA_PAGE_SIZE>>2];  // 8K buffer for flash programming
uint32_t readbuffer[CACHE_SZIE>>2];

int check_flash_instr ( bool inverse,uint32_t inst, uint32_t page) {
    
    uint32_t flash_addr;
    uint32_t i;
      
    MSG(("Checking Inst%d, Page%d\n",inst,page)); 
    flash_addr = (inst *64 *  FLASH_DATA_PAGE_SIZE) + page * FLASH_DATA_PAGE_SIZE;
    vl_am_hal_itm_disable();
    AM_BFW(CTIMER, STCFG, FREEZE, 1);
    AM_BFW(CTIMER, STCFG, CLEAR, 1);
    AM_BFW(CTIMER, STCFG, CLEAR, 0);
    for ( i = 0; i < (8 * 1024 / 4); i ++ ) 
    {
        readbuffer[i] = *(uint32_t*)(flash_addr + i*4);       
    } 
    AM_BFW(CTIMER, STCFG, FREEZE, 0);
    vl_PutCoreToSleep(VLC_CORESLEEP_DEEPSLEEP);
    for ( i = 0; i < (8 * 1024 / 4); i ++) 
    {
        readbuffer[i] = *(uint32_t*)(flash_addr + i*4);       
    }  
    vl_am_hal_itm_enable(true);
    for ( i = 0; i < (8 * 1024 / 4); i ++ ) 
    {
        if (inverse)
        {
             if ( readbuffer[i] != ~(flash_addr+i*4) )
            {
                err_flag++;
                MSG(("Inverse Error, inst%d, page%d,addr %x, value%x\n",inst,page,flash_addr+(i*4),readbuffer[i]));
            }
        }else{
            if ( readbuffer[i] != (flash_addr+(i*4)) )
            {
                err_flag++;
                MSG(("Error, inst%d, page%d,addr %x, value%x\n",inst,page,flash_addr+(i*4),readbuffer[i]));
            }
        }
    }
    return 0;
}

// Override the default interrupt handler
void am_stimer_cmpr0_isr(void)
{
    AM_BFW(CTIMER, STCFG, FREEZE, 1);
    AM_REG(CTIMER, STMINTCLR) = 0xFFFFFFFF;
    AM_BFW(CTIMER, STCFG, CLEAR, 1);
    AM_BFW(CTIMER, STCFG, CLEAR, 0);
}

int main(void) CODE_SECTION;

// Main code
int main (void)
{  
    MCUInit();
    *(uint32_t*) 0x40018000 &= ~0x1;
    *(uint32_t*) 0x40018004 = 0x1751;
    *(uint32_t*) 0x40018000 |= 0x1;
    // *(uint32_t*) 0x4002100C |= (1 << 31); //power down cache in deep sleep
    pwrctrlSimoBuckEn();
    InitComms();
    
    MSG(("Cache Retention Test Start!\n"));

    gpiolib_config_pins(GPIO_TEST_START, 3, 0, 0, 0);
    while(gpiolib_get_bit(GPIO_TEST_START) == 0 ) {  }  
  
    reset_stimer();
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_STIMER_CMPR0);
    stimer_compare_en_and_val(0, 10); //10ms
    AM_BFWe(CTIMER,STCFG,CLEAR,CLEAR);
    AM_BFWe(CTIMER,STCFG,FREEZE,FREEZE);
    //AM_BFW(CTIMER, STCFG, CLKSEL, 1); //XTAL
    AM_BFW(CTIMER, STCFG, CLKSEL, 6); // LFRC
    AM_REG(CTIMER, STMINTEN) = 0x0;
    AM_REG(CTIMER, STMINTEN) = 1<<0; // compare 0

    gpiolib_clear_bit(GPIO_PASS);
    gpiolib_config_pins(GPIO_PASS, 3, 2, 0, 1);
    gpiolib_clear_bit(GPIO_HARDFAULT);
    gpiolib_config_pins(GPIO_HARDFAULT, 3, 2, 0, 1); 
    
    __enable_irq(); 

    stimer_go();
    AM_BFW(CTIMER, STCFG, FREEZE, 1);

    for(uint32_t flash_inst=0; flash_inst<1; flash_inst++) {
        for(uint32_t page=2; page<4; page++) {
            check_flash_instr(false,flash_inst,page);
        }
    }

    AM_BFW(CTIMER, STCFG, FREEZE, 0);
    AM_REG(CTIMER, STMINTEN) = 0x1;
    for(uint32_t flash_inst=0; flash_inst<1; flash_inst++) {
        for(uint32_t page=4; page<6; page++) {
            check_flash_instr(true,flash_inst,page);
        }
    }

    if (err_flag == 0 )
    {
        gpiolib_set_bit(GPIO_PASS);
        MSG(("\nPASS\n")); 
    }
    else
    {
        MSG(("\nFAIL\n")); 
    }

  while(1); 

  return 0;
}

//ISR
__attribute__((weak)) void am_fault_isr(void)
{
    gpiolib_set_bit(GPIO_HARDFAULT);
    MSG(("Hardfault!!\n"));
    while(1){ }
}

