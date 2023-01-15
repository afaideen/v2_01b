/*******************************************************************************
  SD Host Controller Device Driver Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdhc_host.c

  Summary:
    SD Host Controller Device Driver Dynamic Implementation

  Description:
    The SD Host Controller device driver provides a simple interface to manage the 
    SD Host Controller modules on Microchip microcontrollers.  This file Implements 
    the core interface routines for the SD Host Controller driver.

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include <sys/kmem.h>

#include "system_config.h"
#include "system_definitions.h"
#include "system/debug/sys_debug.h"
#include "driver/sdhc/src/drv_sdhc_host.h"
#include "driver/sdhc/src/drv_sdhc_sd.h"
#include "driver/sdhc/src/drv_sdhc_sdio_host.h"
#include "driver/sdhc/src/drv_sdhc_config.h"
#include "driver/sdhc/src/drv_sdhc_error_types.h"
#include "driver/sdhc/src/drv_sdhc_sdio_device.h"
#include "drv_sdhc_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************

volatile uint32_t interrupt_flag=0x00000000;

drv_sdhc_sd_handle_t card_handle;
drv_sdhc_sd_host_t host_handle;

uint32_t *system_buffer;
drv_sdhc_adma_descriptor *descriptorTable;
uint32_t block_count;   
uint8_t ext_csd_flag;
uint8_t scr_read_flag;
uint8_t sd_status_read_flag;
uint8_t sd_switch_flag;

void write_reg(uint32_t offset, uint32_t value)
{
    put_wvalue(offset,value);
}

void write_reg_hw(uint32_t offset, uint16_t value)
{
    put_hvalue(offset,value);
}

void write_reg_bit(uint32_t offset,uint32_t bit)
{
    set_wbit(offset,bit);  
}

void write_reg_hwbit(uint32_t offset, uint16_t bit)
{
    set_hbit(offset,bit);
}

void clear_reg_hwbit(uint32_t offset, uint16_t bit)
{
    clr_hbit(offset,bit);
}

void clear_reg_bit(uint32_t offset,uint32_t bit)
{
    clr_wbit(offset,bit);    
}

void write_reg_bbit(uint32_t offset, uint8_t bit)
{
    set_bit(offset,bit);
}

void clear_reg_bbit(uint32_t offset, uint8_t bit)
{
    clr_bit(offset,bit);
}

uint32_t read_reg_hw(uint32_t offset)
{
    return get_hvalue(offset);
}

void write_reg_b(uint32_t offset, uint8_t value)
{
    put_value(offset,value);
}

uint32_t read_reg_b(uint32_t offset)
{
    return get_value(offset);
}
void host_reg_write(drv_sdhc_sd_host_t *host, uint32_t offset,uint32_t value)
{
    write_reg((host->base_addr)+offset,value);
}

uint32_t read_reg(uint32_t offset)
{
    return get_wvalue(offset);
}

uint32_t host_reg_read(drv_sdhc_sd_host_t *host, uint32_t offset)
{
    return read_reg(host->base_addr+offset);
}

void host_reg_writehw(drv_sdhc_sd_host_t *host, uint32_t offset, uint16_t value)
{
    write_reg_hw(host->base_addr+offset,value);
}

uint32_t host_reg_readhw(drv_sdhc_sd_host_t *host, uint32_t offset)
{
    return read_reg_hw(host->base_addr+offset);
}

void host_reg_writeb(drv_sdhc_sd_host_t *host, uint32_t offset, uint8_t value)
{
    write_reg_b(host->base_addr+offset,value);
}

uint32_t host_reg_readb(drv_sdhc_sd_host_t *host, uint32_t offset)
{
    return read_reg_b(host->base_addr+offset);
}

void host_reg_write_bit(drv_sdhc_sd_host_t *host, uint32_t offset,uint32_t bit)
{
    write_reg_bit(host->base_addr+offset,bit);
}

void host_config_reg_write_bit(drv_sdhc_sd_host_t *host, uint32_t offset,uint32_t bit)
{
    write_reg_bit(host->config_addr+offset,bit);
}

void host_config_reg_clear_bit(drv_sdhc_sd_host_t *host, uint32_t offset,uint32_t bit)
{
    clear_reg_bit(host->config_addr+offset,bit);
}

void host_reg_clear_bit(drv_sdhc_sd_host_t *host, uint32_t offset,uint32_t bit)
{
    clear_reg_bit(host->base_addr+offset,bit);
}

void host_reg_writehw_bit(drv_sdhc_sd_host_t *host, uint32_t offset, uint16_t bit)
{
    write_reg_hwbit(host->base_addr+offset,bit);
}

void host_reg_clearhw_bit(drv_sdhc_sd_host_t *host, uint32_t offset, uint16_t bit)
{
    clear_reg_hwbit(host->base_addr+offset,bit);
}

void host_reg_writeb_bit(drv_sdhc_sd_host_t *host,uint32_t offset, uint8_t bit)
{
    write_reg_bbit(host->base_addr+offset,bit);
}

void host_reg_clearb_bit(drv_sdhc_sd_host_t *host,uint32_t offset, uint8_t bit)
{
    clear_reg_bbit(host->base_addr+offset,bit);
}

void host_cdssel_stabilization()
{
    //This is a workaround discovered during validation
    uint32_t ctrl =  host_reg_readb(&host_handle, SDIO3_SDHC_HCTL_1);
    ctrl |= CARD_DET_SIG_SEL;
	ctrl &= ~CARD_DET_TEST_LVL;
	host_reg_writeb(&host_handle,SDIO3_SDHC_HCTL_1,ctrl);    
}

void _drv_sdhc_set_bus_width(bus_width_t busWidth)
{
    uint32_t ctrl = host_reg_readb(&host_handle, SDIO3_SDHC_HCTL_1);
    
    if (busWidth == BIT_1)
    {
        ctrl &= ~BITBUS4_CTRL;
    }
    else if (busWidth == BIT_4)
    {
        ctrl |= BITBUS4_CTRL;
    }
    
	host_reg_writeb(&host_handle,SDIO3_SDHC_HCTL_1,ctrl);    
}

drv_sdhc_sd_handle_t* _drv_sdhc_check_card_inserted()
{
    drv_sdhc_sd_handle_t* card = &card_handle;
    //uint32_t pres_state = 0;
    
    if (card == NULL)
        return NULL;
    
    //pres_state = host_reg_read(card->host,SDIO3_SDHC_PSTATE);
    
    //if (pres_state & CARD_STATE_STABLE && pres_state & CARD_DETECT)
    if (host_reg_read(card->host,SDIO3_SDHC_PSTATE) & CARD_INSERTED)
	    return card;
    
    return NULL;
}

uint32_t _drv_sdhc_read_buffer_data()
{
    return host_reg_read(&host_handle,SDIO3_SDHC_BDP_0);           
}

void _drv_sdhc_write_buffer_data(uint32_t data)
{
    host_reg_write(&host_handle,SDIO3_SDHC_BDP_0, data);       
}

void _drv_sdhc_set_adma_descripter_table(drv_sdhc_adma_descriptor *descrTable)
{
    host_reg_write(&host_handle,SDIO3_SDHC_ADMA_SYS_ADDR, (uint32_t)(descrTable));   
}

void _drv_sdhc_set_transfer_block_size(uint16_t blk_size)
{
    blk_size &= 0x7FF; /*masking LSB 11 bits*/
    host_reg_writehw(&host_handle,SDIO3_SDHC_BLK_SIZE,blk_size);
    host_handle.cur_blk_size = blk_size;
}

void _drv_sdhc_set_block_count(uint16_t blk_count)
{
    host_reg_writehw(&host_handle,SDIO3_SDHC_BLK_CNT,blk_count);
    host_handle.cur_blk_cnt = blk_count;
}

void host_mmc_enable_push_pull(drv_sdhc_sd_host_t *host)
{
    host_reg_write_bit(host,SDIO3_SDHC_EMMC_CONFIG,EMMC_ENABLE_PUSHPULL);
}

void host_mmc_disable_push_pull(drv_sdhc_sd_host_t *host)
{
    host_reg_clear_bit(host,SDIO3_SDHC_EMMC_CONFIG,EMMC_ENABLE_PUSHPULL);
}

void host_set_voltage_select(drv_sdhc_sd_host_t *host,uint16_t voltage)
{
    if (host->pwr == voltage)
        return;
    
    host->pwr = voltage;
    
    if (voltage == 0) {
        host_reg_writeb(host,SDIO3_SDHC_PCTL,0);
        return;
    }
    
    //power = SD_BUS_PWRON; 
    
    host_reg_writeb(host,SDIO3_SDHC_PCTL,voltage);
}

void host_set_power(drv_sdhc_sd_host_t *host, bool enable)
{
    uint16_t power;
        
    power = host_reg_readb(host,SDIO3_SDHC_PCTL);    

    power |= (SD_BUS_PWRON & enable); 
    
    host_reg_writeb(host,SDIO3_SDHC_PCTL,power);    
}

void host_set_clock(drv_sdhc_sd_host_t *host, uint32_t clock,uint16_t shrd_clk)
{
    uint32_t div;
    uint32_t clk=0;
    unsigned long timeout;
    
    host_reg_writehw(host,SDIO3_SDHC_CLKCTL,0);
    
    if (clock == 0)
        return;
    
    if (host->version >= HOST_SPEC_300) {
        /* Version 3.00 divisors must be a multiple of 2. */
        if (host->max_clk <= clock)
            div = 1;
        else {
            for (div = 2; div < HOST_MAX_DIV_SPEC_300; div += 2) {
                if ((host->max_clk / div) <= clock)
                    break;
            }
        }
    } else {
        /* Version 2.00 divisors must be a power of 2. */
        for (div = 1; div < HOST_MAX_DIV_SPEC_200; div *= 2) {
            if ((host->max_clk / div) <= clock)
                break;
        }
    }
    div >>= 1;

    host_cdssel_stabilization();
    
    clk = (div & HOST_DIV_MASK) << HOST_DIVIDER_SHIFT;

    clk |= ((div & HOST_DIV_HI_MASK) >> HOST_DIV_MASK_LEN)
                                     << HOST_DIVIDER_HI_SHIFT;
    clk |= HOST_CLOCK_INT_EN;
    
    host_reg_writehw(host,SDIO3_SDHC_CLKCTL,clk);  //MJS Added fix
    
    /* Wait max 20 ms */
    timeout = 200;
    while (!((clk = host_reg_readhw(host, SDIO3_SDHC_CLKCTL))
             & HOST_CLOCK_INT_STABLE)) {
        if (timeout == 0) {
            SYS_DEBUG_MESSAGE(SYS_ERROR_FATAL,"\n\rInternal clock never stabilised.");
            return;
        }
        timeout--;
        /* add 1 ms delay here*/
        Nop();
    }
    
    if(shrd_clk)
    {
        clk = host->shared_clk;
        //host_shared_select_clk(host,clk<<16);
    }
    
    clk = host_reg_readhw(host,SDIO3_SDHC_CLKCTL);
    
    clk |= HOST_CLOCK_CARD_EN;
    host_reg_writehw(host,SDIO3_SDHC_CLKCTL,clk);
    
    host->clk= clock;    
}

void host_set_clear_irq(drv_sdhc_sd_host_t *host,uint32_t clear, uint32_t set)
{
    uint32_t ier;
    
    ier = host_reg_read(host, SDIO3_SDHC_INT_STS_EN);
    ier &= ~clear;
    ier |= set;

    host_reg_write(host,SDIO3_SDHC_INT_STS_EN, ier);
    host_reg_write(host,SDIO3_SDHC_INT_SIG_EN, ier);
}

DRV_SDHC_RESPONSE _drv_sdhc_host_master_init(const drv_sdhc_fuse_settings_t *fuse_settings, drv_sdhc_sd_host_t *host, drv_sdhc_adma_descriptor *descrTable)
{
    
    /* use_dma field defines whether host is going to use DMA or not*/
    if(fuse_settings->use_dma)
        host->flags |= HOST_USE_DMA;
    
    /*card_type defines from which card system is goint to boot*/
    switch (fuse_settings->card_type)
    {
    case TYPE_MMC:
        host->flags |= HOST_BOOT_FROM_MMC;
        break;
    case TYPE_SD:
        host->flags |= HOST_BOOT_FROM_SD;
        break;
    case TYPE_SDIO:
        host->flags |= HOST_BOOT_FROM_SDIO;
        break;
    case TYPE_eMMC:
        host->flags |= HOST_BOOT_FROM_EMMC;
        break;
    case TYPE_eSD:
        host->flags |= HOST_BOOT_FROM_ESD;
        break;
    case TYPE_eSDIO: 
        host->flags |= HOST_BOOT_FROM_ESDIO;
        break;
    default:
        host->flags |= HOST_BOOT_FROM_EMMC;
        break;
        
    }
    
    /* slot_type tells on what slot the device is connected*/
    switch(fuse_settings->slot_type)
    {
        
    case REMOVABLE:
        host->flags |= HOST_USE_REMOVABLE_SLOT;
        break;
    case SHARED_BUS: 
        host->flags |= HOST_USE_SHAREDBUS_SLOT;
        /*clock number used*/
        host->shared_clk = fuse_settings->shared_clk;
        /*interrupt pin  used*/
        host->shared_int = fuse_settings->shared_int;
        break;
    default:
        host->flags |= HOST_USE_SHAREDBUS_SLOT;
        break;
    }
    
    
    /*speed mode to be used*/
    switch(fuse_settings->bus_mode)
    {
    case SDR12_MODE:
        host->flags |= HOST_USE_SDR12_MODE;
        break;
    case SDR25_MODE:
        host->flags |= HOST_USE_SDR25_MODE;
        break;
    case SDR50_MODE:
        host->flags |= HOST_USE_SDR50_MODE;
        break;
    case SDR104_MODE:
        host->flags |= HOST_USE_SDR104_MODE;
        break;
    case DDR_50_MODE:
        host->flags |= HOST_USE_DDR50_MODE;
        break;
    case MMC_SDR25_MODE:
        host->flags |= HOST_USE_SDR25_MODE;
        break;
    case MMC_SDR50_MODE:
        host->flags |= HOST_USE_SDR25_MODE;
        break;
    case MMC_1_2V_DDR_MODE:
        host->flags |= HOST_USE_DDR50_MODE | HOST_ENABLE_1_2V_SIGNAL;
        break;
    case MMC_1_8V_DDR_MODE:
        host->flags |= HOST_USE_DDR50_MODE | HOST_ENABLE_1_8V_SIGNAL;
        break;
    default:
        host->flags |= HOST_USE_SDR25_MODE;
        break;
    }
    
/*bus_width tells the width should be used for boot*/
    switch(fuse_settings->bus_width)
    {
        
    case BIT_1:
        host->flags |= HOST_BOOT_BUS_WIDTH_1;
        break;
    case BIT_4: 
        host->flags |= HOST_BOOT_BUS_WIDTH_4;
        break;
    case BIT_8: 
        host->flags |= HOST_BOOT_BUS_WIDTH_8;
        break;
    default:
        host->flags |= HOST_BOOT_BUS_WIDTH_4;
        break;
    } 
    
    if(_drv_sdhc_host_init(host, descrTable) != DRV_SDHC_OK)
        return DRV_SDHC_FAIL;

//    if(host->flags & HOST_USE_SHAREDBUS_SLOT)
//        host_shared_init(&card_handle);

    return DRV_SDHC_OK;
}

void host_reset(drv_sdhc_sd_host_t *host)
{
    //uint32_t sw_reset_val=0;
    
    /* Issue software reset "All" */
    host_reg_writeb(host,SDIO3_SDHC_SFT_RST,HOST_RESET_ALL);
    
    /*wait untill host is reset*/
    while(host_reg_readb(host,SDIO3_SDHC_SFT_RST) & HOST_RESET_ALL);   
    
    /*enable all interrupts*/
    host_set_clear_irq(host, HOST_INT_ALL_MASK,
                       HOST_INT_DATA_END_BIT | 
                       HOST_INT_DATA_CRC | HOST_INT_DATA_TIMEOUT | HOST_INT_INDEX |
                       HOST_INT_END_BIT | HOST_INT_CRC | HOST_INT_TIMEOUT |
                       HOST_INT_DATA_END | HOST_INT_DMA_END | HOST_INT_RESPONSE);
    
}

void host_set_bus_speed(drv_sdhc_sd_handle_t *card, bus_speed_mode_t host_bus_speed)
{
    uint16_t hctl2_val;
  
    SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\n\rSetting host bus speed mode to %x",host_bus_speed);

    hctl2_val = host_reg_readhw(card->host,SDIO3_SDHC_CLKCTL);    
    hctl2_val &= ~HOST_CLOCK_CARD_EN;
    host_reg_writehw(card->host,SDIO3_SDHC_CLKCTL,hctl2_val);

    switch(host_bus_speed)
    {
        case SDR12_MODE:
            hctl2_val = host_reg_readhw(card->host,SDIO3_SDHC_HCTL_2);
            hctl2_val |= UHS_SDR12;
            host_reg_writehw(card->host,SDIO3_SDHC_HCTL_2,hctl2_val);
            break;

        case SDR25_MODE:
            hctl2_val = host_reg_readhw(card->host,SDIO3_SDHC_HCTL_2);
            hctl2_val |= UHS_SDR25;
            host_reg_writehw(card->host,SDIO3_SDHC_HCTL_2,hctl2_val);
            break;

        case SDR50_MODE:
            hctl2_val = host_reg_readhw(card->host,SDIO3_SDHC_HCTL_2);
            hctl2_val |= UHS_SDR50;
            host_reg_writehw(card->host,SDIO3_SDHC_HCTL_2,hctl2_val);
            break;

        case SDR104_MODE:
            hctl2_val = host_reg_readhw(card->host,SDIO3_SDHC_HCTL_2);
            hctl2_val |= UHS_SDR104;
            host_reg_writehw(card->host,SDIO3_SDHC_HCTL_2,hctl2_val);
            break;

        case DDR_50_MODE:
            hctl2_val = host_reg_readhw(card->host,SDIO3_SDHC_HCTL_2);
            hctl2_val |= UHS_DDR50;
            host_reg_writehw(card->host,SDIO3_SDHC_HCTL_2,hctl2_val);
            break;

        default: 
            SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"\n\rHost set bus speed mode: Invalid bus speed selection");
            break;
    }

    hctl2_val = host_reg_readhw(card->host,SDIO3_SDHC_CLKCTL);    
    hctl2_val |= HOST_CLOCK_CARD_EN;
    host_reg_writehw(card->host,SDIO3_SDHC_CLKCTL,hctl2_val);
}

void _drv_sdhc_host_irq(drv_sdhc_sd_handle_t *card)
{
    uint32_t int_mask;
    
    int_mask = host_reg_read(card->host,SDIO3_SDHC_INT_STS);
    
    if (int_mask != 0)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\n\rDRV_SDHC: Host Interrupt %08x",int_mask);
    }
    
    /*CARD INSERT INTERRUPT*/
    if(int_mask & HOST_INT_CARD_INSERT)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS, int_mask & HOST_INT_CARD_INSERT);
        interrupt_flag |= HOST_INT_CARD_INSERT;
    }

    /*CARD REMOVAL INTERRUPT*/
    if(int_mask & HOST_INT_CARD_REMOVE)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS, int_mask & HOST_INT_CARD_REMOVE);
        interrupt_flag |= HOST_INT_CARD_REMOVE;
    }

    /*COMMAND COMPLETE INTERRUPT*/
    if(int_mask & HOST_INT_RESPONSE)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS, int_mask & HOST_INT_RESPONSE);
        interrupt_flag |= CMD_COMPL_FLAG;
    }
    
    
    /*TRANSFER COMPLETE INTERRUPT*/
    if(int_mask & HOST_INT_DATA_END)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_DATA_END);
        interrupt_flag |= DATA_COMPLETE;
    }

    
    /*DMA INTERRUPT*/
    if(int_mask & HOST_INT_DMA_END)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_DMA_END);
        interrupt_flag |= DMA_INTERRUPT;
    }
    
    
    /*BUFFER WRITE READY INTERRUPT*/
    if(int_mask & HOST_INT_SPACE_AVAIL)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_SPACE_AVAIL);
        interrupt_flag |= BUFF_WR_READY_FLAG;
    }
    
    /*BUFFER READ READY INTERRUPT*/
    if(int_mask & HOST_INT_DATA_AVAIL)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_DATA_AVAIL);
        interrupt_flag |= BUFF_RD_READY_FLAG;
    }
        
    /*DATA TIME OUT*/
    if(int_mask & HOST_INT_DATA_TIMEOUT)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_DATA_TIMEOUT);
        SYS_DEBUG_PRINT(SYS_ERROR_ERROR,"\n\rDRV_SDHC: Data Time Out Error CMD%d",card->host->command->opcode);
		interrupt_flag |= HOST_INT_DATA_TIMEOUT;       
    }
    
    /*COMMAND TIMEOUT*/
    if(int_mask & HOST_INT_TIMEOUT)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_TIMEOUT);
        SYS_DEBUG_PRINT(SYS_ERROR_ERROR,"\n\rDRV_SDHC: Command Time Out Error CMD%d",card->host->command->opcode);
		interrupt_flag |= HOST_INT_TIMEOUT;
    }

    /*ACMD12 ERROR*/
    if(int_mask & HOST_INT_ACMD12_ERR)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_ACMD12_ERR);
        SYS_DEBUG_PRINT(SYS_ERROR_FATAL,"\n\rDRV_SDHC: ACMD12 Error CMD%d",card->host->command->opcode);
		interrupt_flag |= HOST_INT_ACMD12_ERR;
    }
    
    /*ADMA ERROR*/
    if(int_mask & HOST_INT_ADMA_ERR)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_ADMA_ERR);
        SYS_DEBUG_PRINT(SYS_ERROR_FATAL,"\n\rDRV_SDHC: ADMA Error CMD%d",card->host->command->opcode);
		interrupt_flag |= HOST_INT_ADMA_ERR;       
    }

    if(int_mask & HOST_INT_CUR_TIMEOUT_ERR)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_CUR_TIMEOUT_ERR);
        SYS_DEBUG_PRINT(SYS_ERROR_FATAL,"\n\rDRV_SDHC: Current Time Out Error CMD%d",card->host->command->opcode);
		interrupt_flag |= HOST_INT_CUR_TIMEOUT_ERR;
    }
    
    if(int_mask & HOST_INT_VENDOR_SPECIFIC_ERR)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_VENDOR_SPECIFIC_ERR);
        SYS_DEBUG_PRINT(SYS_ERROR_FATAL,"\n\rDRV_SDHC: Vendor Specific Error CMD%d",card->host->command->opcode);
		interrupt_flag |= HOST_INT_VENDOR_SPECIFIC_ERR;
    }    

    /*COMMAND CRC ERROR*/
    if(int_mask & HOST_INT_CRC)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_CRC);
        SYS_DEBUG_PRINT(SYS_ERROR_FATAL,"\n\rDRV_SDHC: Command CRC Error CMD%d",card->host->command->opcode);
		interrupt_flag |= HOST_INT_CRC;
       
    }
    
    /*DATA CRC ERROR*/
    if(int_mask & HOST_INT_DATA_CRC)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_DATA_CRC);
        SYS_DEBUG_PRINT(SYS_ERROR_FATAL,"\n\rDRV_SDHC: DATA CRC Error CMD%d",card->host->command->opcode);
		interrupt_flag |= HOST_INT_DATA_CRC;
     
    }
    
    /*COMMAND INDEX ERROR*/
    if(int_mask & HOST_INT_INDEX)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_INDEX);
        SYS_DEBUG_PRINT(SYS_ERROR_FATAL,"\n\rDRV_SDHC: Command Index Error CMD%d",card->host->command->opcode);
		interrupt_flag |= HOST_INT_INDEX;
     
    }
    
    /*DATA END BIT ERROR*/
    if(int_mask & HOST_INT_DATA_END_BIT)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_DATA_END_BIT);
        SYS_DEBUG_PRINT(SYS_ERROR_FATAL,"\n\rDRV_SDHC: DATA End Bit Error CMD%d",card->host->command->opcode);
		interrupt_flag |= HOST_INT_DATA_END_BIT;
  
    }
    
    /*Command END BIT ERROR*/
    if(int_mask & HOST_INT_END_BIT)
    {
        host_reg_write(card->host,SDIO3_SDHC_INT_STS,int_mask & HOST_INT_END_BIT);
        SYS_DEBUG_PRINT(SYS_ERROR_FATAL,"\n\rDRV_SDHC: Command End Bit Error CMD%d",card->host->command->opcode);
		interrupt_flag |= HOST_INT_END_BIT;
     
    }
    
    _drv_sdhc_handle_int(card);
}

uint16_t _drv_sdhc_transfer_mode(drv_sdhc_sd_handle_t *card)
{
    uint16_t transferMode = 0;
    sd_cmd_t* cmd = card->host->command;
    drv_sdhc_sd_host_t* host = card->host;
    bool enableDMA = (host->flags & HOST_USE_DMA);
    
    switch(cmd->opcode)
    {
        //CMD17 : Read Single Block
        case 17:
            {
                transferMode |= (SD_XFER_MODE_DMA_EN_MASK & enableDMA)
                        | SD_XFER_MODE_TO_HOST_DIR_MASK;
            }
            break;
        //CMD18 : Read Multi Block
        case 18:
            {
                transferMode |= SD_XFER_MODE_TO_HOST_DIR_MASK
                        | SD_XFER_MODE_ACMD12_EN_MASK
                        | SD_XFER_MODE_MULTI_BLK_SEL_MASK
                        | (SD_XFER_MODE_DMA_EN_MASK & enableDMA)
                        | SD_XFER_MODE_BLK_CNT_MASK;
            }
            break;
        //CMD24 : Write Single Block
        case 24:
            {
                transferMode |= (SD_XFER_MODE_DMA_EN_MASK & enableDMA);
            }
            break;
        //CMD25 : Write Multi Block
        case 25:
            {
                transferMode |= SD_XFER_MODE_MULTI_BLK_SEL_MASK
                        | SD_XFER_MODE_ACMD12_EN_MASK
                        | (SD_XFER_MODE_DMA_EN_MASK & enableDMA)
                        | SD_XFER_MODE_BLK_CNT_MASK;
            }
            break;
        default:
            break;
    }
    
    return transferMode;
}


DRV_SDHC_RESPONSE _drv_sdhc_host_init(drv_sdhc_sd_host_t *host, drv_sdhc_adma_descriptor *descrTable)
{
	uint32_t host_ver_val;
	uint32_t caps;
	uint32_t intr;
	uint32_t ctrl,ctr2;
	uint32_t dma_reg;
    uint16_t preset_val;
	uint16_t sd_clk;
	uint16_t clk_ctl;
	uint16_t driver_strength;
	uint16_t hctl2_reg;
	uint16_t top_bits,low_bits;
	uint16_t temp,actual_value;
	int16_t clk=0;
    unsigned long timeout;
	
	/*program base address of sd host*/
	host->base_addr = ((uint32_t)&SDHCBLKCON) - (uint32_t)SDIO3_SDHC_BLK_SIZE;
    host->config_addr = (uint32_t)&CFGCON2;
	
	host_ver_val = host_reg_readhw(host,SDIO3_SDHC_HCTL_VER);
	host_ver_val &= 0x00FF;
	
	if(host_ver_val > HOST_SPEC_300)
	{
        SYS_DEBUG_MESSAGE(SYS_ERROR_FATAL,"\n\rInvalid host:");
        return DRV_SDHC_FAIL;
	}
	host->version = host_ver_val;
	
	host_reset(host);

    /* Issue software reset for CMD line */
    host_reg_writeb(host,SDIO3_SDHC_SFT_RST, SFT_RST_CMD);
	
	/*reading capabilities*/
	caps = host_reg_read(host,SDIO3_SDHC_CAP1);
	host->caps1 = caps;
	caps = host_reg_read(host,SDIO3_SDHC_CAP2);
	host->caps2 = caps;
	
	/*setting maximum clock*/
	
//	if (host->version >= HOST_SPEC_300)
	host->max_clk = (host->caps1 & HOST_CLOCK_V3_BASE_MASK)
	>> HOST_CLOCK_BASE_SHIFT;
//	else
//	host->max_clk = (host->caps1 & HOST_CLOCK_BASE_MASK)
//	>> HOST_CLOCK_BASE_SHIFT;
	
	host->max_clk *= 1000000;
	
	if(host->max_clk == 0)
    {	
        SYS_DEBUG_MESSAGE(SYS_ERROR_FATAL,"\n\rDRV_SDHC: Hardware doesn't specify base clock frequency.");
        return DRV_SDHC_FAIL;
	}
	
	host->timeout_clk = (host->caps1 & HOST_TIMEOUT_CLK_MASK) >>
	HOST_TIMEOUT_CLK_SHIFT;
    
	if(host->timeout_clk == 0)
    {
    	SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"\n\rDRV_SDHC: Hardware doesn't specify timeout clock "
        "frequency.");
        return DRV_SDHC_FAIL;
	}
    
	if( host->caps1 & HOST_TIMEOUT_CLK_UNIT)
	host->timeout_clk *= 1000;
	
	/*minimum clock calculation*/
    if (host->version >= HOST_SPEC_300)
    {
        if(host->fuse_settings->use_preset == true)
        {
            preset_val		= host_reg_readhw(host,SDIO3_SDHC_PRESET_VAL);
            sd_clk			= preset_val & 0x03FF;
            driver_strength = (preset_val & 0xC000) >> 14;

            clk_ctl  = host_reg_readhw(host,SDIO3_SDHC_CLKCTL);
            clk_ctl  = clk_ctl & 0x3F;
            top_bits = (sd_clk & 0x0300) >> 8;
            low_bits = sd_clk & 0xFF;
            temp = (low_bits << 2) | top_bits;
            temp = temp << 6;
            clk_ctl |= (temp | HOST_CLOCK_INT_EN);
            host_reg_writehw(host,SDIO3_SDHC_CLKCTL,clk_ctl);		  

            /* Wait max 20 ms */
            timeout = 20;

            while (!((clk = host_reg_readhw(host, SDIO3_SDHC_CLKCTL))
             & HOST_CLOCK_INT_STABLE)) 
            {
                if (timeout == 0) 
                {
                    SYS_DEBUG_MESSAGE(SYS_ERROR_FATAL,"\n\rDRV_SDHC: Internal clock never stabilized.");
                    return DRV_SDHC_FAIL;
                }

                timeout--;

                /* add 1 ms delay here*/
                Nop();
            }

            clk = host_reg_readhw(host,SDIO3_SDHC_CLKCTL);

            clk |= HOST_CLOCK_CARD_EN;
            host_reg_writehw(host,SDIO3_SDHC_CLKCTL,clk);

            hctl2_reg = host_reg_readhw(host,SDIO3_SDHC_HCTL_2);
            hctl2_reg = hctl2_reg & 0xFFCF;
            actual_value = hctl2_reg | (driver_strength << 4);
            host_reg_writehw(host,SDIO3_SDHC_HCTL_2,actual_value);

        }
        else
        {
            host->min_clk= 400000; //400 KHz
            /*set the requested clock if removable slot enabled*/
            if(host->flags & HOST_USE_REMOVABLE_SLOT)
            {
				host_set_clock(host,host->min_clk,0);
            }	
            else
            {
				host_set_clock(host,host->min_clk,1);
            }
		}
    }
    else
    {
        host->min_clk = host->max_clk / HOST_MAX_DIV_SPEC_200;
        host_set_clock(host,25000000,0);
    }
	
	/*set bus voltage to defaut value 3.3*/
	host_set_voltage_select(host,SD_POWER_3_3);
	
	/* Set the timeout value */
	host_reg_writeb(host,SDIO3_SDHC_TCTL,0xE);	
	
	/* bus width setting*/
	ctrl = host_reg_readb(host, SDIO3_SDHC_HCTL_1);
	if (host->flags & HOST_BOOT_BUS_WIDTH_8) 
    {
        ctrl &= ~BITBUS4_CTRL;
        if (host->version >= HOST_SPEC_300)
            ctrl |= EXT_8BITBUS_CTRL;
	} 
    else 
    {
        if (host->version >= HOST_SPEC_300)
        {
            ctrl &= ~BITBUS4_CTRL;
        }
        else
        {
            // Default to 1-bit bus width at start
            ctrl &= ~BITBUS4_CTRL;
        }
	}
    
    if (host->flags & HOST_USE_DMA) 
    {
    	ctrl |= DMA_EN_CTRL;
    }

    // Default to High Speed ENable
	ctrl |= HISPD_EN_CTRL;
    
	host_reg_writeb(host,SDIO3_SDHC_HCTL_1,ctrl);

	/* bus speed setting*/
	if (host->version >= HOST_SPEC_300)
    {
        ctr2 = host_reg_readb(host,SDIO3_SDHC_HCTL_2);
        ctr2 &= ~UHS_MASK;
        
        if(host->flags & HOST_USE_SDR12_MODE)
            ctr2 |= UHS_SDR12;
        else if(host->flags & HOST_USE_SDR25_MODE)
            ctr2 |= UHS_SDR25;
        else if(host->flags & HOST_USE_SDR50_MODE)
            ctr2 |= UHS_SDR50;
        else if(host->flags & HOST_USE_SDR104_MODE)
            ctr2 |= UHS_SDR104;
        else if(host->flags & HOST_USE_DDR50_MODE)
            ctr2 |= UHS_DDR50;
        else
            ctr2 |= UHS_SDR12;
        
        host_reg_writeb(host,SDIO3_SDHC_HCTL_2,ctr2);
    }    
    

    if (host->flags & HOST_USE_DMA)
    {
        host->descriptor_table = descrTable;
        _drv_sdhc_set_adma_descripter_table(descrTable);
        SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\n\rDRV_SDHC: ADMA2 Enabled. Descriptor table set at %08x",host->descriptor_table);
        
        /*enable command and dma data interrupts*/
        intr = DMA_CMD_INT_MASK| DMA_DATA_INT_MASK;
    }
	else
    {
        host->descriptor_table = NULL;
        dma_reg = host_reg_read(host,SDIO3_SDHC_EMMC_CONFIG);
        dma_reg |= 0x20; //SD_DISABLE_DMA;
        host_reg_write(host,SDIO3_SDHC_EMMC_CONFIG,dma_reg);
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO,"\n\rDRV_SDHC: DMA Disabled");

        /*enable command and data interrupts*/
        intr = CMD_INT_MASK| DATA_INT_MASK;
    }
	
	host_set_clear_irq(host,0,intr);
	
    while(!((SDHCINTSTAT >> 6) & 0x1) && !((SDHCSTAT1 >> 16) & 0x1));

    return DRV_SDHC_OK;
}

DRV_SDHC_RESPONSE _drv_sdhc_sd_cmd_send(drv_sdhc_sd_handle_t *card)
{
    bool card_busy;
    uint16_t data_present = 0;
    uint16_t cmd_index_check;
    uint16_t cmd_crc_check;
    uint16_t response_type;
    uint16_t cmd_reg;
    uint16_t cmd_type;
    uint32_t pstate_res;    
    
    card_busy = true;
    
    pstate_res = host_reg_read(&host_handle,SDIO3_SDHC_PSTATE);
    
    /* check the card busy bit  */
    if ( 0 == (pstate_res & 0x3) )
    {
        card_busy = false;
    }
    
    if (card_busy)
    {
        return DRV_SDHC_FAIL;
    }
    
    // Enable all Interrupts status
    SDHCINTEN |= 0xFFFFFEFF;

    // Enable all Interrupt signals
    SDHCINTSEN |= 0xFFFFFEFF;

    /* setup the command, argument, transfer mode, and send it */
    host_reg_write(&host_handle,SDIO3_SDHC_ARG_LOW,host_handle.command->argument);
    host_reg_writehw(&host_handle,SDIO3_SDHC_TM ,host_handle.command->xfer_mode);
    
    if (host_handle.command->data_present)
    {
        data_present = 1;
    }
    
    cmd_type = 0;
    
    switch ( host_handle.command->resp_type )
    {
    case sd_response_r1:
        cmd_index_check = 0;
        cmd_crc_check = 1;
        response_type = SD_CMD_RSP_LEN_48;
        break;
        
    case sd_response_r1b:
        cmd_index_check = 1;
        cmd_crc_check = 1;
        response_type = SD_CMD_RSP_LEN_48BUSY;
        break;
        
    case sd_response_r2:
        cmd_index_check = 0;
        cmd_crc_check = 1;
        response_type = SD_CMD_RSP_LEN_136;
        break;
        
    case sd_response_r3:
        cmd_index_check = 0;
        cmd_crc_check = 0;
        response_type = SD_CMD_RSP_LEN_48;
        break;
        
    case sd_response_r4:
        cmd_index_check = 0;
        cmd_crc_check = 0;
        response_type = SD_CMD_RSP_LEN_48;
        break;
        
    case sd_response_r5:
        cmd_index_check = 0;
        cmd_crc_check = 1;
        response_type = SD_CMD_RSP_LEN_48;
        break;
        
    case sd_response_r6:
        cmd_index_check = 1;
        cmd_crc_check = 1;
        response_type = SD_CMD_RSP_LEN_48;
        break;
        
    case sd_response_r7:
        cmd_index_check = 1;
        cmd_crc_check = 1;
        response_type = SD_CMD_RSP_LEN_48;
        break;
        
    default:
        cmd_index_check = 0;
        cmd_crc_check = 0;
        response_type = SD_CMD_RSP_NONE;
        break;
    }

    cmd_reg = ( (host_handle.command->opcode<<8) | (cmd_type<<6) | (data_present<<5) |
                (cmd_index_check<<4) | (cmd_crc_check<<3) | (response_type<<0) );
    
    SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\n\rDRV_SDHC:Sending CMD%d Transfer Mode:%08x Argument:%08x",
            host_handle.command->opcode, cmd_reg, host_handle.command->xfer_mode, host_handle.command->argument);
    
    host_reg_writehw(&host_handle,SDIO3_SDHC_CMD,cmd_reg);
    
    return DRV_SDHC_OK;
}

DRV_SDHC_RESPONSE _drv_sdhc_capture_response(drv_sdhc_sd_handle_t *card)
{
    sd_cmd_t *cmd = card->host->command;
 
    /* clear the response buffer */
     cmd->response[0] = 0;
     cmd->response[1] = 0;
     cmd->response[2] = 0;
     cmd->response[3] = 0;
     cmd->response[4] = 0;
     cmd->response[5] = 0;
     cmd->response[6] = 0;
     cmd->response[7] = 0;
 
    /* save out response data */
    switch ( cmd->resp_type )
    {
        case sd_response_r2:
            
            cmd->response[0] = host_reg_readhw(card->host,SDIO3_SDHC_RSP7);
            cmd->response[1] = host_reg_readhw(card->host,SDIO3_SDHC_RSP6);
            cmd->response[2] = host_reg_readhw(card->host,SDIO3_SDHC_RSP5);
            cmd->response[3] = host_reg_readhw(card->host,SDIO3_SDHC_RSP4);
            cmd->response[4] = host_reg_readhw(card->host,SDIO3_SDHC_RSP3);
            cmd->response[5] = host_reg_readhw(card->host,SDIO3_SDHC_RSP2);
            cmd->response[6] = host_reg_readhw(card->host,SDIO3_SDHC_RSP1);
            cmd->response[7] = host_reg_readhw(card->host,SDIO3_SDHC_RSP0);
            
            break;
 
        case sd_response_r1:
        case sd_response_r1b:
        case sd_response_r3:
        case sd_response_r6:
        case sd_response_r7:
            cmd->response[0] = host_reg_readhw(card->host,SDIO3_SDHC_RSP0);
            cmd->response[1] = host_reg_readhw(card->host,SDIO3_SDHC_RSP1);
            break;

        case sd_response_r4:
        case sd_response_r5:          
        default:
            /* nothing to do for no response type */
            break;
    }
    return DRV_SDHC_OK;
}
 

void clear_int_flag(int flag)
{
    interrupt_flag &= ~flag;
}

void set_int_flag(int flag)
{
    interrupt_flag |= flag;
}

bool _drv_sdhc_is_write_protected(drv_sdhc_sd_handle_t *card)
{
    uint32_t pstate_res;
    pstate_res = host_reg_read(card->host,SDIO3_SDHC_PSTATE);
    
    return !(pstate_res | WRITE_PROTECT);
}

DRV_SDHC_RESPONSE _drv_sdhc_handle_int(drv_sdhc_sd_handle_t * card)
{
    uint32_t waitCounter = 0;
    sd_cmd_t *cmd = card->host->command;

    if(interrupt_flag &(HOST_INT_DATA_TIMEOUT | HOST_INT_TIMEOUT | HOST_INT_CRC | HOST_INT_DATA_CRC | HOST_INT_INDEX | HOST_INT_DATA_END_BIT))
    {
        if(interrupt_flag & HOST_INT_DATA_CRC)
        {
            /*update current host block count*/
            card->host->cur_blk_cnt = host_reg_readhw(card->host,SDIO3_SDHC_BLK_CNT);
            clear_int_flag(0xFFFFFFFF);
            cmd->data_crc_error_flag = 1;
            _drv_sdhc_reset_error(card_handle.host, SFT_RST_DATA);

            SYS_DEBUG_PRINT(SYS_ERROR_FATAL,"\n\rDRV_SDHC:Data Transfer CRC Error CMD%d Block%d",cmd->opcode,card->host->cur_blk_cnt);

            return DRV_SDHC_FAIL;
        }

        if(cmd->opcode == 8)
        {
            if(interrupt_flag & HOST_INT_TIMEOUT)
            {
                cmd->cmd_completed = 1;
                _drv_sdhc_reset_error(card->host, SFT_RST_CMD);
                clear_int_flag(HOST_INT_TIMEOUT);
                return DRV_SDHC_OK;
            }
        }

        //If this is to un-select a card, we are not going to receive a response from the card, hence the command timeout
        if(cmd->opcode == 7 && cmd->rca == 0)
        {
            if(interrupt_flag & HOST_INT_TIMEOUT)
            {
                cmd->cmd_completed = 1;
                _drv_sdhc_reset_error(card->host, SFT_RST_CMD);
                clear_int_flag(HOST_INT_TIMEOUT);
                return DRV_SDHC_OK;
            }
        }

        if(interrupt_flag & CMD_COMPL_FLAG)
        {
            _drv_sdhc_reset_error(card->host, SDHCI_RESET_CMD);
            clear_int_flag(CMD_COMPL_FLAG);
        }

        if(cmd->opcode == 17 ||
                cmd->opcode == 18 ||
                cmd->opcode == 24 ||
                cmd->opcode == 25)
        {
            if(interrupt_flag & (DATA_COMPLETE | HOST_INT_DATA_TIMEOUT))
            {
                cmd->data_completed = true;
                clear_int_flag(DATA_COMPLETE | HOST_INT_DATA_TIMEOUT);
                return DRV_SDHC_OK;
            }            
        }

        cmd->cmd_error_flag = interrupt_flag;
        set_int_flag(HOST_BUS_ERROR);
        return DRV_SDHC_FAIL;
    }

    if(interrupt_flag & BUFF_RD_READY_FLAG)
    {
        cmd->buffer_ready = true;
        clear_int_flag(BUFF_RD_READY_FLAG);
    }

    if(interrupt_flag & BUFF_WR_READY_FLAG)
    {
        cmd->buffer_ready = true;
        clear_int_flag(BUFF_WR_READY_FLAG);
    }

    if(interrupt_flag & CMD_COMPL_FLAG)
    {
        if(interrupt_flag & ~0xFFFE0000)
        {
            SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\n\rDRV_SDHC:Command Complete CMD%d",cmd->opcode);
            if(_drv_sdhc_capture_response(card) == DRV_SDHC_OK)
            {
                cmd->cmd_completed = 1;
                clear_int_flag(CMD_COMPL_FLAG);
            }           
        }
    }

    if(interrupt_flag & DATA_COMPLETE)
    {
        if(interrupt_flag & ~0xFFFF0000)
        {
            SYS_DEBUG_PRINT(SYS_ERROR_INFO,"\n\rDRV_SDHC:Data Transfer Complete CMD%d",cmd->opcode);
            cmd->data_completed = true;
            clear_int_flag(DATA_COMPLETE);           
        }
    }    

    // Card inserted enable SD Bus Power
    if(interrupt_flag & HOST_INT_CARD_INSERT)
    {
        host_set_power(card->host, true);
        clear_int_flag(HOST_INT_CARD_INSERT);
        
        while (waitCounter < 1000000)
        {
            waitCounter++;
            Nop();
        }
    }
    
    // Card removed disable SD Bus Power
    if(interrupt_flag & HOST_INT_CARD_REMOVE){
        host_set_power(card->host, false);
        clear_int_flag(HOST_INT_CARD_REMOVE);
    }

    // If this is ADMA2, this interrupt is for debugging and can be ignored
    if (interrupt_flag & DMA_INTERRUPT) {
        clear_int_flag(HOST_INT_DMA_END);        
    }

    // If this is ADMA2 and we are here, it means this is a descriptor error that should be ignored
    if (interrupt_flag & HOST_INT_ADMA_ERR) {
        clear_int_flag(HOST_INT_ADMA_ERR);        
    }
    
  return DRV_SDHC_OK;
}

#define MAX_FBR_REGS   7
sdio_op_cond_info_t    gbl_io_resp;
fbr_all_regs_t         gbl_fbr_all_regs;
fbr_reg_info_t         gbl_fbr_regs[MAX_FBR_REGS];

DRV_SDHC_RESPONSE _drv_sdhc_init(const drv_sdhc_fuse_settings_t *fuse_settings, drv_sdhc_adma_descriptor *descrTble,sd_cmd_t* cmd)
{
	DRV_SDHC_RESPONSE err;
	
	//MJS Added memory init
	card_handle.io_resp        = &gbl_io_resp;
	card_handle.fbr_all_regs   = &gbl_fbr_all_regs;
	gbl_fbr_all_regs.fbr_regs  = gbl_fbr_regs;
	
	//End MJS Added memory init.
	
	host_handle.fuse_settings = fuse_settings; //MJS initialization fix.
    host_handle.command = cmd;
	
	switch (fuse_settings ->ip_mode)
	{
        case HOST_SD:
        case HOST_SPI:
            card_handle.host = &host_handle;
            err = _drv_sdhc_host_master_init(fuse_settings,&host_handle,descrTble);

            if(err != DRV_SDHC_OK){
                SYS_DEBUG_MESSAGE(SYS_ERROR_FATAL,"\n\rDRV_SDHC:Error occurred at initialization");
                return DRV_SDHC_FAIL;
            }
	
        	break;
	
        case DEV_SD:
        case DEV_SPI:
	
        	break;
	}

return DRV_SDHC_OK;
}

void _drv_sdhc_reset_error(drv_sdhc_sd_host_t *host, uint8_t byte)
{
    unsigned long timeout;
    uint32_t regset = 0;
    
    /* Issue software reset for data, cmd, or all errors */
    host_reg_writeb(host,SDIO3_SDHC_SFT_RST, byte);
    
    /* Wait max 100 ms */
    timeout = 100;
    
    /*wait until host is resets the error status*/
    while(host_reg_readb(host,SDIO3_SDHC_SFT_RST) & byte){
        if(timeout == 0){
            SYS_DEBUG_MESSAGE(SYS_ERROR_FATAL,"\n\rDRV_SDHC: Host Controller Reset Failed");
            return;
        }
        timeout--;
        Nop();
    }
    
    if (byte & SDHCI_RESET_CMD || byte & SDHCI_RESET_ALL)
    {
        regset |= CMD_INT_MASK;
    }
    
    if (byte & SDHCI_RESET_DATA || byte & SDHCI_RESET_ALL)
    {
        regset |= DATA_INT_MASK;
    }
    
    /*enable all interrupts*/
    host_set_clear_irq(host, HOST_INT_ALL_MASK, HOST_INT_ALL_MASK);   
}
