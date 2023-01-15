/*******************************************************************************
  MPLAB Harmony LCC Generated Driver Implementation File

  File Name:
    drv_gfx_lcc_generic.c

  Summary:
    Build-time generated implementation for the LCC Driver.

  Description:
    Build-time generated implementation for the LCC Driver.

    Created with MPLAB Harmony Version ${CONFIG_MPLAB_HARMONY_VERSION_STRING}
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2016 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#include "framework/driver/gfx/controller/lcc/drv_gfx_lcc_generic.h"

#include <xc.h>
#include <sys/attribs.h>

#include "peripheral/pmp/plib_pmp.h"
#include "peripheral/tmr/plib_tmr.h"
<#if CONFIG_PIC32MX == false && CONFIG_DRV_GFX_LCC_DMA_BUFFER_SIZE_MODE == "Large Buffer">
#include "peripheral/ebi/plib_ebi.h"
</#if>

<#if CONFIG_DRV_GFX_USE_LCC_PALETTE == true>
#include "gfx/gfx_palette.h"
uint16_t LUT[256];
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory">
#ifndef PMADDR_OVERFLOW
#define  PMADDR_OVERFLOW               65536
#endif
inline void LCCRecoverData(void);
inline void LCCSaveData(void);
<#if CONFIG_DRV_GFX_LCC_DMA_WRITE_STRATEGY == "Draw Count Limited">
#define PIXEL_DRAW_PER_DMA_TX          ${CONFIG_USE_DRV_GFX_LCC_DRAW_COUNT_PER_TX}
volatile uint8_t DrawCount = 0;                          /* The current status of how many pixels have been drawn inbetween a DMA IR*/
</#if>
volatile uint8_t overflowcount;                      /* The count for the amount of overflows that have happened in the PMP Adress*/

#define SRAM_ADDR_CS0  0xE0000000
<#if CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WQVGA or lower">
#define SRAM_ADDR_CS1  0xE0040000
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "HVGA">
#define SRAM_ADDR_CS1  0xE0060000
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "VGA">
#define SRAM_ADDR_CS1  0xE00A0000
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WVGA">
#define SRAM_ADDR_CS1  0xE00C0000
</#if>
</#if>

#define VER_BLANK                 (DISP_VER_PULSE_WIDTH+DISP_VER_BACK_PORCH+DISP_VER_FRONT_PORCH-1)


static SYS_DMA_CHANNEL_HANDLE dmaHandle = SYS_DMA_CHANNEL_HANDLE_INVALID;

<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
<#if CONFIG_DRV_GFX_LCC_DMA_BUFFER_SIZE_MODE == "Large Buffer">
#define EBI_ADDR_CS0   0xC0000000
</#if>
#define PMP_ADDRESS_LINES 0
#define PAGE_COUNT 1
<#if CONFIG_DRV_GFX_USE_LCC_PALETTE == true>
GFX_COLOR *LUTAddress;
uint16_t  __attribute__((coherent, aligned(16))) frameLineBuf[DISP_HOR_RESOLUTION];
</#if>
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
uint16_t GraphicsFrame[DISP_HOR_RESOLUTION];
<#if CONFIG_PIC32MX == true>
#define PMP_ADDRESS_LINES 0x7fff
<#else>
#define PMP_ADDRESS_LINES 0xffff
</#if>
</#if>
<#if CONFIG_USE_GFX_STACK == true>
<#if CONFIG_GFX_CFG_COLOR_DEPTH_SIZE == "GFX_COLOR_DEPTH_16">
#define PMP_DATA_LENGTH PMP_DATA_SIZE_16_BITS
<#else> 
<#if CONFIG_DRV_GFX_USE_LCC_PALETTE == true>
#define PMP_DATA_LENGTH PMP_DATA_SIZE_16_BITS
<#else>
#define PMP_DATA_LENGTH PMP_DATA_SIZE_8_BITS
</#if></#if>
<#else>
#define PMP_DATA_LENGTH PMP_DATA_SIZE_16_BITS //Segger only supports 16 bpp color depth
</#if>
uint16_t HBackPorch = (DISP_HOR_PULSE_WIDTH+DISP_HOR_BACK_PORCH);

<#if CONFIG_DRV_GFX_USE_LCC_PIP == true>
//PIP Variables (NULL at start)
uint16_t PipStartT = 0;
uint16_t PipStartL = 0;
uint16_t PipVLength = 0;
uint16_t PipHLength = 0;
static uint32_t PipX,PipY;
uint8_t GFXPIPPage=0;
</#if>
<#if CONFIG_DRV_GFX_USE_LCC_SCROLL == true>
uint16_t scroll,scrollLine,scrollPage = 0;
</#if>

<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
volatile GFX_COLOR *activePageAddress;
volatile GFX_COLOR *frameBufferAddress; 
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
volatile uint16_t* _ebiBaseAddress = (uint16_t*)SRAM_ADDR_CS0;

volatile uint16_t _pixelRendercount;
volatile uint16_t* _framePoint;
volatile uint32_t _frameAddress;
</#if>

volatile bool allowFrameUpdate = false;
volatile bool allowLineUpdate = false;

// *****************************************************************************
/* GFX LCC Driver Instance Object

  Summary:
    Defines the object required for the maintenance of the hardware instance.

  Description:
    This defines the object required for the maintenance of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None.
*/

typedef struct _DRV_GFX_LCC_OBJ
{
    /* Flag to indicate in use  */
    bool                                        inUse;

    /* Save the index of the driver */
    SYS_MODULE_INDEX                            drvIndex;

    /* LCC machine state */
    DRV_GFX_STATES                              state;

    /* Status of this driver instance */
    SYS_STATUS                                  status;

    /* Number of clients */
    uint32_t                                    nClients;

    /* Client of this driver */
    DRV_GFX_CLIENT_OBJ *                        pDrvLCCClientObj;

    DRV_GFX_INIT *                              initData;

    uint16_t      maxY;
    uint16_t      maxX;
	
	LCC_VSYNC_Callback_FnPtr                    vsync_cb;

} DRV_GFX_LCC_OBJ;

static DRV_GFX_LCC_OBJ        drvLCCObj;
static DRV_GFX_CLIENT_OBJ drvLCCClients;

<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
// *****************************************************************************
/*
  Function: DRV_GFX_LCC_GetBuffer( void )

  Summary:
    returns address to the framebuffer

  Description:
    none

  Input:
    none

  Output:
    address to the framebuffer
*/

unsigned short * DRV_GFX_LCC_GetBuffer(void) {
    return (unsigned short *)frameBufferAddress;
}

// *****************************************************************************
/*
  Function: DRV_GFX_LCC_FrameBufferAddressSet( void * address )

  Summary:
    Sets address of the framebuffer

  Description:
    none

  Input:
    none

  Output:
    Sets address of the framebuffer
*/

uint16_t DRV_GFX_LCC_FrameBufferAddressSet( void * address )
{

    if ( NULL == address )
    {
        return 1;
    }

    frameBufferAddress = (GFX_COLOR *) address;

    return 0;
}
</#if>

// *****************************************************************************
/*
  Function: uint16_t DRV_GFX_LCC_Initialize(uint8_t instance)

  Summary:
    resets LCD, initializes PMP

  Description:
    none

  Input:
        instance - driver instance
  Output:
    1 - call not successful (PMP driver busy)
    0 - call successful
*/
SYS_MODULE_OBJ DRV_GFX_LCC_Initialize(const SYS_MODULE_INDEX index,
                                      const SYS_MODULE_INIT* const init)
{
    static uint16_t horizontalSize, verticalSize;

    /* Validate the driver index */
    if ( index >= GFX_CONFIG_NUMBER_OF_MODULES )
        return SYS_MODULE_OBJ_INVALID;

    DRV_GFX_LCC_OBJ *dObj = &drvLCCObj;

    /* Object is valid, set it in use */
    dObj->inUse = true;
    dObj->state = SYS_STATUS_BUSY;
    dObj->initData = (DRV_GFX_INIT *) init;

    /* Save the index of the driver. Important to know this
    as we are using reference based accessing */
    dObj->drvIndex = index;
    drvLCCObj.initData->TCON_Init = TCON_MODULE;

<#if CONFIG_DRV_GFX_DISPLAY_SYS_INIT_SCRIPT?has_content>
	${CONFIG_DRV_GFX_DISPLAY_SYS_INIT_SCRIPT}
</#if>
    if ((drvLCCObj.initData->orientation == 90) || (drvLCCObj.initData->orientation == 270))
	{
        horizontalSize = drvLCCObj.initData->verticalResolution;
        verticalSize = drvLCCObj.initData->horizontalResolution;
        drvLCCObj.maxX = horizontalSize - 1;
        drvLCCObj.maxY = verticalSize - 1;
    }
	else
	{
        horizontalSize = drvLCCObj.initData->horizontalResolution;
        verticalSize = drvLCCObj.initData->verticalResolution;
        drvLCCObj.maxX = horizontalSize - 1;
        drvLCCObj.maxY = verticalSize - 1;
    }

<#if CONFIG_DRV_GFX_DISP_USE_RESET == true>
<#if CONFIG_DRV_GFX_DISP_RESET_POSITIVE_POLARITY == true>
    BSP_LCD_RESETOn();
<#else>
    BSP_LCD_RESETOff();
</#if></#if>
<#if CONFIG_DRV_GFX_DISP_USE_CHIP_SELECT == true>
<#if CONFIG_DRV_GFX_DISP_CHIP_SELECT_POSITIVE_POLARITY == true>
    BSP_LCD_CSOff();
<#else>
    BSP_LCD_CSOn();
</#if></#if>

    if (drvLCCObj.initData->TCON_Init != NULL)
		drvLCCObj.initData->TCON_Init();

    drvLCCObj.initData->activePage = 0;
    drvLCCObj.initData->visualPage = 0;

<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
    activePageAddress  = KVA0_TO_KVA1(frameBufferAddress);
<#if CONFIG_PIC32MX == false && CONFIG_DRV_GFX_LCC_DMA_BUFFER_SIZE_MODE == "Large Buffer">
    PLIB_EBI_WriteOutputControlSet(EBI_ID_0, true, false);
    PLIB_EBI_DataEnableSet(EBI_ID_0, true, true);
    
    PLIB_EBI_BaseAddressSet(EBI_ID_0, 0, 0x20000000);
    PLIB_EBI_MemoryCharacteristicsSet(EBI_ID_0, 0, SRAM, MEMORY_SIZE_8MB, CS_TIMING_0);
    PLIB_EBI_MemoryTimingConfigSet(EBI_ID_0, 0, 0, 0, 4, 3, 3, 0);

    PLIB_EBI_StaticMemoryWidthRegisterSet(EBI_ID_0, 0, MEMORY_WIDTH_16BIT);
    PLIB_EBI_FlashPowerDownModeSet(EBI_ID_0, true);
<#else>
    /* Disable the PMP module */
    PLIB_PMP_Disable(0);

    PLIB_PMP_OperationModeSelect(0, PMP_MASTER_READ_WRITE_STROBES_INDEPENDENT);

    /* pins polarity setting */
    PLIB_PMP_ReadWriteStrobePolaritySelect(0, 1 - (DISP_INV_LSHIFT));
    PLIB_PMP_WriteEnableStrobePolaritySelect(0, PMP_POLARITY_ACTIVE_LOW);

    PLIB_PMP_ReadWriteStrobePortEnable(0);
    PLIB_PMP_WriteEnableStrobePortEnable(0);

    PLIB_PMP_DataSizeSelect(0, PMP_DATA_LENGTH);

    /* wait states setting */
    PLIB_PMP_WaitStatesDataHoldSelect(0, PMP_DATA_HOLD_1);
    PLIB_PMP_WaitStatesDataSetUpSelect(0, PMP_DATA_WAIT_ONE);
    PLIB_PMP_WaitStatesStrobeSelect(0, PMP_STROBE_WAIT_3);

    /* setting the hardware for the required interrupt mode */
    PLIB_PMP_InterruptModeSelect(0, PMP_INTERRUPT_EVERY_RW_CYCLE);

    PLIB_PMP_AddressIncrementModeSelect(0, PMP_ADDRESS_AUTO_INCREMENT);

    /* Enable the PMP module */
    PLIB_PMP_Enable(0);
</#if>
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
    /* Disable the PMP module */
    PLIB_PMP_Disable(0);

    PLIB_PMP_OperationModeSelect(0, PMP_MASTER_READ_WRITE_STROBES_INDEPENDENT);

    /* pins polarity setting */
    PLIB_PMP_ReadWriteStrobePolaritySelect(0, 1 - (DISP_INV_LSHIFT));
    PLIB_PMP_WriteEnableStrobePolaritySelect(0, PMP_POLARITY_ACTIVE_LOW);

    PLIB_PMP_ReadWriteStrobePortEnable(0);
    PLIB_PMP_WriteEnableStrobePortEnable(0);

    PLIB_PMP_DataSizeSelect(0, PMP_DATA_LENGTH);

    /* wait states setting */
    PLIB_PMP_WaitStatesDataHoldSelect(0, PMP_DATA_HOLD_1);
    PLIB_PMP_WaitStatesDataSetUpSelect(0, PMP_DATA_WAIT_ONE);
    PLIB_PMP_WaitStatesStrobeSelect(0, PMP_STROBE_WAIT_3);

    /* setting the hardware for the required interrupt mode */
    PLIB_PMP_InterruptModeSelect(0, PMP_INTERRUPT_EVERY_RW_CYCLE);

    PLIB_PMP_AddressIncrementModeSelect(0, PMP_ADDRESS_AUTO_INCREMENT);

    /* Enable the PMP module */
    PLIB_PMP_Enable(0);
    /* Ports Setting */
    PLIB_PMP_AddressPortEnable(0, PMP_ADDRESS_LINES);
    PLIB_PMP_AddressSet(0, 0);

<#if CONFIG_PIC32MX == true>
#ifdef BSP_SRAM_A15Off
    BSP_SRAM_A15Off();
#endif
<#else>
    BSP_SRAM_A16Off();
    BSP_SRAM_A17Off();
    BSP_SRAM_A18Off();

<#if CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_19 == true>
    BSP_SRAM_A19Off();
</#if>
</#if>
<#if CONFIG_PIC32MX == false && CONFIG_DRV_GFX_LCC_DMA_BUFFER_SIZE_MODE == "Large Buffer">
    PLIB_EBI_WriteOutputControlSet(EBI_ID_0, true, false);
    PLIB_EBI_DataEnableSet(EBI_ID_0, true, true);
    PLIB_EBI_BaseAddressSet(EBI_ID_0, 0, 0x20000000);
    PLIB_EBI_MemoryCharacteristicsSet(EBI_ID_0, 0, ${CONFIG_DRV_GFX_LCC_EXTERNAL_MEMORY_TYPE}, ${CONFIG_DRV_GFX_LCC_EXTERNAL_MEMORY_SIZE}, CS_TIMING_0);
    PLIB_EBI_MemoryTimingConfigSet(EBI_ID_0, ${CONFIG_DRV_GFX_LCC_EXTERNAL_MEMORY_CHIP_SELECT}, 0, 0, 0, 0, 0, 1);
    PLIB_EBI_StaticMemoryWidthRegisterSet(EBI_ID_0, 0, ${CONFIG_DRV_GFX_LCC_EXTERNAL_MEMORY_WIDTH});
    PLIB_EBI_FlashPowerDownModeSet(EBI_ID_0, true);
</#if>
</#if>

    /*Turn Backlight on*/
<#if CONFIG_DRV_GFX_DISP_BACKLIGHT_ENABLE_LEVEL == "1">
    BSP_LCD_BACKLIGHTOn();
<#else>
    BSP_LCD_BACKLIGHTOff();
</#if>

    drvLCCObj.initData->horizontalResolution = horizontalSize;
    drvLCCObj.initData->verticalResolution = verticalSize;
    drvLCCObj.state = SYS_STATUS_READY;

    dObj->nClients = 0;
    dObj->status = SYS_STATUS_READY;

    /* Return the driver handle */
    return (SYS_MODULE_OBJ)dObj;
}

static int DRV_GFX_LCC_Start()
{
	/*Suspend DMA Module*/
    SYS_DMA_Suspend();
	
    /* Allocate DMA channel */
    dmaHandle = SYS_DMA_ChannelAllocate(DRV_GFX_LCC_DMA_CHANNEL_INDEX);
    
	if (SYS_DMA_CHANNEL_HANDLE_INVALID == dmaHandle)
        return -1;
		
    // set the transfer parameters: source & destination address,
    // source & destination size, number of bytes per event
<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
<#if CONFIG_DRV_GFX_LCC_DMA_BUFFER_SIZE_MODE == "Large Buffer">
    SYS_DMA_ChannelTransferAdd( dmaHandle,
                               (void *)frameBufferAddress,
                               (HBackPorch << 1),
                               (uint32_t*) EBI_ADDR_CS0,
                               (1 << PMP_DATA_LENGTH),
                               (HBackPorch << 1));

    // once we configured the DMA channel we can enable it
    SYS_DMA_ChannelEnable( dmaHandle );

    SYS_DMA_ChannelForceStart( dmaHandle );
<#else>
    SYS_DMA_ChannelTransferAdd(  dmaHandle,
                                (void *)frameBufferAddress,
                                HBackPorch << PMP_DATA_LENGTH,
                                (void *) &PMDIN,
                                1 << PMP_DATA_LENGTH,
                                1 << PMP_DATA_LENGTH );

    /* Enable the transfer done interrupt, when all buffer transferred */
    PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX,
            DMA_INT_BLOCK_TRANSFER_COMPLETE);

    SYS_INT_SourceEnable(INT_SOURCE_DMA_0 + DRV_GFX_LCC_DMA_CHANNEL_INDEX);

    // set the transfer event control: what event is to start the DMA transfer
    SYS_DMA_ChannelSetup(dmaHandle,
            SYS_DMA_CHANNEL_OP_MODE_BASIC,
            DRV_GFX_LCC_DMA_TRIGGER_SOURCE);

    PLIB_TMR_PrescaleSelect(DRV_GFX_LCC_TMR_INDEX, 0);
    PLIB_TMR_Period16BitSet(DRV_GFX_LCC_TMR_INDEX, 1);
    PLIB_TMR_Start(DRV_GFX_LCC_TMR_INDEX);

    // once we configured the DMA channel we can enable it
    SYS_DMA_ChannelEnable(dmaHandle);
</#if>
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
    SYS_DMA_ChannelTransferAdd(dmaHandle, (void *) &PMDIN, 2, &GraphicsFrame[0],
            HBackPorch, DRV_GFX_LCC_DMA_TRANSFER_LENGTH);
</#if>

<#if CONFIG_PIC32MX == true || CONFIG_DRV_GFX_LCC_DMA_BUFFER_SIZE_MODE == "Large Buffer">
    /* Enable the transfer done interrupt, when all buffer transferred */
    PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX,
            DMA_INT_BLOCK_TRANSFER_COMPLETE);

    SYS_INT_SourceEnable(INT_SOURCE_DMA_0 + DRV_GFX_LCC_DMA_CHANNEL_INDEX);

<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
    PLIB_DMA_ChannelXAutoEnable(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX);
</#if>
    // set the transfer event control: what event is to start the DMA transfer
    SYS_DMA_ChannelSetup(dmaHandle,
            SYS_DMA_CHANNEL_OP_MODE_BASIC,
            DRV_GFX_LCC_DMA_TRIGGER_SOURCE);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory">
    PLIB_TMR_PrescaleSelect(DRV_GFX_LCC_TMR_INDEX, ${CONFIG_DRV_GFX_LCC_TMR_PRESCALE_VALUE});
    PLIB_TMR_Period16BitSet(DRV_GFX_LCC_TMR_INDEX, 1);
    PLIB_TMR_Start(DRV_GFX_LCC_TMR_INDEX);

    // once we configured the DMA channel we can enable it
    SYS_DMA_ChannelEnable(dmaHandle);
</#if>

<#if CONFIG_DRV_GFX_DISPLAY_SYS_START_SCRIPT?has_content>
	${CONFIG_DRV_GFX_DISPLAY_SYS_START_SCRIPT}
</#if>
    /*Unsuspend DMA Module*/
    SYS_DMA_Resume();
	
	return 0;
}

// *****************************************************************************
/*
  Function: void DRV_GFX_LCC_VSYNC_CallbackSet(LCC_VSYNC_Callback_FnPtr cb)

  Summary:
    sets VSYNC state callback pointer

  Description:
    none

  Input:
        cb - callback pointer
*/
void DRV_GFX_LCC_VSYNC_CallbackSet(LCC_VSYNC_Callback_FnPtr cb)
{
	drvLCCObj.vsync_cb = cb;
}

// *****************************************************************************
/*
  Function: LCC_VSYNC_STATE DRV_GFX_LCC_VSYNC_GetState(void)

  Summary:
    Gets the state of the current VSYNC mode

  Description:
    none

  Output:
        state of VSYNC register
		LCC_VSYNC_TRUE = in VSYNC
		LCC_VSYNC_FALSE = not in VSYNC
*/
LCC_VSYNC_STATE DRV_GFX_LCC_VSYNC_GetState(void)
{
	return BSP_LCD_VSYNCStateGet();
}

// *****************************************************************************
/*
  Function: DRV_GFX_LCC_Open(uint8_t instance)

  Summary:
    opens an instance of the graphics controller

  Description:
    none

  Input:
    instance of the driver

  Output:
    1 - driver not initialied
    2 - instance doesn't exist
    3 - instance already open
    instance to driver when successful
*/
DRV_GFX_HANDLE DRV_GFX_LCC_Open( const SYS_MODULE_INDEX index,
                             const DRV_IO_INTENT intent )
{
    DRV_GFX_CLIENT_OBJ * client = (DRV_GFX_CLIENT_OBJ *)DRV_HANDLE_INVALID;

    /* Check if the specified driver index is in valid range */
    if(index >= GFX_CONFIG_NUMBER_OF_MODULES)
    {
    }
    /* Check if instance object is ready*/
    else if(drvLCCObj.status != SYS_STATUS_READY)
    {
    }
    else if(intent != DRV_IO_INTENT_EXCLUSIVE)
    {
    }
    else if(drvLCCObj.nClients > 0)
    {
    }
    else
    {
		if(DRV_GFX_LCC_Start() != 0)
            return DRV_GFX_HANDLE_INVALID; // currently single context only
			
        client = &drvLCCClients;

        client->inUse = true;
        client->drvObj = &drvLCCObj;

        /* Increment the client number for the specific driver instance*/
        drvLCCObj.nClients++;
    }

    /* Return invalid handle */
    return ((DRV_HANDLE)client);
}

// *****************************************************************************
/* Function:
    void DRV_GFX_LCC_Close( DRV_HANDLE handle )

  Summary:
    closes an instance of the graphics controller

  Description:
    This is closes the instance of the driver specified by handle.
*/
void DRV_GFX_LCC_Close( DRV_HANDLE handle )
{
    /* Start of local variable */
    DRV_GFX_CLIENT_OBJ * client = (DRV_GFX_CLIENT_OBJ *)NULL;
    DRV_GFX_LCC_OBJ * drvObj = (DRV_GFX_LCC_OBJ *)NULL;
    /* End of local variable */

    /* Check if the handle is valid */
    if(handle == DRV_HANDLE_INVALID)
    {
//        SYS_DEBUG(0, "Bad Client Handle");
    }
    else
    {
        client = (DRV_GFX_CLIENT_OBJ *)handle;

        if(client->inUse)
        {
            client->inUse = false;
            drvObj = (DRV_GFX_LCC_OBJ *)client->drvObj;

            /* Remove this client from the driver client table */
            drvObj->nClients--;
        }
        else
        {
//            SYS_DEBUG(0, "Client Handle no inuse");
        }

    }
    return;
}

// *****************************************************************************
/*
  Function:
     void DRV_GFX_LCC_MaxXGet()

  Summary:
     Returns x extent of the display.

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    <code>

  Remarks:
*/
uint16_t DRV_GFX_LCC_MaxXGet()
{
    return drvLCCObj.maxX;
}

// *****************************************************************************
/*
  Function:
     void DRV_GFX_LCC_MaxYGet()

  Summary:
     Returns y extent of the display.

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    <code>

  Remarks:
*/
uint16_t DRV_GFX_LCC_MaxYGet()
{
    return drvLCCObj.maxY;
}

/*********************************************************************
  Function:
     void DRV_GFX_LCC_InterfaceSet( DRV_HANDLE handle, DRV_GFX_INTERFACE * interface )

  Summary:
    Returns the API of the graphics controller

  Description:
    none

  Return:

  *********************************************************************/
void DRV_GFX_LCC_InterfaceSet( DRV_HANDLE handle, DRV_GFX_INTERFACE * interface )
{
    interface->BarFill = DRV_GFX_LCC_BarFill;
    interface->PixelArrayPut = DRV_GFX_LCC_PixelArrayPut;
    interface->PixelArrayGet = DRV_GFX_LCC_PixelArrayGet;
    interface->PixelPut = DRV_GFX_LCC_PixelPut;
    interface->ColorSet = DRV_GFX_LCC_SetColor;
    interface->PageSet = DRV_GFX_LCC_SetPage;
<#if CONFIG_DRV_GFX_USE_LCC_PIP == true>
    interface->Layer = DRV_GFX_LCC_Layer;
</#if>
    interface->MaxXGet = DRV_GFX_LCC_MaxXGet;
    interface->MaxYGet = DRV_GFX_LCC_MaxYGet;
}

uint16_t DRV_GFX_LCC_SetPage(uint8_t pageType, uint8_t page)
{

        switch(pageType)
        {
            case ACTIVE_PAGE:
                drvLCCObj.initData->activePage = page;
<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
                activePageAddress = KVA0_TO_KVA1(frameBufferAddress);
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
            if ( drvLCCObj.initData->activePage == 0 )
                _ebiBaseAddress = (uint16_t*)SRAM_ADDR_CS0;
            else
                _ebiBaseAddress = (uint16_t*)SRAM_ADDR_CS1;
</#if>
                break;
            case VISUAL_PAGE:
                drvLCCObj.initData->visualPage = page;
<#if CONFIG_PIC32MX == false>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory">
<#if CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WQVGA or lower">
	            BSP_SRAM_A17StateSet(page&0x1);
	            BSP_SRAM_A18StateSet(page>>1&0x1);
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "HVGA">
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "VGA">
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WVGA">
</#if></#if></#if>
                break;
        }

        return(0);
}
// *****************************************************************************
/*
  Function: void DRV_GFX_LCC_SetColor(uint8_t instance, GFX_COLOR color)

  Summary: Sets the color for the driver instance

  Description:
  
  Output: none

*/

void DRV_GFX_LCC_SetColor(GFX_COLOR color)
{
  drvLCCObj.initData->color = color;
}

// *****************************************************************************
/*
  Function: void DRV_GFX_LCC_DisplayRefresh(void)

  Summary: State Machine driving H-Sync, V-Sync and DE timing

  Description:
  
  Output: none

*/

void DRV_GFX_LCC_DisplayRefresh(void)
{
<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory"
	&& CONFIG_DRV_GFX_LCC_REFRESH_STRATEGY == "Aggressive"
	&& CONFIG_DRV_GFX_USE_LCC_SCROLL == true>
    static uint16_t scrollPos,scrollState;
</#if>
<#if CONFIG_DRV_GFX_USE_LCC_PIP == true>
    static uint16_t remainingPixels=0;
    static uint8_t prevGraphicsState = BLANKING_PERIOD;
    static uint16_t pipLine = 0;    
    static uint32_t pixelAddress=0;
</#if>
<#if CONFIG_DRV_GFX_LCC_REFRESH_STRATEGY == "Conventional">
    typedef enum
	{
        HSYNC_FRONT_PORCH,
        HSYNC_PULSE,
        HSYNC_BACK_PORCH,
        HSYNC_DATA_ENABLE,
        HSYNC_DATA_ENABLE_OVERFLOW        
    } HSYNC_STATES;

    typedef enum
    {
        VSYNC_FRONT_PORCH,
        VSYNC_PULSE,
        VSYNC_BACK_PORCH,
        VSYNC_BLANK        
    } VSYNC_STATES;

    static int16_t line = 0;
<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
    static uint32_t offset = 0;
</#if>
<#if CONFIG_DRV_GFX_USE_LCC_PALETTE == true>
    static GFX_COLOR *point;
    int i;
</#if>
    static uint16_t pixels = 0;
    static uint32_t hSyncs = 0;
    
    static uint32_t vsyncPeriod = DISP_VER_FRONT_PORCH + DISP_VER_RESOLUTION + DISP_VER_BACK_PORCH;
    static uint32_t vsyncPulseDown = 0;
    static uint32_t vsyncPulseUp = 0;
    static uint32_t vsyncEnd = 0;

    static HSYNC_STATES hsyncState = HSYNC_FRONT_PORCH;
    static VSYNC_STATES vsyncState = VSYNC_BLANK;

<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory">
<#if CONFIG_DRV_GFX_LCC_DMA_WRITE_STRATEGY == "Draw Count Limited">
    DrawCount = 0;
</#if></#if>
    switch(vsyncState)
    {
        case VSYNC_FRONT_PORCH:
            if (hSyncs > vsyncPulseDown)
            {
<#if CONFIG_DRV_GFX_DISP_VSYNC_NEGATIVE_POLARITY == true>
                BSP_LCD_VSYNCOn();
<#else>
                BSP_LCD_VSYNCOff();
</#if>
                vsyncPulseUp = hSyncs + DISP_VER_PULSE_WIDTH;
                vsyncState = VSYNC_PULSE;

				if(drvLCCObj.vsync_cb != NULL)
					drvLCCObj.vsync_cb();

                allowFrameUpdate = false;
                line = 0;
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_PIC32MX == true>
			#ifdef BSP_SRAM_A15StateSet
                BSP_SRAM_A15StateSet(0&0x1);
			#endif
                BSP_SRAM_A16StateSet(0&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory">
<#if CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WQVGA or lower">
                BSP_SRAM_A16StateSet(0&0x1);
                overflowcount = 0;
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "HVGA">
                BSP_SRAM_A16StateSet(drvLCCObj.initData->visualPage&0x1);
                BSP_SRAM_A17StateSet(drvLCCObj.initData->visualPage&0x1);
                BSP_SRAM_A18StateSet(0&0x1);
				if ( drvLCCObj.initData->visualPage == 1 )
					overflowcount = 3;
				else
					overflowcount = 0;
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "VGA">
                BSP_SRAM_A16StateSet(drvLCCObj.initData->visualPage&0x1);
                BSP_SRAM_A17StateSet(0&0x1);
                BSP_SRAM_A18StateSet(drvLCCObj.initData->visualPage&0x1);
				if ( drvLCCObj.initData->visualPage == 1 )
					overflowcount = 5;
				else
					overflowcount = 0;
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WVGA">
                BSP_SRAM_A16StateSet(0&0x1);
                BSP_SRAM_A17StateSet(drvLCCObj.initData->visualPage&0x1);
                BSP_SRAM_A18StateSet(drvLCCObj.initData->visualPage&0x1);
				if ( drvLCCObj.initData->visualPage == 1 )
					overflowcount = 6;
				else
					overflowcount = 0;
</#if></#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_19 == true>

                BSP_SRAM_A19StateSet(0&0x1);
</#if>
            }
            break;
        case VSYNC_PULSE:
            if (hSyncs >= vsyncPulseUp)
            {
<#if CONFIG_DRV_GFX_DISP_VSYNC_NEGATIVE_POLARITY == true>
                BSP_LCD_VSYNCOff();
<#else>
                BSP_LCD_VSYNCOn();
</#if>
                vsyncEnd = hSyncs + DISP_VER_BACK_PORCH;
                vsyncState = VSYNC_BACK_PORCH;
            }
            break;
        case VSYNC_BACK_PORCH:
            if (hSyncs >= vsyncEnd)
            {
                vsyncState = VSYNC_BLANK;
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory" && CONFIG_PIC32MX == true>
				DCH1CSIZ = DISP_HOR_RESOLUTION;
</#if>
            }
            break;
        case VSYNC_BLANK:
            break;
    }

    switch (hsyncState)
    {
        case HSYNC_FRONT_PORCH:
<#if CONFIG_DRV_GFX_DISP_USE_DATA_ENABLE == true>
<#if CONFIG_DRV_GFX_DISP_DATA_ENABLE_POSITIVE_POLARITY == true>
            BSP_LCD_DEOff();
<#else>
            BSP_LCD_DEOn();
</#if></#if>
            hsyncState = HSYNC_PULSE;
            allowLineUpdate = true;
			if (DISP_HOR_FRONT_PORCH > 0)
			{
	            pixels = DISP_HOR_FRONT_PORCH;
	            break;
			}
        case HSYNC_PULSE:
<#if CONFIG_DRV_GFX_DISP_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOn();
<#else>
            BSP_LCD_HSYNCOff();
</#if>
            if (hSyncs >= vsyncPeriod)
            {
				vsyncPeriod = hSyncs + DISP_VER_PULSE_WIDTH + DISP_VER_FRONT_PORCH + DISP_VER_RESOLUTION + DISP_VER_BACK_PORCH;
                vsyncPulseDown = hSyncs + DISP_VER_FRONT_PORCH;
                vsyncState = VSYNC_FRONT_PORCH;
                allowFrameUpdate = true;
            }
            hSyncs++; 
            pixels = DISP_HOR_PULSE_WIDTH;
            hsyncState = HSYNC_BACK_PORCH;  
            break;
        case HSYNC_BACK_PORCH:
<#if CONFIG_DRV_GFX_DISP_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOff();
<#else>
            BSP_LCD_HSYNCOn();
</#if>
            hsyncState = HSYNC_DATA_ENABLE; 
            allowLineUpdate = false;
			if (DISP_HOR_BACK_PORCH > 0)
			{
				pixels = DISP_HOR_BACK_PORCH;
				break;
			}
        case HSYNC_DATA_ENABLE:
            if (vsyncState == VSYNC_BLANK)
            {
<#if CONFIG_DRV_GFX_DISP_USE_DATA_ENABLE == true>
<#if CONFIG_DRV_GFX_DISP_DATA_ENABLE_POSITIVE_POLARITY == true>
                BSP_LCD_DEOn();
<#else>
                BSP_LCD_DEOff();
</#if></#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory" && CONFIG_PIC32MX == true>
				DCH1CSIZ = 1;
</#if>

<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
                offset = drvLCCObj.initData->visualPage *
                         DISP_HOR_RESOLUTION * DISP_VER_RESOLUTION +
                         line++ * DISP_HOR_RESOLUTION;

<#if CONFIG_DRV_GFX_USE_LCC_PALETTE == true>
                LUTAddress = (GFX_COLOR*)&frameBufferAddress[offset];
                point = (GFX_COLOR*)LUTAddress;
                for (i=0; i<DISP_HOR_RESOLUTION; i++)  /*Do color LUT Here. Each line is 240 bytes*/
                {
                    frameLineBuf[i] = LUT[*point++];
                }
                PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint32_t)&frameLineBuf );
<#else>
                PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint32_t)&frameBufferAddress[offset]);
</#if>
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
                PMADDR = ((PMP_ADDRESS_LINES) & ((line++) * DISP_HOR_RESOLUTION));
                if((PMADDR_OVERFLOW - PMADDR) < (DISP_HOR_RESOLUTION))       
				{   
					hsyncState = HSYNC_DATA_ENABLE_OVERFLOW;      //Do Overflow routine
					PLIB_DMA_ChannelXDestinationSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint16_t)(((PMADDR_OVERFLOW)- PMADDR)<<1));
					PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
					PLIB_DMA_ChannelXEnable(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX);
                    pixels = DISP_HOR_RESOLUTION - ((PMADDR_OVERFLOW)- PMADDR);
					return;
				}
</#if>
			}
            pixels = DISP_HOR_RESOLUTION;
            hsyncState = HSYNC_FRONT_PORCH;
			break;
        case HSYNC_DATA_ENABLE_OVERFLOW:
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_PIC32MX == true>
		#ifdef BSP_SRAM_A15StateSet
            BSP_SRAM_A15StateSet(overflowcount&0x1);
            BSP_SRAM_A16StateSet(overflowcount>>1&0x1);
		#else
            BSP_SRAM_A16StateSet(overflowcount>>1&0x1);
        #endif
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WQVGA or lower">
            BSP_SRAM_A16StateSet(++overflowcount&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "HVGA">
            BSP_SRAM_A16StateSet(++overflowcount&0x1);
            BSP_SRAM_A17StateSet(overflowcount>>1&0x1);
            BSP_SRAM_A18StateSet(overflowcount>>2&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "VGA">
            BSP_SRAM_A16StateSet(++overflowcount&0x1);
            BSP_SRAM_A17StateSet(overflowcount>>1&0x1);
            BSP_SRAM_A18StateSet(overflowcount>>2&0x1);
            BSP_SRAM_A19StateSet(overflowcount>>3&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WVGA">
            BSP_SRAM_A16StateSet(++overflowcount&0x1);
            BSP_SRAM_A17StateSet(overflowcount>>1&0x1);
            BSP_SRAM_A18StateSet(overflowcount>>2&0x1);
            BSP_SRAM_A19StateSet(overflowcount>>3&0x1);
</#if>
            hsyncState = HSYNC_FRONT_PORCH;
            break;
    }

<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
    PLIB_DMA_ChannelXSourceSizeSet(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX,(uint16_t)(pixels<<1));
    PLIB_DMA_ChannelXCellSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint16_t)(pixels<<1));
    PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
    SYS_DMA_ChannelForceStart(dmaHandle);
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
    PLIB_DMA_ChannelXDestinationSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint16_t)(pixels<<1));
    PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
    SYS_DMA_ChannelEnable(dmaHandle);
</#if>
<#elseif CONFIG_DRV_GFX_LCC_REFRESH_STRATEGY == "Aggressive">
    static uint8_t GraphicsState = ACTIVE_PERIOD;
    static uint16_t remaining = 0;
    static short line = 0;
<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
    uint32_t offset = 0;
<#if CONFIG_DRV_GFX_USE_LCC_PALETTE == true>
    static GFX_COLOR *point;
    int i;
</#if>
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
<#if CONFIG_DRV_GFX_LCC_DMA_WRITE_STRATEGY == "Draw Count Limited">
    DrawCount = 0;
</#if>
<#if CONFIG_DEVICE == "PIC32MX460F512L" || CONFIG_DEVICE == "PIC32MX360F512L">
        static uint16_t dmatransfersremaining = 0;

        if(dmatransfersremaining != 0)
        {

            if(dmatransfersremaining > 254)
             {
                   //Setup DMA Transfer
                   PLIB_DMA_ChannelXDestinationSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, 254);
                   dmatransfersremaining -= 254;
             }
             else
             {
                   //Setup DMA Transfer
                   PLIB_DMA_ChannelXDestinationSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, dmatransfersremaining);
                   dmatransfersremaining -= dmatransfersremaining;
             }

         PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
         PLIB_DMA_ChannelXEnable(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX);
         return;
        }
</#if></#if>

    switch(GraphicsState)
    {
        case ACTIVE_PERIOD:
<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
            remaining = DISP_HOR_RESOLUTION << PMP_DATA_LENGTH;
            GraphicsState = BLANKING_PERIOD;
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
            remaining = DISP_HOR_RESOLUTION;
            GraphicsState = BLANKING_PERIOD;
<#if CONFIG_PIC32MX == true>
            DCH1CSIZ = DISP_HOR_RESOLUTION;
</#if></#if>

<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory" || 
        CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WVGA">
<#if CONFIG_DRV_GFX_DISP_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOff();
<#else>
            BSP_LCD_HSYNCOn();
</#if></#if>
            if(line >= 0)
            {
<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
                offset = drvLCCObj.initData->visualPage *
                         DISP_HOR_RESOLUTION * DISP_VER_RESOLUTION +
                         line * DISP_HOR_RESOLUTION;

<#if CONFIG_DRV_GFX_USE_LCC_PALETTE == true>
                LUTAddress = (GFX_COLOR*)&frameBufferAddress[offset];
                point = (GFX_COLOR*)LUTAddress;
                for (i=0; i<DISP_HOR_RESOLUTION; i++)  /*Do color LUT Here. Each line is 240 bytes*/
                {
                    frameLineBuf[i] = LUT[*point++];
                }
                PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint32_t)&frameLineBuf );
<#else>
                PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint32_t)&frameBufferAddress[offset] );
</#if>
                if(line == (DISP_VER_RESOLUTION))
                {
					// notify registered callback that the display is in vsync mode
					if(drvLCCObj.vsync_cb != NULL)
						drvLCCObj.vsync_cb();
						
<#if CONFIG_DRV_GFX_DISP_VSYNC_NEGATIVE_POLARITY == true>
                    BSP_LCD_VSYNCOn();
<#else>
                    BSP_LCD_VSYNCOff();
</#if>
                    line = -(VER_BLANK);
<#if CONFIG_DRV_GFX_USE_LCC_PIP == true>
                    pipLine = 0;
</#if>
<#if CONFIG_DRV_GFX_USE_LCC_SCROLL == true>
                    scrollState = scroll;
                    scrollPos = scrollLine;
</#if>
                }
                else
                {
<#if CONFIG_DRV_GFX_DISP_VSYNC_NEGATIVE_POLARITY == true>
                    BSP_LCD_VSYNCOff();
<#else>
                    BSP_LCD_VSYNCOn();
</#if>
<#if CONFIG_DRV_GFX_DISP_USE_DATA_ENABLE == true>
<#if CONFIG_DRV_GFX_DISP_DATA_ENABLE_POSITIVE_POLARITY == true>
                    BSP_LCD_DEOn();
<#else>
                    BSP_LCD_DEOff();
</#if></#if>

<#if CONFIG_DRV_GFX_USE_LCC_PIP == true>
                    if(((line) >= PipStartT)&&((line) <= (PipStartT + PipVLength))&&(PipVLength != 0))
                    {

                        remaining = PipStartL;
                        GraphicsState = PIP;

                        if(!PipStartL) //Draw PIP Line
                        {
            case PIP:
                            offset = GFXPIPPage * DISP_HOR_RESOLUTION * DISP_VER_RESOLUTION +
                                     ( PipY + pipLine )  *
                                     DISP_HOR_RESOLUTION +
                                     PipX;

                            PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint32_t)&frameBufferAddress[offset]);
                            pipLine++;
                            remaining = PipHLength;
                            remainingPixels = (DISP_HOR_RESOLUTION)-PipHLength-PipStartL;
                            pixelAddress = PipHLength-PipStartL;
                            GraphicsState = FINISH_LINE;
                        }
                    }
</#if>

<#if CONFIG_DRV_GFX_USE_LCC_SCROLL == true>
                    if(scrollState >0)
                    {
                        switch(scroll)
                        {
                            case 1:             //Up
                            case 2:             //Down
                                if((line) < scrollPos)
                                {
                                    pixelAddress = (DISP_VER_RESOLUTION-1)-(scrollPos-(line));
                                    offset = drvLCCObj.initData->visualPage *
                                             DISP_HOR_RESOLUTION *
                                             DISP_VER_RESOLUTION + pixelAddress;
                                    PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint32_t)&frameBufferAddress[offset]);
                                }
                                else
                                {
                                    pixelAddress = ((line)-scrollPos);
                                    offset = scrollPage * DISP_HOR_RESOLUTION *
                                             DISP_VER_RESOLUTION + pixelAddress;
                                    PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint32_t)&frameBufferAddress[offset]);
                                }
                                break;

                            case 3://Left
                            case 4://Right
                                remainingPixels = (DISP_HOR_RESOLUTION-1)-scrollPos;
                                pixelAddress = (line+1);

                                if(scroll == 3)
                                {
                                    offset = drvLCCObj.initData->visualPage *
                                             DISP_HOR_RESOLUTION *
                                             DISP_VER_RESOLUTION +
                                             pixelAddress * DISP_HOR_RESOLUTION +
                                             remainingPixels;

                                    PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint32_t)&frameBufferAddress[offset]);
                                }
                                else
                                {
                                    offset =
                                      scrollPage *
                                      DISP_HOR_RESOLUTION *
                                      DISP_VER_RESOLUTION +
                                      DISP_HOR_RESOLUTION *
                                      pixelAddress        +
                                      remainingPixels;

                                    PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint32_t)&frameBufferAddress[offset]);
                                }

                                remaining = scrollPos;
                                remainingPixels = (DISP_HOR_RESOLUTION - scrollLine);
                                pixelAddress = 0;
                                GraphicsState = FINISH_LINE;
                        }
                    
                        if((scroll>2) && (GraphicsState != FINISH_LINE))
                        {
        case FINISH_LINE:       //Finish Line Render

                            if(scroll == 3)
                            {
                                offset = scrollPage *
                                         DISP_HOR_RESOLUTION *
                                         DISP_VER_RESOLUTION +
                                         DISP_HOR_RESOLUTION *
                                         line                +
                                         pixelAddress;

                                PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint32_t)&frameBufferAddress[offset]);
                            }
                            else
                            {
                                offset =
                                    drvLCCObj.initData->visualPage *
                                    DISP_HOR_RESOLUTION *
                                    DISP_VER_RESOLUTION +
                                    DISP_HOR_RESOLUTION *
                                    line                +
                                    pixelAddress;

                                PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint32_t)&frameBufferAddress[offset]);
                            }
                            remaining = remainingPixels;
                            GraphicsState = BLANKING_PERIOD;
                        }
                    }
</#if>
                }
           }

           PLIB_DMA_ChannelXSourceSizeSet(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX,remaining);
<#if CONFIG_DRV_GFX_LCC_DMA_BUFFER_SIZE_MODE == "Large Buffer">
           PLIB_DMA_ChannelXCellSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, remaining);
</#if>
           break;
            
        case BLANKING_PERIOD:
<#if CONFIG_DRV_GFX_DISP_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOn();
<#else>
            BSP_LCD_HSYNCOff();
</#if>
<#if CONFIG_DRV_GFX_DISP_USE_DATA_ENABLE == true>
<#if CONFIG_DRV_GFX_DISP_DATA_ENABLE_POSITIVE_POLARITY == true>
            BSP_LCD_DEOff();
<#else>
            BSP_LCD_DEOn();
</#if></#if>
            PLIB_DMA_ChannelXSourceSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (HBackPorch << PMP_DATA_LENGTH));
            GraphicsState = ACTIVE_PERIOD;
            line++;

<#if CONFIG_DRV_GFX_LCC_DMA_BUFFER_SIZE_MODE == "Large Buffer">
            PLIB_DMA_ChannelXCellSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (HBackPorch<<1));
</#if>
        }

    PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
   
<#if CONFIG_DRV_GFX_LCC_DMA_BUFFER_SIZE_MODE == "Large Buffer">
    SYS_DMA_ChannelForceStart(dmaHandle);
<#else>
    	SYS_DMA_ChannelEnable(dmaHandle);
</#if>
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
                PMADDR = ((PMP_ADDRESS_LINES) & ((line) * DISP_HOR_RESOLUTION));

                if((line) == (DISP_VER_RESOLUTION))
                {
					// notify registered callback that the display is in vsync mode
					if(drvLCCObj.vsync_cb != NULL)
						drvLCCObj.vsync_cb();
				
<#if CONFIG_DRV_GFX_DISP_VSYNC_NEGATIVE_POLARITY == true>
                    BSP_LCD_VSYNCOn();
<#else>
                    BSP_LCD_VSYNCOff();
</#if>
                    line = (-VER_BLANK);
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_PIC32MX == true>
				#ifdef BSP_SRAM_A15StateSet
                    BSP_SRAM_A15StateSet(0&0x1);
				#endif
                    BSP_SRAM_A16StateSet(0&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory">
<#if CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WQVGA or lower">
                BSP_SRAM_A16StateSet(0&0x1);
                overflowcount = 0;
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "HVGA">
                BSP_SRAM_A16StateSet(drvLCCObj.initData->visualPage&0x1);
                BSP_SRAM_A17StateSet(drvLCCObj.initData->visualPage&0x1);
                BSP_SRAM_A18StateSet(0&0x1);
				if ( drvLCCObj.initData->visualPage == 1 )
					overflowcount = 3;
				else
					overflowcount = 0;
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "VGA">
                BSP_SRAM_A16StateSet(drvLCCObj.initData->visualPage&0x1);
                BSP_SRAM_A17StateSet(0&0x1);
                BSP_SRAM_A18StateSet(drvLCCObj.initData->visualPage&0x1);
				if ( drvLCCObj.initData->visualPage == 1 )
					overflowcount = 5;
				else
					overflowcount = 0;
<#elseif CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WVGA">
                BSP_SRAM_A16StateSet(0&0x1);
                BSP_SRAM_A17StateSet(drvLCCObj.initData->visualPage&0x1);
                BSP_SRAM_A18StateSet(drvLCCObj.initData->visualPage&0x1);
				if ( drvLCCObj.initData->visualPage == 1 )
					overflowcount = 6;
				else
					overflowcount = 0;
</#if></#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_19 == true>

					BSP_SRAM_A19StateSet(0&0x1);
</#if>
<#if CONFIG_DRV_GFX_USE_LCC_PIP == true>
                    pipLine = 0;
</#if>
               }
               else
               {
<#if CONFIG_DRV_GFX_DISP_USE_DATA_ENABLE == true>
<#if CONFIG_DRV_GFX_DISP_DATA_ENABLE_POSITIVE_POLARITY == true>
                    BSP_LCD_DEOn();
<#else>
                    BSP_LCD_DEOff();
</#if></#if>
<#if CONFIG_DRV_GFX_USE_LCC_PIP == true>
                   if((line >= PipStartT)&&(line <= (PipStartT + PipVLength))&&(PipVLength != 0))
                   {
                       overflowcount = ((line) * DISP_HOR_RESOLUTION)/PMADDR_OVERFLOW;
                       GraphicsState = PIP;

                       if(!PipStartL)
                       {
                           //Draw PIP Line
      case PIP:            
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_17 == true>
						   BSP_SRAM_A17StateSet(GFXPIPPage&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_18 == true>
                           BSP_SRAM_A18StateSet((GFXPIPPage>>1)&0x1);
</#if>
                           pixelAddress = (uint32_t)(((PipY+pipLine++)*(DISP_HOR_RESOLUTION))+(PipX));

                           PMADDR = pixelAddress&(PMP_ADDRESS_LINES);

                           overflowcount = pixelAddress/PMADDR_OVERFLOW;
                           remaining = PipHLength;

                           remainingPixels = DISP_HOR_RESOLUTION - remaining - PipStartL;
                           pixelAddress = ((line*DISP_HOR_RESOLUTION) + remaining + PipStartL);
 
                           GraphicsState = FINISH_LINE;
                        }
                       else
                       {
                           remaining = PipStartL;
                       }

<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory">
<#if CONFIG_PIC32MX == true>
				#ifdef BSP_SRAM_A15StateSet
					BSP_SRAM_A15StateSet(overflowcount&0x1);          //count holds the additional address line count
                    BSP_SRAM_A16StateSet((overflowcount>>1)&0x1);
				#else
                    BSP_SRAM_A16StateSet(overflowcount&0x1);	
				#endif
<#else>
				    BSP_SRAM_A16StateSet(overflowcount&0x1);
</#if></#if>
                 }

                 if(scroll > 0)
                 {

                     switch(scroll)
                     {
                         case 1:             //Up
                         case 2:             //Down
                            if(line < scrollLine)
                            {
                                pixelAddress = ((drvLCCObj.initData->verticalResolution-1)-(scrollLine-line)) * DISP_HOR_RESOLUTION;
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_17 == true>
                                BSP_SRAM_A17StateSet(drvLCCObj.initData->visualPage&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_18 == true>
                                BSP_SRAM_A18StateSet((drvLCCObj.initData->visualPage>>1)&0x1);
</#if>
                            }
                            else
                            {
                                pixelAddress = (line-scrollLine) * DISP_HOR_RESOLUTION;
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_17 == true>
                                BSP_SRAM_A17StateSet(scrollPage&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_18 == true>
                                BSP_SRAM_A18StateSet((scrollPage>>1)&0x1);
</#if>
                            }

                            PMADDR = (pixelAddress)&(PMP_ADDRESS_LINES);
                            overflowcount = pixelAddress/PMADDR_OVERFLOW;
                            break;

                        case 3://Left
                        case 4://Right
                            pixelAddress = ((line+1) * DISP_HOR_RESOLUTION) + ((drvLCCObj.initData->horizontalResolution-1)-scrollLine);
                   
                            PMADDR = (pixelAddress)&(PMP_ADDRESS_LINES);
                            overflowcount = pixelAddress/PMADDR_OVERFLOW;

                            if(scroll == 3)
                            {
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_17 == true>
                                BSP_SRAM_A17StateSet(drvLCCObj.initData->visualPage&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_18 == true>
                                BSP_SRAM_A18StateSet((drvLCCObj.initData->visualPage>>1)&0x1);
</#if>
                            }
                            else
                            {
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_17 == true>
                                BSP_SRAM_A17StateSet(scrollPage&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_18 == true>
                                BSP_SRAM_A18StateSet((scrollPage>>1)&0x1);
</#if>
                            }

                            remaining = scrollLine;
                            remainingPixels = DISP_HOR_RESOLUTION - scrollLine;
                            pixelAddress = (line) * DISP_HOR_RESOLUTION;
                            GraphicsState = FINISH_LINE;
                            break;
                    }

                  if((scroll > 2) && (GraphicsState != FINISH_LINE))
                  {
        case FINISH_LINE:            //Finish Line Render
                      remaining = remainingPixels;
                   
                      PMADDR = (pixelAddress)&(PMP_ADDRESS_LINES);
                      overflowcount = pixelAddress/PMADDR_OVERFLOW;

                      if(scroll == 3)
                      {
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_17 == true>
                          BSP_SRAM_A17StateSet(scrollPage&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_18 == true>
                          BSP_SRAM_A18StateSet((scrollPage>>1)&0x1);
</#if>
                      }
                      else
                      {
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_17 == true>
                          BSP_SRAM_A17StateSet(drvLCCObj.initData->visualPage&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_USE_DRV_GFX_LCC_EXTERNAL_MEMORY_ADDRESS_18 == true>
                          BSP_SRAM_A18StateSet((drvLCCObj.initData->visualPage>>1)&0x1);
</#if>
                      }
 
                      GraphicsState = BLANKING_PERIOD;
                  }
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory">
<#if CONFIG_PIC32MX == true>
				#ifdef BSP_SRAM_A15StateSet
                  BSP_SRAM_A15StateSet(++overflowcount&0x1);          //count holds the additional address line count
                  BSP_SRAM_A16StateSet((overflowcount>>1)&0x1);
				#else
                  BSP_SRAM_A16StateSet(overflowcount&0x1);
				#endif
<#else>
                  BSP_SRAM_A16StateSet(overflowcount&0x1);
</#if></#if>
              }

              prevGraphicsState = GraphicsState;
</#if>

<#if CONFIG_DRV_GFX_USE_LCC_PIP == true>
                    if((PMADDR_OVERFLOW - PMADDR) < (remaining))       
                    {   
                      GraphicsState = OVERFLOW;      //Do Overflow routine
                      PLIB_DMA_ChannelXDestinationSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint16_t)(((PMADDR_OVERFLOW)- PMADDR)<<1));
                      PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
                      PLIB_DMA_ChannelXEnable(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX);
                      remaining = remaining - ((PMADDR_OVERFLOW)- PMADDR);
                      return;
                    }           
<#else>
                    if((PMADDR_OVERFLOW - PMADDR) < (DISP_HOR_RESOLUTION))       
                    {   
                      GraphicsState = OVERFLOW;      //Do Overflow routine
                      PLIB_DMA_ChannelXDestinationSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint16_t)(((PMADDR_OVERFLOW)- PMADDR)<<1));
                      PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
                      PLIB_DMA_ChannelXEnable(DMA_ID_0,DRV_GFX_LCC_DMA_CHANNEL_INDEX);
                      remaining = DISP_HOR_RESOLUTION - ((PMADDR_OVERFLOW)- PMADDR);
                      return;
                    }           
</#if>
                }
            }
            break;

       case OVERFLOW: //Adjust the address lines that aren't part of PMP
<#if CONFIG_DRV_GFX_USE_LCC_PIP == true>
            GraphicsState = prevGraphicsState;     //goto Front Porch
<#else>
            GraphicsState = BLANKING_PERIOD;     //goto Front Porch
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_PIC32MX == true>
		#ifdef BSP_SRAM_A15StateSet
            BSP_SRAM_A15StateSet(++overflowcount&0x1);          //count holds the additional address line count
            BSP_SRAM_A16StateSet((overflowcount>>1)&0x1);
		#else
            BSP_SRAM_A16StateSet(overflowcount&0x1);
		#endif
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WQVGA or lower">
            BSP_SRAM_A16StateSet(++overflowcount&0x1);
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "HVGA">
            BSP_SRAM_A16StateSet(++overflowcount&0x1);
			BSP_SRAM_A17StateSet((overflowcount>>1)&0x1);
			BSP_SRAM_A18StateSet((overflowcount>>2)&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "VGA">
            BSP_SRAM_A16StateSet(++overflowcount&0x1);
			BSP_SRAM_A17StateSet((overflowcount>>1)&0x1);
			BSP_SRAM_A18StateSet((overflowcount>>2)&0x1);
			BSP_SRAM_A19StateSet((overflowcount>>3)&0x1);
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory"
&& CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL == "WVGA">
            BSP_SRAM_A16StateSet(++overflowcount&0x1);
			BSP_SRAM_A17StateSet((overflowcount>>1)&0x1);
			BSP_SRAM_A18StateSet((overflowcount>>2)&0x1);
			BSP_SRAM_A19StateSet((overflowcount>>3)&0x1);
</#if>
</#if>
			break;

        case BLANKING_PERIOD:   //Front Porch then Back Porch Start 
<#if CONFIG_DRV_GFX_DISP_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOn();
<#else>
            BSP_LCD_HSYNCOff();
</#if>
<#if CONFIG_DRV_GFX_DISP_USE_DATA_ENABLE == true>
<#if CONFIG_DRV_GFX_DISP_DATA_ENABLE_POSITIVE_POLARITY == true>
            BSP_LCD_DEOff();
<#else>
            BSP_LCD_DEOn();
</#if></#if>
<#if CONFIG_DRV_GFX_LCC_PIXEL_SUPPORT_LEVEL != "WVGA">
            GraphicsState = PMDIN;
            while(PMMODEbits.BUSY == 1);
<#if CONFIG_DRV_GFX_DISP_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOff();
<#else>
            BSP_LCD_HSYNCOn();
</#if></#if>
<#if CONFIG_DRV_GFX_DISP_VSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_VSYNCOff();
<#else>
            BSP_LCD_VSYNCOn();
</#if>
            //Setup DMA Back Porch
            remaining = HBackPorch;
            GraphicsState = ACTIVE_PERIOD;   
            line++;
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory" && CONFIG_PIC32MX == true>
            DCH1CSIZ = 1;
</#if>
            break;

        default:
            break;
    }

    //Setup DMA Transfer
<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory">
<#if CONFIG_DEVICE == "PIC32MX460F512L" || CONFIG_DEVICE == "PIC32MX360F512L">
    if((remaining) > 254)
    {
        dmatransfersremaining = (remaining ) - 254;
        remaining = (254 >> 1);
    }
</#if></#if>
    PLIB_DMA_ChannelXDestinationSizeSet(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, (uint16_t) (remaining << 1));
    PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, DRV_GFX_LCC_DMA_CHANNEL_INDEX, DMA_INT_BLOCK_TRANSFER_COMPLETE);
    SYS_DMA_ChannelEnable(dmaHandle);
</#if>
</#if>
}
  
// *****************************************************************************
/*
  Function: void DRV_GFX_LCC_PixelPut(uint16_t x, uint16_t y)

  Summary:
    outputs one pixel into the frame buffer at the x,y coordinate given

  Description:
    none

  Input:
        x,y - pixel coordinates
  Output:
    1 - call not successful (lcc driver busy)
    0 - call successful
*/
void DRV_GFX_LCC_PixelPut(uint16_t x, uint16_t y)
{
<#if CONFIG_DRV_GFX_DISP_ORIENTATION == "270">
    uint16_t tempx=x;
    x = (DISP_HOR_RESOLUTION-1) - y;
    y = tempx; 
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "90">
    uint16_t tempy=y;
    y = (DISP_VER_RESOLUTION-1) - x;
    x = tempy; 
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "180">
    x = (DISP_HOR_RESOLUTION-1) - x;
    y = (DISP_VER_RESOLUTION-1) - y;
</#if>
<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
    GFX_COLOR *point = (GFX_COLOR*)activePageAddress + x + y *(DISP_HOR_RESOLUTION);
    *point = drvLCCObj.initData->color;
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
    _framePoint = (uint16_t*)_ebiBaseAddress + (((y)*(DISP_HOR_RESOLUTION))+(x));
<#elseif CONFIG_PIC32MX == true>
    _frameAddress = (uint32_t)(((y)*(DISP_HOR_RESOLUTION))+(x));
</#if>
    LCCSaveData();

<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
    *_framePoint++ = drvLCCObj.initData->color;
<#elseif CONFIG_PIC32MX == true>
    PMDIN = drvLCCObj.initData->color;
</#if>
    LCCRecoverData();
</#if>
}  

// *****************************************************************************
/*
  Function: void DRV_GFX_LCC_BarFill(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)

  Summary:
    outputs one pixel into the frame buffer at the x,y coordinate given

  Description:
    none

  Input:
        left,top - pixel coordinates
        right, bottom - pixel coordinates

  Output:
          1 - call not successful (lcc driver busy)
          0 - call successful
*/
void  DRV_GFX_LCC_BarFill(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)
{
<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
    static GFX_COLOR *point;
    uint16_t tempy = top;
    uint16_t tempx = left;

    for(tempy = top; tempy <= bottom; tempy++)
    {
<#if CONFIG_DRV_GFX_DISP_ORIENTATION == "270">
        left = (DISP_HOR_RESOLUTION-1) - tempy;
        top = tempx;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "90">
        top = (DISP_VER_RESOLUTION-1) - tempx;
        left = tempy;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "180">
        left = (DISP_HOR_RESOLUTION-1) - tempx;
        top = (DISP_VER_RESOLUTION-1) - tempy;
<#else>
        left = tempx;
        top = tempy;
</#if>
        point = (GFX_COLOR*)activePageAddress + left + top *(DISP_HOR_RESOLUTION);

        for(left = tempx; left <= right; left++)
        {
            *point = drvLCCObj.initData->color;
<#if CONFIG_DRV_GFX_DISP_ORIENTATION == "90">
            point -= DISP_HOR_RESOLUTION;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "270">
            point += DISP_HOR_RESOLUTION;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "180">
            point--;
<#else>
            point++;
</#if>
        }
	}
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
<#if CONFIG_DRV_GFX_DISP_ORIENTATION == "270" || CONFIG_DRV_GFX_DISP_ORIENTATION == "90">
    uint16_t lineCount = right-left + 1;//hchange;
    uint16_t count = bottom - top + 1;//vchange;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "180" || CONFIG_DRV_GFX_DISP_ORIENTATION == "0">
    uint16_t lineCount = bottom - top + 1;//vchange;
    uint16_t count = right - left + 1;//hchange;
</#if>
<#if CONFIG_DRV_GFX_DISP_ORIENTATION == "270">
    uint16_t tempx = left;
    left = (DISP_HOR_RESOLUTION) - top - count;
    top = tempx;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "90">
    uint16_t tempy = top;
    top = (DISP_VER_RESOLUTION) - left - lineCount;
    left = tempy;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "180">
    top = (DISP_VER_RESOLUTION) - top - lineCount;
    left =  (DISP_HOR_RESOLUTION) - left - count;
</#if>
    _frameAddress = (uint32_t)(((top)*(DISP_HOR_RESOLUTION))+(left));

    while(lineCount)
    {
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
       _framePoint = (uint16_t*)_ebiBaseAddress + (_frameAddress);
</#if>
       for(_pixelRendercount = count; _pixelRendercount > 0; _pixelRendercount--)
       {
          /*Suspend DMA Trigger*/
          LCCSaveData();
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
		if(_pixelRendercount > 8) //Write 8 pixels at once
		{
			*_framePoint++ = drvLCCObj.initData->color;
			*_framePoint++ = drvLCCObj.initData->color;
			*_framePoint++ = drvLCCObj.initData->color;
			*_framePoint++ = drvLCCObj.initData->color;
			*_framePoint++ = drvLCCObj.initData->color;
			*_framePoint++ = drvLCCObj.initData->color;
			*_framePoint++ = drvLCCObj.initData->color;
			_pixelRendercount -= 7;
			_frameAddress += 7;
		}

		*_framePoint++ = drvLCCObj.initData->color;
<#elseif CONFIG_PIC32MX == true>
    if((_pixelRendercount > 8) && (PMADDR < PMP_ADDRESS_LINES-8)) //Write 8 pixels at once
    {
        PMDIN = drvLCCObj.initData->color; while(PMMODEbits.BUSY == 1);
        PMDINSET = 0; while(PMMODEbits.BUSY == 1);
        PMDINSET = 0; while(PMMODEbits.BUSY == 1);
        PMDINSET = 0; while(PMMODEbits.BUSY == 1);
        PMDINSET = 0;
        _frameAddress += 7;
        PMDINSET = 0;
        _pixelRendercount -= 7;
        PMDINSET = 0; while(PMMODEbits.BUSY == 1);
        PMDINSET = 0;
    }
    else
    {
       PMDIN = drvLCCObj.initData->color;
    }
</#if>
        LCCRecoverData();
        _frameAddress++;
       }
       _frameAddress += (DISP_HOR_RESOLUTION - count);
       lineCount--;
    }
</#if>
}  

// *****************************************************************************
/*
  Function: void  DRV_GFX_LCC_PixelArrayPut(uint16_t *color, uint16_t x, uint16_t y, uint16_t count)

  Summary:
    outputs an array of pixels of length count starting at *color 

  Description:
    none

  Input:
          instance - driver instance
          *color - start of the array
		  x - x coordinate of the start point.
		  y - y coordinate of the end point.
		  count - number of pixels
  Output:
         handle to the number of pixels remaining
*/
void  DRV_GFX_LCC_PixelArrayPut(GFX_COLOR *color, uint16_t x, uint16_t y, uint16_t count, uint16_t lineCount)
{
<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
    uint16_t tempy = y;
    uint16_t tempx = x;
    uint16_t tempCount = count;
    GFX_COLOR *point;

    while(lineCount--)
    {
<#if CONFIG_DRV_GFX_DISP_ORIENTATION == "270">
		x = (DISP_HOR_RESOLUTION-1) - tempy;
		y = tempx;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "90">
		y = (DISP_VER_RESOLUTION-1) - tempx;
		x = tempy;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "180">
		x = (DISP_HOR_RESOLUTION-1) - tempx;
		y = (DISP_VER_RESOLUTION-1) - tempy;
<#else>
		x = tempx;
		y = tempy;
</#if>
		point = (GFX_COLOR*)activePageAddress + x + y *(DISP_HOR_RESOLUTION);

		while(tempCount--)
		{

			*point = *color++;
<#if CONFIG_DRV_GFX_DISP_ORIENTATION == "90">
            point -= DISP_HOR_RESOLUTION;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "270">
            point += DISP_HOR_RESOLUTION;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "180">
            point--;
<#else>
            point++;
</#if>
        }

		tempCount = count;
		tempy++;
	}
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
<#if CONFIG_DRV_GFX_DISP_ORIENTATION == "270">
		uint16_t tempx = x;
		uint16_t hchange = count;
		x = (DISP_HOR_RESOLUTION-1) - y;
		y = tempx;
		count = lineCount;
		lineCount = hchange;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "90">
       uint16_t hchange = count;
       uint16_t tempy = y;
       y = (DISP_VER_RESOLUTION-1) - count;
       x = tempy;
       count = lineCount;
       lineCount = hchange;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "180">
        x = (DISP_HOR_RESOLUTION-1) - count;
        y = (DISP_VER_RESOLUTION-1) - y;
</#if>
    _frameAddress = (uint32_t)(((y)*(DISP_HOR_RESOLUTION))+(x));

    while(lineCount)
    {
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
       _framePoint = (uint16_t*)_ebiBaseAddress + (_frameAddress);
</#if>
       for(_pixelRendercount = count; _pixelRendercount > 0; _pixelRendercount--)
       {
          /*Suspend DMA Trigger*/
          LCCSaveData();
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
        if(_pixelRendercount > 8) //Write 8 pixels at once
        {
			*_framePoint++ = *color++;
			*_framePoint++ = *color++;
			*_framePoint++ = *color++;
			*_framePoint++ = *color++;
			*_framePoint++ = *color++;
			*_framePoint++ = *color++;
			*_framePoint++ = *color++;
			_pixelRendercount -= 7;
			_frameAddress += 7;
        }

        *_framePoint++ = *color++;
<#elseif CONFIG_PIC32MX == true>
        if((_pixelRendercount > 4) && (PMADDR < PMP_ADDRESS_LINES-8)) //Write 8 pixels at once
        {
            PMDIN = *color++;
            _pixelRendercount -= 3;
            PMDIN = *color++;
            _frameAddress += 3;
            PMDIN = *color++;
            while(PMMODEbits.BUSY==1);
        }

        PMDIN =  *color++;
</#if>
        LCCRecoverData();
        _frameAddress++;
       }
       _frameAddress += (DISP_HOR_RESOLUTION - count);
       lineCount--;
    }
</#if>
} 

// *****************************************************************************
/*
  Function: uint16_t*  DRV_GFX_LCC_PixelArrayGet(uint16_t *color, uint16_t x, uint16_t y, uint16_t count)

  Summary:
    outputs an array of pixels of length count starting at *color 

  Description:
    none

  Input:
          instance - driver instance
          *color - start of the array
		  x - x coordinate of the start point.
		  y - y coordinate of the end point.
		  count - number of pixels
  Output:
         handle to the number of pixels remaining
*/
uint16_t*  DRV_GFX_LCC_PixelArrayGet(GFX_COLOR *color, uint16_t x, uint16_t y, uint16_t count)
{
<#if CONFIG_DRV_GFX_LCC_MODE == "Internal Memory">
<#if CONFIG_DRV_GFX_DISP_ORIENTATION == "270">
    uint16_t tempx = x;
    x = (DISP_HOR_RESOLUTION-1) - y;
    y = tempx; 
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "90">
    uint16_t tempy = y;
    y = (DISP_VER_RESOLUTION-1) - x;
    x = tempy; 
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "180">
    x = (DISP_HOR_RESOLUTION-1) - x;
    y = (DISP_VER_RESOLUTION-1) - y;
</#if>
	GFX_COLOR *point = (GFX_COLOR*)activePageAddress + x + y *(DISP_HOR_RESOLUTION);

	while(count--)
	{
        *color = *point;
        color++;
<#if CONFIG_DRV_GFX_DISP_ORIENTATION == "90">
        point -= DISP_HOR_RESOLUTION;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "270">
        point += DISP_HOR_RESOLUTION;
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "180">
        point--;
<#else>
        point++;
</#if>
    }
<#elseif CONFIG_DRV_GFX_LCC_MODE == "External Memory">
<#if CONFIG_DRV_GFX_DISP_ORIENTATION == "270">
    uint16_t tempx = x;
    x = (DISP_HOR_RESOLUTION-1) - y;
    y = tempx; 
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "90">
    uint16_t tempy = y;
    y = (DISP_VER_RESOLUTION-1) - x;
    x = tempy; 
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "180">
    x = (DISP_HOR_RESOLUTION-1) - x;
    y = (DISP_VER_RESOLUTION-1) - y;
</#if>
     _frameAddress =  (uint32_t)(((y)*(DISP_HOR_RESOLUTION))+(x));
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
              _framePoint = (uint16_t*)_ebiBaseAddress + (_frameAddress);
</#if>
              while(count)
              {
                /*Suspend DMA Trigger*/
                LCCSaveData();
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
                if(count > 4) // Read 4 pixels at once
                {
                    *color++ = *_framePoint++;
                    count -= 3;
                    *color++ = *_framePoint++;
                    _frameAddress += 3;
                    *color++ = *_framePoint++;
                }
                *color++ = *_framePoint++;
<#elseif CONFIG_PIC32MX == true>
				PMCONCLR = 0x0100;

				*color = PMDIN;
					while(PMMODEbits.BUSY==1);
				*color++ = PMDIN;

					if((count > 4) && (PMADDR < PMP_ADDRESS_LINES-8)) //Write 8 pixels at once
					{
						*color++ = PMDIN;
						_frameAddress += 3;
						*color++ = PMDIN;
						count -= 3;
						*color++ = PMDIN;

					}
					while(PMMODEbits.BUSY==1);
					PMCONSET = 0x0100;
</#if>

                LCCRecoverData();
                
                _frameAddress++;
                count--;
              }
</#if>
    return 0;
}

<#if CONFIG_DRV_GFX_USE_LCC_PIP == true>
<#if CONFIG_DRV_GFX_USE_LCC_LAYERS == true>
uint16_t*  DRV_GFX_LCC_Layer(uint8_t instance, GFX_LAYER_PARAMS* layer)
{
    GFXPIPPage = layer->page;

   if(layer->on == 0)
   {    
       PipVLength = 0;
       return 0;
   }
<#if CONFIG_DRV_GFX_DISP_ORIENTATION == "90">
     PipStartL = (layer->top);
     PipStartT = (DISP_VER_RESOLUTION-1)-(layer->left + layer->width);
     PipVLength = layer->width;
     PipHLength = layer->height;
     PipY = (DISP_VER_RESOLUTION-1) - (layer->layerLeft+layer->width);
     PipX = (layer->layerTop);
<#elseif CONFIG_DRV_GFX_DISP_ORIENTATION == "0">
    PipStartL = layer->left;
    PipStartT = layer->top;   
    PipVLength = layer->height;
    PipHLength = layer->width;
    PipX = layer->layerLeft;
    PipY = layer->layerTop;
</#if>
</#if>
    return 0;
}
</#if>

<#if CONFIG_DRV_GFX_LCC_MODE == "External Memory">
uint16_t prevaddr;

inline void LCCSaveData(void)
{
<#if CONFIG_USE_DRV_GFX_LCC_DMA_WRITE_STRATEGY_IN_BETWEEN_FRAME_AND_LINE == true>
    while ( allowFrameUpdate == false && allowLineUpdate == false ) ;
</#if>
<#if CONFIG_PIC32MX == true>
    while(DCH1CSIZ != 1);
</#if>
<#if CONFIG_DRV_GFX_LCC_DMA_WRITE_STRATEGY == "Draw Count Limited">
    while(DrawCount > PIXEL_DRAW_PER_DMA_TX);
</#if>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
<#if CONFIG_USE_DRV_GFX_LCC_DMA_WRITE_STRATEGY_IN_BETWEEN_FRAME_AND_LINE == true>
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, 7, 1);
</#if>
<#if CONFIG_DRV_GFX_LCC_DMA_WRITE_STRATEGY == "Draw Count Limited">
    PLIB_TMR_Stop(DRV_GFX_LCC_TMR_INDEX);
    while(PMMODEbits.BUSY == 1); //WAIT for DMA transfer to be suspended
</#if>
    CFGEBIA = 0x000fffff;
    CFGEBIC = 0x00002003;
<#elseif CONFIG_PIC32MX == true>
    DMACONbits.SUSPEND = 1;
    while(PMMODEbits.BUSY == 1); //WAIT for DMA transfer to be suspended
    //Perform Write
<#if CONFIG_PIC32MX == true>
#ifdef BSP_SRAM_A15StateSet
    BSP_SRAM_A15StateSet((_frameAddress >> 15)&0x1);
#endif
</#if>
    BSP_SRAM_A16StateSet((_frameAddress >> 16)&0x1);
    BSP_SRAM_A17StateSet((drvLCCObj.initData->activePage)&0x1);
    BSP_SRAM_A18StateSet((drvLCCObj.initData->activePage>>1)&0x1);

    //Save previous address value
    prevaddr = PMADDR;
    PMADDR = _frameAddress;
</#if>
}

inline void LCCRecoverData(void)
{
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
    CFGEBIA = 0;
    CFGEBIC = 0;
<#if CONFIG_DRV_GFX_LCC_DMA_WRITE_STRATEGY == "Draw Count Limited">
    PLIB_TMR_Start(DRV_GFX_LCC_TMR_INDEX);
</#if>
<#if CONFIG_USE_DRV_GFX_LCC_DMA_WRITE_STRATEGY_IN_BETWEEN_FRAME_AND_LINE == true>
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, 7, 1);
</#if>
<#elseif CONFIG_PIC32MX == true>
    //Clean-up Address Lines
#ifdef BSP_SRAM_A15StateSet
    BSP_SRAM_A15StateSet((overflowcount)&0x1);          //count holds the additional address line count
    BSP_SRAM_A16StateSet((overflowcount>>1)&0x1);
#else
    BSP_SRAM_A16StateSet((overflowcount)&0x1);
#endif
    BSP_SRAM_A17StateSet((drvLCCObj.initData->visualPage)&0x1);
    BSP_SRAM_A18StateSet((drvLCCObj.initData->visualPage>>1)&0x1);
    PMADDR = prevaddr;
    DMACONbits.SUSPEND = 0;
</#if>
<#if CONFIG_DRV_GFX_LCC_DMA_WRITE_STRATEGY == "Draw Count Limited">
    DrawCount++;
</#if>
}
</#if>

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_GFX_LCC_Status ( SYS_MODULE_OBJ object )

  Summary:
    Returns status of the specific module instance of the Driver module.

  Description:
    This function returns the status of the specific module instance disabling its
    operation (and any hardware for driver modules).

  PreCondition:
    The DRV_GFX_LCC_Initialize function should have been called before calling
    this function.

  Parameters:
    object          - DRV_GFX_LCC object handle, returned from DRV_GFX_LCC_Initialize

  Returns:
    SYS_STATUS_READY    Indicates that any previous module operation for the
                        specified module has completed

    SYS_STATUS_BUSY     Indicates that a previous module operation for the
                        specified module has not yet completed

    SYS_STATUS_ERROR    Indicates that the specified module is in an error state
*/

SYS_STATUS DRV_GFX_LCC_Status ( SYS_MODULE_OBJ object )
{
    DRV_GFX_LCC_OBJ *dObj = (DRV_GFX_LCC_OBJ*)object;
    return ( dObj->state );

} /* SYS_TMR_Status */

<#if CONFIG_DRV_GFX_USE_LCC_PALETTE == true>
uint8_t DRV_GFX_PaletteSet(uint16_t *pPaletteEntry, uint16_t startEntry, uint16_t length)
{
    uint16_t counter;

    if((pPaletteEntry == NULL) || ((startEntry + length) > 256))
    {
        return -1;
    }

    for(counter = 0; counter < length; counter++)
    {
        LUT[counter]  = *pPaletteEntry++;
    }

    return 0;
}
</#if>
