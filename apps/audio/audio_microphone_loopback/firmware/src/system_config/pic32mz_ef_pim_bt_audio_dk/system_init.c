/*******************************************************************************
  System Initialization File

  File Name:
    system_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, defines the configuration bits, 
    and allocates any necessary global system resources, such as the 
    sysObj structure that contains the object handles to all the MPLAB Harmony 
    module objects in the system.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"


// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************
// <editor-fold defaultstate="collapsed" desc="Configuration Bits">

/*** DEVCFG0 ***/

#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx2
#pragma config TRCEN =      OFF
#pragma config BOOTISA =    MIPS32
#pragma config FECCCON =    OFF_UNLOCKED
#pragma config FSLEEP =     OFF
#pragma config DBGPER =     PG_ALL
#pragma config SMCLR =      MCLR_NORM
#pragma config SOSCGAIN =   GAIN_2X
#pragma config SOSCBOOST =  ON
#pragma config POSCGAIN =   GAIN_2X
#pragma config POSCBOOST =  ON
#pragma config EJTAGBEN =   NORMAL
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      SPLL
#pragma config DMTINTV =    WIN_127_128
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    HS
#pragma config OSCIOFNC =   OFF
#pragma config FCKSM =      CSDCMD
#pragma config WDTPS =      PS1048576
#pragma config WDTSPGM =    STOP
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     NORMAL
#pragma config FWDTWINSZ =  WINSZ_25
#pragma config DMTCNT =     DMT31
#pragma config FDMTEN =     OFF
/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_1
#pragma config FPLLRNG =    RANGE_8_16_MHZ
#pragma config FPLLICLK =   PLL_POSC
#pragma config FPLLMULT =   MUL_33
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLFSEL =   FREQ_12MHZ
/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config FMIIEN =     OFF
#pragma config FETHIO =     OFF
#pragma config PGL1WAY =    OFF
#pragma config PMDL1WAY =   OFF
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   OFF

/*** BF1SEQ0 ***/

#pragma config TSEQ =       0xffff
#pragma config CSEQ =       0xffff
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="DRV_CODEC_AK4642 Initialization Data">
/*** CODEC Driver Initialization Data ***/
const DRV_AK4642_INIT drvak4642Codec0InitData =
{
    .moduleInit.value = SYS_MODULE_POWER_RUN_FULL,
    .i2sDriverModuleIndex = DRV_AK4642_I2S_DRIVER_MODULE_INDEX_IDX0,
    .i2cDriverModuleIndex = DRV_AK4642_I2C_DRIVER_MODULE_INDEX_IDX0,

    .volume = DRV_AK4642_VOLUME,
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_GFX Initialization Data">
/*** DRV GFX Initialization Data ***/

DRV_GFX_INIT drvGfxInit =
{
    .orientation             = DISP_ORIENTATION,
    .horizontalResolution    = DISP_HOR_RESOLUTION,
    .verticalResolution      = DISP_VER_RESOLUTION,
    .dataWidth               = DISP_DATA_WIDTH,
    .horizontalPulseWidth    = DISP_HOR_PULSE_WIDTH,
    .horizontalBackPorch     = DISP_HOR_BACK_PORCH,
    .horizontalFrontPorch    = DISP_HOR_FRONT_PORCH,
    .verticalPulseWidth      = DISP_VER_PULSE_WIDTH,
    .verticalBackPorch       = DISP_VER_BACK_PORCH,
    .verticalFrontPorch      = DISP_VER_FRONT_PORCH,
    .logicShift              = DISP_INV_LSHIFT,
    .LCDType                 = 1,
    .colorType               = 0,
    .TCON_Init               = TCON_MODULE,
};

// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_I2C Initialization Data">
// *****************************************************************************
/* I2C Driver Initialization Data
*/

const DRV_I2C_INIT drvI2C0InitData =
{
    .i2cId = DRV_I2C_PERIPHERAL_ID_IDX0,
    .i2cMode = DRV_I2C_OPERATION_MODE_IDX0,
    .baudRate = DRV_I2C_BAUD_RATE_IDX0,
    .busspeed = DRV_I2C_SLEW_RATE_CONTROL_IDX0,
    .buslevel = DRV_I2C_SMBus_SPECIFICATION_IDX0,
    .mstrInterruptSource = DRV_I2C_MASTER_INT_SRC_IDX0,
    .errInterruptSource = DRV_I2C_ERR_MZ_INT_SRC_IDX0,
};




// </editor-fold>
 // <editor-fold defaultstate="collapsed" desc="DRV_I2S Initialization Data">
/*** I2S Driver Initialization Data ***/
const DRV_I2S_INIT drvI2S0InitData =
{
    .moduleInit.value = DRV_I2S_POWER_STATE_IDX0,
    .spiID = DRV_I2S_PERIPHERAL_ID_IDX0, 
    .usageMode = DRV_I2S_USAGE_MODE_IDX0,
    .baudClock = SPI_BAUD_RATE_CLK_IDX0,
    .baud = DRV_I2S_BAUD_RATE,
    .clockMode = DRV_I2S_CLK_MODE_IDX0,
    .audioCommWidth = SPI_AUDIO_COMM_WIDTH_IDX0,
    .audioTransmitMode = SPI_AUDIO_TRANSMIT_MODE_IDX0,
    .inputSamplePhase = SPI_INPUT_SAMPLING_PHASE_IDX0,
    .protocolMode = DRV_I2S_AUDIO_PROTOCOL_MODE_IDX0,
    .queueSizeTransmit = QUEUE_SIZE_TX_IDX0,
    .queueSizeReceive = QUEUE_SIZE_RX_IDX0,
    .dmaChannelTransmit = DRV_I2S_TX_DMA_CHANNEL_IDX0,
    .txInterruptSource = DRV_I2S_TX_INT_SRC_IDX0,    
    .dmaInterruptTransmitSource = DRV_I2S_TX_DMA_SOURCE_IDX0,    
    .dmaChannelReceive = DRV_I2S_RX_DMA_CHANNEL_IDX0,
    .rxInterruptSource = DRV_I2S_RX_INT_SRC_IDX0,
    .dmaInterruptReceiveSource = DRV_I2S_RX_DMA_SOURCE_IDX0,
};





// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Data
// *****************************************************************************
// *****************************************************************************

/* Structure to hold the object handles for the modules in the system. */
SYSTEM_OBJECTS sysObj;

// *****************************************************************************
// *****************************************************************************
// Section: Module Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="SYS_CLK Initialization Data">
// *****************************************************************************
/* System Clock Initialization Data
*/
const SYS_CLK_INIT sysClkInit =
{
    .moduleInit = {0},
    .systemClockFrequencyHz = SYS_CLK_FREQ,
    .waitTillComplete = true,
    .onWaitInstruction = SYS_CLK_ON_WAIT,
};
// </editor-fold>
//<editor-fold defaultstate="collapsed" desc="SYS_DEVCON Initialization Data">
/*******************************************************************************
  Device Control System Service Initialization Data
*/

const SYS_DEVCON_INIT sysDevconInit =
{
    .moduleInit = {0},
};

// </editor-fold>
//<editor-fold defaultstate="collapsed" desc="SYS_DMA Initialization Data">
/*** System DMA Initialization Data ***/

const SYS_DMA_INIT sysDmaInit =
{
	.sidl = SYS_DMA_SIDL_DISABLE,

};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="Graphics Library Initialization Data">
/*** GFX Initialization Data ***/

 const GFX_INIT gfxInit0 =
{
    .drvInitialize    = NULL,
    .drvOpen          = DRV_GFX_OTM2201A_Open,
    .drvInterfaceSet  = DRV_GFX_OTM2201A_InterfaceSet,
    .preemptionLevel  = GFX_SELF_PREEMPTION_LEVEL
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Static Initialization Functions
// *****************************************************************************
// *****************************************************************************


/*******************************************************************************
  Function:
    void SYS_CLK_DYNAMIC_Initialize ( void )

  Summary:
    Initializes the peripheral bus clock dividers, reference clock dividers and output enables

 */

void SYS_CLK_DYNAMIC_Initialize()
{
    SYS_DEVCON_SystemUnlock ( );

    /* System Clock Initialization Code */
    PLIB_OSC_FRCDivisorSelect( OSC_ID_0, OSC_FRC_DIV_1);

    /* Enable Peripheral Bus 1 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_1, 2 );
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_1 );
    /* Enable Peripheral Bus 2 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_2, 2 );
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_2 );
    /* Enable Peripheral Bus 3 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_3, 2 );
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_3 );
    /* Enable Peripheral Bus 4 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_4, 2 );
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_4 );
    /* Enable Peripheral Bus 5 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_5, 2 );
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_5 );
    /* Enable Peripheral Bus 7 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_7, 1 );
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_7 );
    /* Enable Peripheral Bus 8 */
    PLIB_OSC_PBClockDivisorSet (OSC_ID_0, OSC_PERIPHERAL_BUS_8, 1 );
    PLIB_OSC_PBOutputClockEnable (OSC_ID_0, OSC_PERIPHERAL_BUS_8 );
    /* Enable and configure REFCLKO1*/
    
    /* ROSEL System PLL output SPLL */
    PLIB_OSC_ReferenceOscBaseClockSelect ( OSC_ID_0, OSC_REFERENCE_1, 7 );
    /* RODIV */
    PLIB_OSC_ReferenceOscDivisorValueSet ( OSC_ID_0, OSC_REFERENCE_1, 8 );
    /* ROTRIM */
    PLIB_OSC_ReferenceOscTrimSet ( OSC_ID_0, OSC_REFERENCE_1, 29 );

    PLIB_OSC_ReferenceOscEnable ( OSC_ID_0, OSC_REFERENCE_1 );    /* Disable REFCLK1_OE*/
    PLIB_OSC_ReferenceOutputEnable ( OSC_ID_0, OSC_REFERENCE_1 );    /* Disable REFCLKO2*/
    PLIB_OSC_ReferenceOscDisable ( OSC_ID_0, OSC_REFERENCE_2 );    /* Disable REFCLK2_OE*/
    PLIB_OSC_ReferenceOutputDisable ( OSC_ID_0, OSC_REFERENCE_2 );    /* Disable REFCLKO3*/
    PLIB_OSC_ReferenceOscDisable ( OSC_ID_0, OSC_REFERENCE_3 );    /* Disable REFCLK3_OE*/
    PLIB_OSC_ReferenceOutputDisable ( OSC_ID_0, OSC_REFERENCE_3 );    /* Disable REFCLKO4*/
    PLIB_OSC_ReferenceOscDisable ( OSC_ID_0, OSC_REFERENCE_4 );    /* Disable REFCLK4_OE*/
    PLIB_OSC_ReferenceOutputDisable ( OSC_ID_0, OSC_REFERENCE_4 );
    SYS_DEVCON_SystemLock ( );
}


// *****************************************************************************
// *****************************************************************************
// Section: System Initialization
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Initialize ( void *data )

  Summary:
    Initializes the board, services, drivers, application and other modules.

  Remarks:
    See prototype in system/common/sys_module.h.
 */

void SYS_Initialize ( void* data )
{
    /* Core Processor Initialization */
    SYS_CLK_Initialize(&sysClkInit);
    SYS_CLK_DYNAMIC_Initialize();
    sysObj.sysDevcon = SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)&sysDevconInit);
    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());
    SYS_PORTS_Initialize();
    /* Board Support Package Initialization */
    BSP_Initialize();        

    /* Initialize Drivers */
    sysObj.drvI2C0 = DRV_I2C_Initialize(DRV_I2C_INDEX_0, (SYS_MODULE_INIT *)&drvI2C0InitData);


    SYS_INT_VectorPrioritySet(INT_VECTOR_I2C1_MASTER, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_I2C1_MASTER, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_VectorPrioritySet(INT_VECTOR_I2C1_BUS, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_I2C1_BUS, INT_SUBPRIORITY_LEVEL0);


    sysObj.drvI2S0 = DRV_I2S_Initialize(DRV_I2S_INDEX_0, (SYS_MODULE_INIT *)&drvI2S0InitData);




    /*Initialize PMP0 */
    DRV_PMP0_Initialize();
    DRV_PMP0_ModeConfig();

    sysObj.sysDma = SYS_DMA_Initialize((SYS_MODULE_INIT *)&sysDmaInit);
    SYS_INT_VectorPrioritySet(INT_VECTOR_DMA0, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_DMA0, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_VectorPrioritySet(INT_VECTOR_DMA2, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_DMA2, INT_SUBPRIORITY_LEVEL0);

    SYS_INT_SourceEnable(INT_SOURCE_DMA_0);
    SYS_INT_SourceEnable(INT_SOURCE_DMA_2);


    sysObj.drvak4642Codec0 = DRV_AK4642_Initialize(DRV_AK4642_INDEX_0, (SYS_MODULE_INIT *)&drvak4642Codec0InitData);

    DRV_GFX_OTM2201A_Initialize(GFX_INDEX_0, (SYS_MODULE_INIT*)&drvGfxInit);

    /* Initialize System Services */

    /*** Interrupt Service Initialization Code ***/
    SYS_INT_Initialize();
  
    /* Initialize Middleware */

    sysObj.gfxObject0 = GFX_Initialize(GFX_INDEX_0, (SYS_MODULE_INIT *)&gfxInit0);

    ImageDecoderInit();
    JPEGInit();

    /* Initialize the GFX HGC */
    GFX_HGC_Initialize();

    /* Enable Global Interrupts */
    SYS_INT_Enable();

    /* Initialize the Application */
    APP_Initialize();
}


/*******************************************************************************
 End of File
*/

