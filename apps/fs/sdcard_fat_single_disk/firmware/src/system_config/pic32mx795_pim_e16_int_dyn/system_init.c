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
#pragma config ICESEL =     ICS_PGx2
#pragma config PWP =        OFF
#pragma config BWP =        OFF
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      PRIPLL
#pragma config FSOSCEN =    ON
#pragma config IESO =       ON
#pragma config POSCMOD =    XT
#pragma config OSCIOFNC =   OFF
#pragma config FPBDIV =     DIV_1
#pragma config FCKSM =      CSECME
#pragma config WDTPS =      PS1048576
#pragma config FWDTEN =     OFF
/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_2
#pragma config FPLLMUL =    MUL_20
#pragma config FPLLODIV =   DIV_1
#pragma config UPLLIDIV =   DIV_12
#pragma config UPLLEN =     OFF
/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config FSRSSEL =    PRIORITY_7
#pragma config FMIIEN =     OFF
#pragma config FETHIO =     OFF
#pragma config FCANIO =     OFF
#pragma config FUSBIDIO =   OFF
#pragma config FVBUSONIO =  OFF
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="DRV_SDCARD Initialization Data">
/*** SDCARD Driver Initialization Data ***/

const DRV_SDCARD_INIT drvSDCardInit =
{
    .spiId = SPI_ID_1,
    .spiIndex = 0,
    .sdcardSpeedHz = 10000000,
    .spiClk = CLK_BUS_PERIPHERAL_1,
    .writeProtectPort = PORT_CHANNEL_F,
    .writeProtectBitPosition = PORTS_BIT_POS_1,
    .chipSelectPort = PORT_CHANNEL_B,
    .chipSelectBitPosition = PORTS_BIT_POS_1,
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_SPI Initialization Data"> 
 /*** SPI Driver Initialization Data ***/
  /*** Index 0  ***/
 DRV_SPI_INIT drvSpi0InitData =
 {
    .spiId = DRV_SPI_SPI_ID_IDX0,
    .taskMode = DRV_SPI_TASK_MODE_IDX0,
    .spiMode = DRV_SPI_SPI_MODE_IDX0,
    .allowIdleRun = DRV_SPI_ALLOW_IDLE_RUN_IDX0,
    .spiProtocolType = DRV_SPI_SPI_PROTOCOL_TYPE_IDX0,
    .commWidth = DRV_SPI_COMM_WIDTH_IDX0,
    .spiClk = DRV_SPI_SPI_CLOCK_IDX0,
    .baudRate = DRV_SPI_BAUD_RATE_IDX0,
    .bufferType = DRV_SPI_BUFFER_TYPE_IDX0,
    .clockMode = DRV_SPI_CLOCK_MODE_IDX0,
    .inputSamplePhase = DRV_SPI_INPUT_PHASE_IDX0,
    .txInterruptSource = DRV_SPI_TX_INT_SOURCE_IDX0,
    .rxInterruptSource = DRV_SPI_RX_INT_SOURCE_IDX0,
    .errInterruptSource = DRV_SPI_ERROR_INT_SOURCE_IDX0,
    .queueSize = DRV_SPI_QUEUE_SIZE_IDX0,
    .jobQueueReserveSize = DRV_SPI_RESERVED_JOB_IDX0,
 };
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_Timer Initialization Data">
/*** TMR Driver Initialization Data ***/

const DRV_TMR_INIT drvTmr0InitData =
{
    .moduleInit.sys.powerState = DRV_TMR_POWER_STATE_IDX0,
    .tmrId = DRV_TMR_PERIPHERAL_ID_IDX0,
    .clockSource = DRV_TMR_CLOCK_SOURCE_IDX0, 
    .prescale = DRV_TMR_PRESCALE_IDX0,
    .mode = DRV_TMR_OPERATION_MODE_16_BIT,
    .interruptSource = DRV_TMR_INTERRUPT_SOURCE_IDX0,
    .asyncWriteEnable = false,
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
//<editor-fold defaultstate="collapsed" desc="SYS_DEVCON Initialization Data">
/*******************************************************************************
  Device Control System Service Initialization Data
*/

const SYS_DEVCON_INIT sysDevconInit =
{
    .moduleInit = {0},
};

// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="SYS_FS Initialization Data">
/*** File System Initialization Data ***/

const SYS_FS_MEDIA_MOUNT_DATA sysfsMountTable[SYS_FS_VOLUME_NUMBER] = 
{
	{NULL}
};




const SYS_FS_REGISTRATION_TABLE sysFSInit [ SYS_FS_MAX_FILE_SYSTEM_TYPE ] =
{
    {
        .nativeFileSystemType = FAT,
        .nativeFileSystemFunctions = &FatFsFunctions
    }
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="SYS_TMR Initialization Data">
/*** TMR Service Initialization Data ***/
const SYS_TMR_INIT sysTmrInitData =
{
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    .drvIndex = DRV_TMR_INDEX_0,
    .tmrFreq = 1250, 
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Static Initialization Functions
// *****************************************************************************
// *****************************************************************************


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
    SYS_CLK_Initialize( NULL );
    sysObj.sysDevcon = SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)&sysDevconInit);
    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());
    SYS_DEVCON_JTAGDisable();
    SYS_PORTS_Initialize();
    /* Board Support Package Initialization */
    BSP_Initialize();        

    /* Initialize Drivers */

    /*** SPI Driver Index 0 initialization***/

    SYS_INT_VectorPrioritySet(DRV_SPI_INT_VECTOR_IDX0, DRV_SPI_INT_PRIORITY_IDX0);
    SYS_INT_VectorSubprioritySet(DRV_SPI_INT_VECTOR_IDX0, DRV_SPI_INT_SUB_PRIORITY_IDX0);
    sysObj.spiObjectIdx0 = DRV_SPI_Initialize(DRV_SPI_INDEX_0, (const SYS_MODULE_INIT  * const)&drvSpi0InitData);
    sysObj.drvSDCard = DRV_SDCARD_Initialize(DRV_SDCARD_INDEX_0,(SYS_MODULE_INIT *)&drvSDCardInit);

    sysObj.drvTmr0 = DRV_TMR_Initialize(DRV_TMR_INDEX_0, (SYS_MODULE_INIT *)&drvTmr0InitData);

    SYS_INT_VectorPrioritySet(INT_VECTOR_T1, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T1, INT_SUBPRIORITY_LEVEL0);
 
 
 
    /* Initialize System Services */

    /*** File System Service Initialization Code ***/
    SYS_FS_Initialize( (const void *) sysFSInit );

    /*** Interrupt Service Initialization Code ***/
    SYS_INT_Initialize();

    /*** TMR Service Initialization Code ***/
    sysObj.sysTmr  = SYS_TMR_Initialize(SYS_TMR_INDEX_0, (const SYS_MODULE_INIT  * const)&sysTmrInitData);
  
    /* Initialize Middleware */

    /* Enable Global Interrupts */
    SYS_INT_Enable();

    /* Initialize the Application */
    APP_Initialize();
}


/*******************************************************************************
 End of File
*/

