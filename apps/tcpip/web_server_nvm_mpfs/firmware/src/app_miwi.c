/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_miwi.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#include "app_miwi.h"
#include "GenericTypeDefs.h"
#include "mla_v13_june_2013/LCD_ST7032.h"
#include "WirelessProtocols/SymbolTime.h"
//#include <peripheral/osc.h>
#include "legacy/osc.h"
#include "legacy/ports.h"
#include "legacy/spi_legacy.h"
#include "legacy/int_5xx_6xx_7xx_legacy.h"
#include "legacy/system.h"
#include "WirelessProtocols/SymbolTime.h"
#include "XEEPROM.h"
#include "WirelessProtocols/MiWiHelper.h"
#include "mla_v13_june_2013/nvm/nvm.h"


void putsUART2(unsigned char *buffer);
void DelayMs(uint16_t ms);
BOOL MiApp_ProtocolInit(BOOL bNetworkFreezer);
static void InitializeBoard(void);

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
#define PIC32MX_SPI1_SDO_SCK_MASK_VALUE      (0x00000140)
#define PIC32MX_SPI1_SDI_MASK_VALUE          (0x00000080)
//#define PIC32MX_INT2_MASK_VALUE              (0x00000010)
//#define PIC32MX_INT1_MASK_VALUE              (0x00000008)
#define PIC32MX_INT1_MASK_VALUE              (0x00000100)
/* MAX SPI CLOCK FREQ SUPPORTED FOR MIWI TRANSCIEVER */
#define MAX_SPI_CLK_FREQ_FOR_P2P             (10000000) //For MRF24J40 10MHz, for MRF89XA 1MHz

BYTE LCDText[16*2+1];


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_MIWI_DATA app_miwiData;
BYTE miwi_msg[][40] = 
                    {
                            "\r\nMiWi Network created\r\n",
                            "\r\nMiWi Network failed\r\n",
                            "\r\n",
                            "\r\nRunning MiWi at channel...",
                    };
 static MyConfigType DefaultConfig __attribute__((space(prog),address(0x9D000000))) =
        {
            TCPIP_NETWORK_DEFAULT_INTERFACE_NAME,       // interface
            TCPIP_NETWORK_DEFAULT_HOST_NAME,            // hostName
            TCPIP_NETWORK_DEFAULT_MAC_ADDR,             // macAddr
            TCPIP_NETWORK_DEFAULT_IP_ADDRESS,           // ipAddr
            TCPIP_NETWORK_DEFAULT_IP_MASK,              // ipMask
            TCPIP_NETWORK_DEFAULT_GATEWAY,              // gateway
            TCPIP_NETWORK_DEFAULT_DNS,                  // priDNS
            TCPIP_NETWORK_DEFAULT_SECOND_DNS,           // secondDNS
            TCPIP_NETWORK_DEFAULT_POWER_MODE,           // powerMode
        //                    TCPIP_NETWORK_DEFAULT_INTERFACE_FLAGS,      // startFlags
        //    &TCPIP_NETWORK_DEFAULT_MAC_DRIVER,           // pMacObject
        };

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

static void InitializeBoard(void) {
    // WiFi Module hardware Initialization handled by Library

    // Enable multi-vectored interrupts
//    INTEnableSystemMultiVectoredInt();
//
    // Enable optimal performance
//    SYSTEMConfigPerformance(GetSystemClock());
//    mOSCSetPBDIV(OSC_PB_DIV_4); // Use 1:1 CPU Core:Peripheral clocks
//    mOSCSetPBDIV(OSC_PB_DIV_1); // Use 1:1 CPU Core:Peripheral clocks
//
//    // Disable JTAG port so we get our I/O pins back, but first
//    // wait 50ms so if you want to reprogram the part with
//    // JTAG, you'll still have a tiny window before JTAG goes away.
//    // The PIC32 Starter Kit debuggers use JTAG and therefore must not
//    // disable JTAG.
//    DelayMs(50);
//    DDPCONbits.JTAGEN = 0;

    // LEDs
    LEDS_OFF();
    mPORTESetPinsDigitalOut(BIT_5 | BIT_6 | BIT_7);

    // Switches
    mPORTDSetPinsDigitalIn(BIT_4 | BIT_5 | BIT_6);
    ConfigCNPullups(CN13_PULLUP_ENABLE | CN14_PULLUP_ENABLE | CN15_PULLUP_ENABLE);

    // LCD
    mPORTESetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2 | BIT_3);
    //Configure LCD SPI pins
    mPORTFSetPinsDigitalOut(BIT_8);
    mPORTDSetPinsDigitalOut(BIT_15);

    //SPI Flash
    mPORTDSetPinsDigitalOut(BIT_14);


    //UART
    mPORTFSetPinsDigitalOut(BIT_5);
    mPORTFSetPinsDigitalIn(BIT_4);

    //MiWi
#if defined(MRF24J40) || defined(MRF49XA)
    PHY_CS = 1;
    mPORTDSetPinsDigitalOut(BIT_9);

    PHY_RESETn = 1;
    mPORTDSetPinsDigitalOut(BIT_11);
#endif

#if defined(MRF49XA)
    nFSEL_TRIS = 0;
    FINT_TRIS = 1;

    nFSEL = 1;
#elif defined(MRF24J40)
    PHY_WAKE = 1;
    mPORTBSetPinsDigitalOut(BIT_9);
#else
    Data_nCS_TRIS = 0;
    Config_nCS_TRIS = 0;
    Data_nCS = 1;
    Config_nCS = 1;
    IRQ1_INT_TRIS = 1;
    IRQ0_INT_TRIS = 1;

#endif

    /* Set the Port Directions of SDO, SDI, Clock & Slave Select Signal */
    /* Set SCK port pin to output */
    mPORTDSetPinsDigitalOut(BIT_10);
    /* Set SDO port pin to output */
    mPORTDSetPinsDigitalOut(BIT_0);
    /* Set SDI port pin to input */
    mPORTCSetPinsDigitalIn(BIT_4);
    /* Set INT1, INT2 port pins to input */
    mPORTESetPinsDigitalIn(BIT_8 | BIT_9);

    /* Clear SPI1CON register */
    SPI1CONCLR = 0xFFFFFFFF;

#ifdef HARDWARE_SPI
    unsigned int pbFreq;

    /* Enable SPI1, Set to Master Mode & Set CKE bit : Serial output data changes on transition
      from active clock state to Idle clock state */
    SPI1CON = 0x00008120;
    /* Peripheral Bus Frequency = System Clock / PB Divider */
    pbFreq = (DWORD) CLOCK_FREQ / (1 << mOSCGetPBDIV());

    /* PB Frequency can be maximum 40 MHz */
    if (pbFreq > (2 * MAX_SPI_CLK_FREQ_FOR_P2P)) {
        {
            unsigned int SPI_Clk_Freq;

            unsigned char SPI_Brg1 = 1;

            //For the SPI1
            /* Continue the loop till you find SPI Baud Rate Register Value */
            while (1) {
                /* SPI Clock Calculation as per PIC32 Manual */
                SPI_Clk_Freq = pbFreq / (2 * (SPI_Brg1 + 1));

                if (SPI_Clk_Freq <= MAX_SPI_CLK_FREQ_FOR_P2P) {
                    break;
                }

                SPI_Brg1++;
            }



            mSpiChnSetBrg(1, SPI_Brg1);

        }
    } else {
        /* Set SPI1 Baud Rate */
        mSpiChnSetBrg(1, 0);

    }

#endif

    /* Set the Interrupt Priority */
    mINT2SetIntPriority(4);     //for radio miwi

#if defined(MRF89XA)
    mINT1SetIntPriority(4);
#endif

    /* Set Interrupt Subpriority Bits for INT2 */
    mINT2SetIntSubPriority(2);

#if defined(MRF89XA)
    mINT2SetIntSubPriority(1);
#endif

    /* Set INT2 to falling edge */
    mINT2SetEdgeMode(0);

#if defined(MRF89XA)
    mINT1SetEdgeMode(1);
    mINT2SetEdgeMode(1);
#endif

    /* Enable INT2 */
    mINT2IntEnable(1);

#if defined(MRF89XA)
    mINT2IntEnable(1);
#endif

    /* Enable Multi Vectored Interrupts */
    //    INTEnableSystemMultiVectoredInt();

#if defined(MRF89XA)
    PHY_IRQ1 = 0;
    PHY_IRQ0 = 0;
    PHY_RESETn_TRIS = 1;
#else
    RFIF = 0;
    if (RF_INT_PIN == 0) {
        RFIF = 1;
    }
#endif

    // Initialize the EEPROM
    XEEInit();

    // UART Initialization
#if defined(STACK_USE_UART)
    UARTTX_TRIS = 0;
    UARTRX_TRIS = 1;
    UMODE = 0x8000; // Set UARTEN.  Note: this must be done before setting UTXEN
    USTA = 0x00001400; // RXEN set, TXEN set
#define CLOSEST_UBRG_VALUE ((GetPeripheralClock()+8ul*BAUD_RATE)/16/BAUD_RATE-1)
#define BAUD_ACTUAL (GetPeripheralClock()/16/(CLOSEST_UBRG_VALUE+1))

#define BAUD_ERROR ((BAUD_ACTUAL > BAUD_RATE) ? BAUD_ACTUAL-BAUD_RATE : BAUD_RATE-BAUD_ACTUAL)
#define BAUD_ERROR_PRECENT	((BAUD_ERROR*100+BAUD_RATE/2)/BAUD_RATE)
#if (BAUD_ERROR_PRECENT > 3)
#warning UART frequency error is worse than 3%
#elif (BAUD_ERROR_PRECENT > 2)
#warning UART frequency error is worse than 2%
#endif

    UBRG = CLOSEST_UBRG_VALUE;
#endif
//#ifdef ENABLE_CONSOLE
//    ConsoleInit();
//#endif

}

void example_write_nvm_flash(void)
{
       

        
         static struct 
        {
            char                ifName[10 + 1];       // interface name
            char                nbnsName[16 + 1];     // host name
            char                ifMacAddr[17 + 1];    // MAC address
            char                ipAddr[15 +1];        // IP address
            char                ipMask[15 + 1];       // mask
            char                gwIP[15 + 1];         // gateway IP address
            char                dns1IP[15 + 1];       // DNS IP address
            char                dns2IP[15 + 1];       // DNS IP address
        }httpNetData = {
                "ENC28J60  ",
                "BIRDPEEK        ",
                "00:04:A3:11:22:33",
                "192.168.43.200",
                "255.255.255.0",
                "192.168.43.1",
                "192.168.43.10",
                "192.168.43.11",
                };
        DWORD *addr, buffer[4096];        
        WORD i, size;
       
        memset(&buffer, 0xffffffff, sizeof(buffer));
        size = sizeof(DefaultConfig);
        memcpy(buffer, &DefaultConfig, 4096); //overwrite/save backup old data
        memcpy(buffer, &httpNetData, sizeof(httpNetData)); //copy new value from httpNetData into buffer
        Nop();
        addr = (DWORD*)&DefaultConfig;
        NVMUpdate(addr, buffer);
        
        Nop();
        LEDS_ON();

}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
MIWI_TICK t1, t0;
/*******************************************************************************
  Function:
    void APP_MIWI_Initialize ( void )

  Remarks:
    See prototype in app_miwi.h.
 */

void APP_MIWI_Initialize ( void )
{
        
        /* Place the App state machine in its initial state. */
        app_miwiData.state = APP_MIWI_STATE_INIT;
        LEDS_OFF();
//        InitializeBoard();
        XEEInit();
        SPIFlashInit();
        LCDInit();
        /*******************************************************************/
        // Display Start-up Splash Screen
        /*******************************************************************/
        LCDBacklightON();
        LED2_ON();
        LCDErase();
        sprintf((char *) LCDText, (char*) "  MiWi - WiFi  ");
        sprintf((char *) &(LCDText[16]), (char*) " Gateway  Demo");
        LCDUpdate();
        
        DelayMs(200);
        LCDErase();
        strcpy((char *) LCDText, DRV_WIFI_DEFAULT_SSID);
        strcpy((char *) LCDText+16, TCPIP_NETWORK_DEFAULT_IP_ADDRESS);
        LCDUpdate();
        
        DelayMs(200);
        
        LCDBacklightOFF();
        if( BSP_SWITCH_0StateGet() )
        {
                MiApp_ProtocolInit(FALSE);
                app_miwiData.connection = CreateNewConnectionAtChannel(currentChannel ) ;
                app_miwiData.channel = currentChannel;
        }
        else{
                MiApp_ProtocolInit(TRUE);
        }
        
        putsUART(miwi_msg[3] );
        PrintDec( app_miwiData.channel  );
        putsUART(miwi_msg[2]);
        t1 = MiWi_TickGet();
        
        example_write_nvm_flash();
        
}


/******************************************************************************
  Function:
    void APP_MIWI_Tasks ( void )

  Remarks:
    See prototype in app_miwi.h.
 */

void APP_MIWI_Tasks ( void )
{
         
        Nop();
        t0 = MiWi_TickGet();
        if( MiWi_TickGetDiff(t0, t1) > (2 * ONE_SECOND) )
        {
                t1 = MiWi_TickGet();
                LED_1 ^= 1;
        }
        Nop();

    /* Check the application's current state. */
    switch ( app_miwiData.state )
    {
        /* Application's initial state. */
        case APP_MIWI_STATE_INIT:
        {
            bool appInitialized = true;            
       
        
            if (appInitialized)
            {
            
                app_miwiData.state = APP_MIWI_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_MIWI_STATE_SERVICE_TASKS:
        {
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
