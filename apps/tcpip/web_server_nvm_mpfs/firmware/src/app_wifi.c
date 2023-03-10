/*******************************************************************************
  File Name:
    app_wifi.c

  Summary:


  Description:

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
#include <sys/attribs.h>
#include "app.h"
#include "GenericTypeDefs.h"

#if defined(WIRELESS_EVAL_BOARD)
#include "LCD_ST7032.h"
void DelayMs(uint16_t ms);
#endif


/*
 * So far this application is either using FreeRTOS V8.x.x
 * or not using any RTOS at all, so other cases are simply ignored.
 */
#if (OSAL_USE_RTOS == 1) /* It means FreeRTOS V8.x.x is used. */
#define APP_OSAL_MUTEX_LOCK() APP_OSAL_MutexLock(&s_appLock, OSAL_WAIT_FOREVER)
#define APP_OSAL_MUTEX_UNLOCK() APP_OSAL_MutexUnlock(&s_appLock)
#else
#define APP_OSAL_MUTEX_LOCK() do {} while (0)
#define APP_OSAL_MUTEX_UNLOCK() do {} while (0)
#endif

#if defined(TCPIP_IF_MRF24W)
#define WIFI_INTERFACE_NAME "MRF24W"
#elif defined(TCPIP_IF_MRF24WN)
#define WIFI_INTERFACE_NAME "MRF24WN"
#endif

#if defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
#define IS_MDNS_RUN() true
#else
#define IS_MDNS_RUN() false
#define TCPIP_MDNS_ServiceRegister(a, b, c, d, e, f, g, h) do { } while (0)
#define TCPIP_MDNS_ServiceDeregister(a) do { } while (0)
#endif /* defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD) */

#if defined(TCPIP_STACK_USE_NBNS)
#define IS_NBNS_RUN() true
#else
#define IS_NBNS_RUN() false
#endif /* defined(TCPIP_STACK_USE_NBNS) */

#define IS_WF_INTF(x) ((strcmp(x, "MRF24W") == 0) || (strcmp(x, "MRF24WN") == 0))

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************
extern BYTE LCDText[16*2+1];
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */
static APP_DATA s_appData;
static BSP_LED_STATE s_LEDstate = BSP_LED_STATE_OFF;
static IWPRIV_GET_PARAM s_app_get_param;
static IWPRIV_SET_PARAM s_app_set_param;

#if (OSAL_USE_RTOS == 1) /* It means FreeRTOS V8.x.x is used. */
static OSAL_MUTEX_HANDLE_TYPE s_appLock;
#endif

bool g_redirect_signal = false;
WF_CONFIG_DATA g_wifi_cfg;
WF_DEVICE_INFO g_wifi_deviceInfo;
WF_REDIRECTION_CONFIG g_redirectionConfig;

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

/* TODO:  Add any necessary local functions.
*/

#if (OSAL_USE_RTOS == 1) /* It means FreeRTOS V8.x.x is used. */
/* The application task runs forever, so no de-init function is provided. */
static bool APP_OSAL_MutexInit(OSAL_MUTEX_HANDLE_TYPE *mutex);
static void APP_OSAL_MutexLock(OSAL_MUTEX_HANDLE_TYPE *mutex, uint16_t waitMS);
static void APP_OSAL_MutexUnlock(OSAL_MUTEX_HANDLE_TYPE *mutex);
#endif

static void APP_CONSOLE_HeaderDisplay(void);
static void APP_WIFI_IPv6MulticastFilter_Set(TCPIP_NET_HANDLE netH);
static void APP_WIFI_PowerSave_Config(bool enable);
static void APP_WIFI_DHCPS_Sync(TCPIP_NET_HANDLE netH);
static void APP_TCPIP_IFModules_Disable(TCPIP_NET_HANDLE netH);
static void APP_TCPIP_IFModules_Enable(TCPIP_NET_HANDLE netH);
static void APP_TCPIP_IF_Down(TCPIP_NET_HANDLE netH);
static void APP_TCPIP_IF_Up(TCPIP_NET_HANDLE netH);

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_WIFI_Initialize(void)

  Remarks:
    None.
 */
void APP_WIFI_Initialize(void)
{
#if (OSAL_USE_RTOS == 1) /* It means FreeRTOS V8.x.x is used. */
    if (!APP_OSAL_MutexInit(&s_appLock)) {
        SYS_CONSOLE_MESSAGE("APP: Mutex initialization failed!\r\n");
        return;
    }
#endif

    s_app_set_param.conn.initConnAllowed = true;
    iwpriv_set(INITCONN_OPTION_SET, &s_app_set_param);
    
        
}

/*******************************************************************************
  Function:
    void APP_WIFI_Tasks(void)

  Remarks:
    None.
 */
void APP_WIFI_Tasks(void)
{
    static bool isWiFiPowerSaveConfigured = false;
    static bool wasNetUp[2] = {true, true}; // this app supports 2 interfaces so far
    static uint32_t startTick = 0;
    static IPV4_ADDR defaultIPWiFi = {-1};
    static IPV4_ADDR dwLastIP[2] = { {-1}, {-1} }; // this app supports 2 interfaces so far
    static TCPIP_NET_HANDLE netHandleWiFi;
    int i, nNets;
    
     IPV4_ADDR       ipAddr, ipMask, gateway, priDNS, secondDNS;

    switch (s_appData.state)
    {
        case APP_MOUNT_DISK:
            if (SYS_FS_Mount(SYS_FS_NVM_VOL, LOCAL_WEBSITE_PATH_FS, MPFS2, 0, NULL) == 0)
            {
                SYS_CONSOLE_PRINT("SYS_Initialize: The %s File System is mounted\r\n", SYS_FS_MPFS_STRING);
                APP_CONSOLE_HeaderDisplay();
                s_appData.state = APP_TCPIP_WAIT_INIT;
            }
            else
            {
                //SYS_CONSOLE_PRINT("SYS_Initialize: Mount the %s File System, pending\r\n", SYS_FS_MPFS_STRING);
            }
            break;

        case APP_TCPIP_WAIT_INIT: {
            SYS_STATUS tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            if (tcpipStat < 0)
            {
                SYS_CONSOLE_MESSAGE("APP: TCP/IP stack initialization failed!\r\n");
                s_appData.state = APP_TCPIP_ERROR;
            }
            else if (tcpipStat == SYS_STATUS_READY)
            {
                s_appData.state = APP_WIFI_CONFIG;
            }
            break;
        }

        case APP_WIFI_CONFIG:
            /*
             * Following "if condition" is useless when demo firstly
             * boots up, since stack's status has already been checked in
             * APP_TCPIP_WAIT_INIT. But it is necessary in Wi-Fi interface
             * reset due to connection errors.
             */
            iwpriv_get(INITSTATUS_GET, &s_app_get_param);
            if (s_app_get_param.driverStatus.initStatus == IWPRIV_READY) {
                s_app_get_param.devInfo.data = &g_wifi_deviceInfo;
                iwpriv_get(DEVICEINFO_GET, &s_app_get_param);
                defaultIPWiFi.Val = TCPIP_STACK_NetAddress(netHandleWiFi);
                netHandleWiFi = TCPIP_STACK_NetHandleGet(WIFI_INTERFACE_NAME);
                s_appData.state = APP_TCPIP_MODULES_ENABLE;
            } else {
                break;
            }

        case APP_TCPIP_MODULES_ENABLE:
            // check the available interfaces
            nNets = TCPIP_STACK_NumberOfNetworksGet();
            for (i = 0; i < nNets; ++i)
                APP_TCPIP_IFModules_Enable(TCPIP_STACK_IndexToNet(i));
            s_appData.state = APP_TCPIP_TRANSACT;
            if(MyConfig.startFlags == TCPIP_NETWORK_CONFIG_IP_STATIC)
            {
                TCPIP_DHCP_Disable(netHandleWiFi);
                TCPIP_Helper_StringToIPAddress(MyConfig.netConfig.ipAddr, &ipAddr);        
                TCPIP_Helper_StringToIPAddress(MyConfig.netConfig.ipMask, &ipMask);        
                TCPIP_Helper_StringToIPAddress(MyConfig.netConfig.gateway, &gateway);  
                TCPIP_Helper_StringToIPAddress(MyConfig.netConfig.priDNS, &priDNS);        
                TCPIP_Helper_StringToIPAddress(MyConfig.netConfig.secondDNS, &secondDNS);       
                
                TCPIP_STACK_NetAddressSet(netHandleWiFi, (IPV4_ADDR*)&ipAddr, (IPV4_ADDR*)&ipMask, true);
                TCPIP_STACK_NetAddressGatewaySet(netHandleWiFi, (IPV4_ADDR*)&gateway);
                TCPIP_STACK_NetAddressDnsPrimarySet(netHandleWiFi, (IPV4_ADDR*)&priDNS);
                TCPIP_STACK_NetAddressDnsSecondSet(netHandleWiFi, (IPV4_ADDR*)&secondDNS);
            }
            else if(MyConfig.startFlags != TCPIP_NETWORK_CONFIG_IP_STATIC)
            {
                    TCPIP_DHCP_Enable(netHandleWiFi);
            }

        case APP_TCPIP_TRANSACT:
            iwpriv_get(CONNSTATUS_GET, &s_app_get_param);
            if (s_app_get_param.conn.status == IWPRIV_CONNECTION_FAILED) 
            {

                APP_TCPIP_IFModules_Disable(netHandleWiFi);
                APP_TCPIP_IF_Down(netHandleWiFi);
                APP_TCPIP_IF_Up(netHandleWiFi);
                isWiFiPowerSaveConfigured = false;
                s_appData.state = APP_WIFI_CONFIG;
                break;
            } else if (s_app_get_param.conn.status == IWPRIV_CONNECTION_REESTABLISHED) {
               
                TCPIP_DHCP_Disable(netHandleWiFi);
                if(MyConfig.startFlags != TCPIP_NETWORK_CONFIG_IP_STATIC)
                {
                         // restart dhcp client and config power save
                        TCPIP_DHCP_Enable(netHandleWiFi);
                }
                isWiFiPowerSaveConfigured = false;
            }
            
            SYS_CMD_READY_TO_READ();

            /*
             * Following for loop is to deal with manually controlling
             * interface down/up (for example, through console commands
             * or web page).
             */
            nNets = TCPIP_STACK_NumberOfNetworksGet();
            for (i = 0; i < nNets; ++i)
            {
                TCPIP_NET_HANDLE netH = TCPIP_STACK_IndexToNet(i);
                if (!TCPIP_STACK_NetIsUp(netH) && wasNetUp[i])
                {
                    const char *netName = TCPIP_STACK_NetNameGet(netH);
                    wasNetUp[i] = false;
                    
                    APP_TCPIP_IFModules_Disable(netH);
                    if (IS_WF_INTF(netName))
                        isWiFiPowerSaveConfigured = false;
                }

                if ( TCPIP_STACK_NetIsUp(netH) && !wasNetUp[i] )
                {
                    wasNetUp[i] = true;
                    APP_TCPIP_IFModules_Enable(netH);
                }
            }

            /*
             * If we get a new IP address that is different than the default one,
             * we will run PowerSave configuration.
             */
            if (!isWiFiPowerSaveConfigured &&
                TCPIP_STACK_NetIsUp(netHandleWiFi) &&
                (TCPIP_STACK_NetAddress(netHandleWiFi) != defaultIPWiFi.Val)) 
            {
                APP_WIFI_PowerSave_Config(true);
                isWiFiPowerSaveConfigured = true;
            }

            APP_WIFI_DHCPS_Sync(netHandleWiFi);

            APP_OSAL_MUTEX_LOCK();
            /*
             * If the IP address of an interface has changed,
             * display the new value on the system console.
             */
            for (i = 0; i < nNets; ++i)
            {
                IPV4_ADDR ipAddr;
                TCPIP_NET_HANDLE netH = TCPIP_STACK_IndexToNet(i);
                ipAddr.Val = TCPIP_STACK_NetAddress(netH);
                if (dwLastIP[i].Val != ipAddr.Val)
                {
                        #if defined(WIRELESS_EVAL_BOARD)
                        LED2_OFF();
                        LED0_INV();
                        LCDErase();
                        strcpy((char *) LCDText, DRV_WIFI_DEFAULT_SSID);
                        sprintf((char *)LCDText+16, "%d.%d.%d.%d",  ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
                        LCDUpdate();
                        LCDBacklightON();
                        #endif
                        dwLastIP[i].Val = ipAddr.Val;
                        SYS_CONSOLE_PRINT("%s IP Address: %d.%d.%d.%d \r\n", TCPIP_STACK_NetNameGet(netH), ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
                        #if defined(WIRELESS_EVAL_BOARD)
                         startTick = SYS_TMR_TickCountGet();
                         LCDBacklightOFF();
                        #endif
                }
                 
            }

//            if (SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet() / 2ul)
            if (SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet() * 1ul)
            {
                startTick = SYS_TMR_TickCountGet();
//                if( TCPIP_DHCP_IsBound(TCPIP_STACK_IndexToNet(0)) )
                {
                        #if defined(WIRELESS_EVAL_BOARD)
                        LCDBacklightOFF();
                        #endif
                        s_LEDstate ^= BSP_LED_STATE_ON;
                }
                BSP_LEDStateSet(APP_LED_1, s_LEDstate);
            }
            APP_OSAL_MUTEX_UNLOCK();

            break;

        default:
            break;
    }
}

#if (OSAL_USE_RTOS == 1) /* It means FreeRTOS V8.x.x is used. */
static bool APP_OSAL_MutexInit(OSAL_MUTEX_HANDLE_TYPE *mutex)
{
    if (OSAL_MUTEX_Create(mutex) == OSAL_RESULT_TRUE)
        return true;
    else
        return false;
}

static void APP_OSAL_MutexLock(OSAL_MUTEX_HANDLE_TYPE *mutex, uint16_t waitMS)
{
    OSAL_MUTEX_Lock(mutex, waitMS);
}

static void APP_OSAL_MutexUnlock(OSAL_MUTEX_HANDLE_TYPE *mutex)
{
    OSAL_MUTEX_Unlock(mutex);
}
#endif

static void APP_CONSOLE_HeaderDisplay(void)
{
    #if defined(WIFI_TCPIP_WEB_SERVER_DEMO)
        SYS_CONSOLE_MESSAGE("\r\n====================================");
        SYS_CONSOLE_MESSAGE("\r\n*** Wi-Fi TCP/IP Web Server Demo ***");
        SYS_CONSOLE_MESSAGE("\r\n====================================\r\n");
    #else
        SYS_CONSOLE_MESSAGE("\r\n===================================");
        SYS_CONSOLE_MESSAGE("\r\n*** Microchip Wi-Fi TCP/IP Demo ***");
        SYS_CONSOLE_MESSAGE("\r\n===================================\r\n");
    #endif
}

static void APP_WIFI_IPv6MulticastFilter_Set(TCPIP_NET_HANDLE netH)
{
#if defined(TCPIP_STACK_USE_IPV6)
    const uint8_t *pMacAddr = TCPIP_STACK_NetAddressMac(netH);
    int i;
    uint8_t linkLocalSolicitedMulticastMacAddr[6];
    uint8_t solicitedNodeMulticastMACAddr[] = {0x33, 0x33, 0xff, 0x00, 0x00, 0x00};
    uint8_t allNodesMulticastMACAddr[] = {0x33, 0x33, 0x00, 0x00, 0x00, 0x01};

    linkLocalSolicitedMulticastMacAddr[0] = 0x33;
    linkLocalSolicitedMulticastMacAddr[1] = 0x33;
    linkLocalSolicitedMulticastMacAddr[2] = 0xff;

    for (i = 3; i < 6; i++)
        linkLocalSolicitedMulticastMacAddr[i] = pMacAddr[i];

    s_app_set_param.multicast.addr = linkLocalSolicitedMulticastMacAddr;
    iwpriv_set(MULTICASTFILTER_SET, &s_app_set_param);
    s_app_set_param.multicast.addr = solicitedNodeMulticastMACAddr;
    iwpriv_set(MULTICASTFILTER_SET, &s_app_set_param);
    s_app_set_param.multicast.addr = allNodesMulticastMACAddr;
    iwpriv_set(MULTICASTFILTER_SET, &s_app_set_param);
#endif
}

static void APP_WIFI_PowerSave_Config(bool enable)
{
#if (WF_DEFAULT_POWER_SAVE == WF_ENABLED)
    s_app_set_param.powerSave.enabled = enable;
    iwpriv_set(POWERSAVE_SET, &s_app_set_param);
#endif
}

static void APP_WIFI_DHCPS_Sync(TCPIP_NET_HANDLE netH)
{
#if defined(TCPIP_STACK_USE_DHCP_SERVER)
    bool updated;
    TCPIP_MAC_ADDR addr;

    s_app_get_param.clientInfo.addr = addr.v;
    iwpriv_get(CLIENTINFO_GET, &s_app_get_param);
    updated = s_app_get_param.clientInfo.updated;

    if (updated)
        TCPIP_DHCPS_LeaseEntryRemove(netH, (TCPIP_MAC_ADDR *)&addr);
#endif
}

static void APP_TCPIP_IFModules_Disable(TCPIP_NET_HANDLE netH)
{
    const char *netName = TCPIP_STACK_NetNameGet(netH);

    if (IS_WF_INTF(netName) && TCPIP_STACK_NetIsUp(netH)) {
        APP_WIFI_PowerSave_Config(false);
    }

    TCPIP_DHCP_Disable(netH);
    TCPIP_MDNS_ServiceDeregister(netH);
}

static void APP_TCPIP_IFModules_Enable(TCPIP_NET_HANDLE netH)
{
    int netIndex = TCPIP_STACK_NetIndexGet(netH);
    const char *netName = TCPIP_STACK_NetNameGet(netH);
    const char *netBiosName = TCPIP_STACK_NetBIOSName(netH);

//    if(MyConfig.startFlags != TCPIP_NETWORK_CONFIG_IP_STATIC)
        TCPIP_DHCP_Enable(netH);
    if (IS_WF_INTF(netName)) {
        APP_WIFI_IPv6MulticastFilter_Set(netH);
    }
    if (IS_NBNS_RUN()) {
        SYS_CONSOLE_PRINT("  Interface %s on host %s - NBNS enabled\r\n", netName, netBiosName);
    }
    if (IS_MDNS_RUN()) {
        char mDNSServiceName[] = "MyWebServiceNameX "; // base name of the service Must not exceed 16 bytes long
        // the last digit will be incremented by interface
        mDNSServiceName[sizeof(mDNSServiceName) - 2] = '1' + netIndex;
        TCPIP_MDNS_ServiceRegister(netH, mDNSServiceName, "_http._tcp.local", 80, ((const uint8_t *)"path=/index.htm"),
            1, NULL, NULL);
    }
}

static void APP_TCPIP_IF_Down(TCPIP_NET_HANDLE netH)
{
    TCPIP_STACK_NetDown(netH);
}

static void APP_TCPIP_IF_Up(TCPIP_NET_HANDLE netH)
{
    SYS_MODULE_OBJ tcpipStackObj;
    TCPIP_STACK_INIT tcpip_init_data;
    const TCPIP_NETWORK_CONFIG *pIfConf;
    uint16_t net_ix = TCPIP_STACK_NetIndexGet(netH);

    tcpipStackObj = TCPIP_STACK_Initialize(0, 0);
    TCPIP_STACK_InitializeDataGet(tcpipStackObj, &tcpip_init_data);
    pIfConf = tcpip_init_data.pNetConf + net_ix;
    TCPIP_STACK_NetUp(netH, pIfConf);
//    TCPIP_STACK_NetUp(netH, &MyConfig.netConfig);
}

/*******************************************************************************
 End of File
*/
