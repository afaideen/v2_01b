/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_tls_client.c

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

#include "app_tls_client.h"

int wolfSSL_Debugging_ON(void);
int wolfSSL_Debugging_OFF(void);

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define     PORTNUM                             443
#define     HOSTNAME0                     "myfreedomaintest.website"
//#define     HOSTNAME0                     "192.168.0.103"

#define     HOSTNAME1                     "api.weatherapi.com"
#define     apiWeatherKey                   "303fb9ce3b5f40c1ace11052221207"
#define     location_param                  "Segamat"

#define     HOSTNAME2                     "api.openweathermap.org"
#define     latitudeSepang                                      "2.902632"
#define     longitudeSepang                                     "101.634693"
#define     apiOpenWeatherMapKEY                      "892a3a878ae1aa72d5b34877caa5b239"

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

APP_TLS_CLIENT_DATA             app_tls_clientData;
char                                            networkBuffer[1024];
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

BYTE                     latitude[16];
BYTE                     longitude[16];
BYTE                     option_api;
BYTE                    *host0, *host1, *host2;


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_TLS_CLIENT_Initialize ( void )

  Remarks:
    See prototype in app_tls_client.h.
 */

void APP_TLS_CLIENT_Initialize ( void )
{
#if defined( DEBUG_WOLFSSL )
        wolfSSL_Debugging_ON();
//        wolfSSL_Debugging_OFF();
#endif
        
    /* Place the App state machine in its initial state. */
    app_tls_clientData.state = 0xff;

     //    OPTION 0
    strcpy((char*)app_tls_clientData.api0.hostname, HOSTNAME0);   
    host0 = &app_tls_clientData.api0.hostname[0];

    //    OPTION 1
    strcpy((char*)app_tls_clientData.api1.hostname, HOSTNAME1);
    strcpy((char*)app_tls_clientData.api1.api_key, apiWeatherKey);
    strcpy((char*)app_tls_clientData.api1.param.location, location_param);
    host1 = &app_tls_clientData.api1.hostname[0];
    
//    OPTION 2
    strcpy((char*)app_tls_clientData.api2.hostname, HOSTNAME2);
    strcpy((char*)app_tls_clientData.api2.api_key, apiOpenWeatherMapKEY);
    strcpy((char*)app_tls_clientData.api2.param.gps_lat, (const char*)latitudeSepang);
    strcpy((char*)app_tls_clientData.api2.param.gps_lng, (const char*)longitudeSepang);
    strcpy((char*)latitude, (const char*)latitudeSepang);
    strcpy((char*)longitude, (const char*)longitudeSepang);
    host2 = &app_tls_clientData.api2.hostname[0];
    
    app_tls_clientData.port = 443;
}


/******************************************************************************
  Function:
    void APP_TLS_CLIENT_Tasks ( void )

  Remarks:
    See prototype in app_tls_client.h.
 */

void APP_TLS_CLIENT_Tasks ( void )
{
    static uint32_t     startTick = 0, t_sw = 0;
    TCPIP_DNS_RESULT result;
    SYS_TMR_HANDLE delayHandle;
    static BYTE body_data[] = "{"
                                "\"key1\": \"Hi! Are you my server?\","
                                "\"key2\": 128.0123,"
                                "\"key3\": true"
                            "}";
     size_t size_dt;
    
    if(SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet()/2ul)
    {
        startTick = SYS_TMR_TickCountGet();           
        BSP_LED_2Toggle();               
    }
    
    if( !BSP_SWITCH_0StateGet()||!BSP_SWITCH_1StateGet() || !BSP_SWITCH_2StateGet() )
    {
        if(SYS_TMR_TickCountGet() - t_sw >= SYS_TMR_TickCounterFrequencyGet() * 0.25)
        {
            if(!BSP_SWITCH_0StateGet())
                option_api = 0;
            else if(!BSP_SWITCH_1StateGet())
                option_api = 1;
            else if(!BSP_SWITCH_2StateGet())
                option_api = 2;
            t_sw = SYS_TMR_TickCountGet();
            app_tls_clientData.state = APP_TLS_CLIENT_STATE_INIT;
            if(option_api == 0)
                while(!BSP_SWITCH_0StateGet());
            else if(option_api == 2)
                while(!BSP_SWITCH_1StateGet());
            else if(option_api == 2)
                while(!BSP_SWITCH_2StateGet());
            delayHandle = SYS_TMR_DelayMS(250); // 0.25 second delay. 
            while(SYS_TMR_DelayStatusGet(delayHandle) == false) // Code will block here
            {
                SYS_TMR_Tasks(sysObj.sysTmr);
            }
        }
    }
    else
        t_sw = SYS_TMR_TickCountGet();

    /* Check the application's current state. */
    switch ( app_tls_clientData.state )
    {
        /* Application's initial state. */
        case APP_TLS_CLIENT_STATE_INIT:
        {
            app_tls_clientData.testStart = SYS_TMR_SystemCountGet();
            app_tls_clientData.dnsComplete = 0;
            app_tls_clientData.connectionOpened = 0;
            app_tls_clientData.sslNegComplete = 0;
            app_tls_clientData.firstRxDataPacket = 0;
            app_tls_clientData.lastRxDataPacket = 0;
            app_tls_clientData.rawBytesReceived = 0;
            app_tls_clientData.rawBytesSent = 0;
            app_tls_clientData.clearBytesReceived = 0;
            app_tls_clientData.clearBytesSent = 0;
            
            app_tls_clientData.ipMode = 4;
            if(option_api == 0)
            {
                app_tls_clientData.host = host0;
//                app_tls_clientData.port = 5000;
                app_tls_clientData.port = PORTNUM;
            }
            else  if(option_api == 1){
                app_tls_clientData.host = host1;
                app_tls_clientData.port = 443;
            }
            else if(option_api == 2)
            {
                app_tls_clientData.host = host2;
                app_tls_clientData.port = 443;
            }
            result = TCPIP_DNS_Resolve((char*)app_tls_clientData.host, TCPIP_DNS_TYPE_A);
            app_tls_clientData.queryState = 4;
            app_tls_clientData.state = APP_TLS_CLIENT_WAIT_ON_DNS;
        }

        case APP_TLS_CLIENT_WAIT_ON_DNS:
        {
                result = TCPIP_DNS_IsResolved(
                        (char*)app_tls_clientData.host, 
                        &app_tls_clientData.address, 
                        app_tls_clientData.ipMode == 4 ? IP_ADDRESS_TYPE_IPV4 : IP_ADDRESS_TYPE_IPV6
                        );
                switch (result) {
                    case TCPIP_DNS_RES_PENDING:                
                        break;
                    case TCPIP_DNS_RES_OK:
                        SYS_CONSOLE_PRINT("DNS Resolved IPv4 Address: %d.%d.%d.%d for host '%s'\r\n",
                                    app_tls_clientData.address.v4Add.v[0],
                                    app_tls_clientData.address.v4Add.v[1],
                                    app_tls_clientData.address.v4Add.v[2],
                                    app_tls_clientData.address.v4Add.v[3],
                                    app_tls_clientData.host);
                        app_tls_clientData.state = APP_TLS_CLIENT_START_CONNECTION;
                        break;
                    default:
                        if (app_tls_clientData.queryState == 4 || app_tls_clientData.ipMode == 6) {
                            SYS_CONSOLE_PRINT("DNS Is Resolved returned %d Aborting\r\n", result);
                            app_tls_clientData.state = APP_TLS_CLIENT_WAITING_FOR_COMMAND;
                            break;
                        } else {
                            SYS_CONSOLE_PRINT("DNS Is Resolved returned %d trying IPv4 Address\r\n", result);
                            result = TCPIP_DNS_Resolve((char*)app_tls_clientData.host, TCPIP_DNS_TYPE_A);
                            app_tls_clientData.queryState = 4;
                            if (result >= 0) {
                                app_tls_clientData.state = APP_TLS_CLIENT_WAIT_ON_DNS;
                            } else {
                                SYS_CONSOLE_PRINT("DNS Query returned %d Aborting\r\n", result);
                                app_tls_clientData.state = APP_TLS_CLIENT_WAITING_FOR_COMMAND;
                            }
                        }
                        break;
                }
        
                break;
        }
        case APP_TLS_CLIENT_START_CONNECTION:
            // If we're here it means that we have a proper address.
            app_tls_clientData.dnsComplete = SYS_TMR_SystemCountGet();
            SYS_CONSOLE_PRINT("Starting TCP/IPv4 Connection to : %d.%d.%d.%d port  '%d'\r\n",
                        app_tls_clientData.address.v4Add.v[0],
                        app_tls_clientData.address.v4Add.v[1],
                        app_tls_clientData.address.v4Add.v[2],
                        app_tls_clientData.address.v4Add.v[3],
                        app_tls_clientData.port);
            app_tls_clientData.socket = NET_PRES_SocketOpen(0,
                        NET_PRES_SKT_UNENCRYPTED_STREAM_CLIENT,
                        IP_ADDRESS_TYPE_IPV4,
                        app_tls_clientData.port,
                        (NET_PRES_ADDRESS *) & app_tls_clientData.address,
                        NULL);
            NET_PRES_SocketWasReset(app_tls_clientData.socket);
            if (app_tls_clientData.socket == INVALID_SOCKET) {
                SYS_CONSOLE_MESSAGE("Could not create socket - aborting\r\n");
                app_tls_clientData.state = APP_TLS_CLIENT_WAITING_FOR_COMMAND;
            }
            SYS_CONSOLE_MESSAGE("Starting connection\r\n");
            app_tls_clientData.state = APP_TLS_CLIENT_WAIT_FOR_CONNECTION;
            break;
        case APP_TLS_CLIENT_WAIT_FOR_CONNECTION:
            if (!NET_PRES_SocketIsConnected(app_tls_clientData.socket)) {
                break;
            }
            app_tls_clientData.connectionOpened = SYS_TMR_SystemCountGet();
            SYS_CONSOLE_MESSAGE("Connection Opened: Starting SSL Negotiation\r\n");
            if (!NET_PRES_SocketEncryptSocket(app_tls_clientData.socket)) {
                SYS_CONSOLE_MESSAGE("SSL Create Connection Failed - Aborting\r\n");
                app_tls_clientData.state = APP_TLS_CLIENT_CLOSE_CONNECTION;
            } else {
                app_tls_clientData.state = APP_TLS_CLIENT_WAIT_FOR_SSL_CONNECT;

            }
            break;
        case APP_TLS_CLIENT_WAIT_FOR_SSL_CONNECT:
            if (NET_PRES_SocketIsNegotiatingEncryption(app_tls_clientData.socket)) {
                break;
            }
            if (!NET_PRES_SocketIsSecure(app_tls_clientData.socket)) 
            {
                SYS_CONSOLE_MESSAGE("SSL Connection Negotiation Failed - Aborting\r\n");
                app_tls_clientData.state = APP_TLS_CLIENT_CLOSE_CONNECTION;
                break;
            }
            SYS_CONSOLE_MESSAGE("SSL Connection Opened: Starting Clear Text Communication\r\n");
            app_tls_clientData.cyaSSLLogEnabled = 0;
            app_tls_clientData.sslNegComplete = SYS_TMR_SystemCountGet();
            
            app_tls_clientData.state = APP_TLS_CLIENT_SEND_REQUEST_SSL;
            break;
       
        case APP_TLS_CLIENT_SEND_REQUEST_SSL:
        
            if(option_api == 0)
            {
                    //Option 0: Local private server https://192.168.0.103:5000/post/test
                    size_dt = strlen((const char*)body_data);
                    sprintf(networkBuffer, 
                    "GET /post/test HTTP/1.1\r\n"
                    "Host: %s:%d\r\n"
                    "Accept: */*\r\n"
                    "Connection: keep-alive\r\n"
                    "Content-Length: %d"
                    "\r\n\r\n"
                    "%s"
                    , HOSTNAME0
                    , PORTNUM
                    , (int)size_dt
                    , body_data);
            }
            if(option_api == 1)
            {
                //https://api.weatherapi.com/v1/current.json?q=Segamat&key=303fb9ce3b5f40c1ace11052221207
                //Option 1: Weather Api
                sprintf(networkBuffer, "GET /v1/current.json?q=%s&key=%s HTTP/1.1\r\n"
                        "Host: %s\r\n"
                        "Accept: application/json\r\n"
                        "Connection: close\r\n"
                        "\r\n",
                        location_param, app_tls_clientData.api1.api_key, host1);
            }
            else if(option_api == 2)
            {
                //https://api.openweathermap.org/data/2.5/weather?lat=2.902632&lon=101.634693&appid=892a3a878ae1aa72d5b34877caa5b239
                //Option 2:Open Weather Map Api
                sprintf(networkBuffer, "GET /data/2.5/weather?lat=%s&lon=%s&appid=%s HTTP/1.1\r\n"
                        "Host: %s\r\n"
                        //"Accept: text/plain\r\n"
                        "Connection: close\r\n"
                        "\r\n",
                        latitude, longitude, app_tls_clientData.api2.api_key, host2);
            }
            NET_PRES_SocketWrite(app_tls_clientData.socket, (uint8_t*) networkBuffer, strlen(networkBuffer));
            SYS_CONSOLE_MESSAGE(networkBuffer);
            app_tls_clientData.clearBytesSent += strlen(networkBuffer);
            app_tls_clientData.rawBytesSent += strlen(networkBuffer);
            app_tls_clientData.state = APP_TLS_CLIENT_WAIT_FOR_RESPONSE_SSL;
            break;
        case APP_TLS_CLIENT_WAIT_FOR_RESPONSE_SSL:
            
//            v = NET_PRES_SocketReadIsReady(app_tls_clientData.socket);
//            if (v == 0) {
            if (NET_PRES_SocketReadIsReady(app_tls_clientData.socket) == 0) {
                if (NET_PRES_SocketWasReset(app_tls_clientData.socket)) {
                    app_tls_clientData.state = APP_TLS_CLIENT_CLOSE_CONNECTION;
                }
                else if (!NET_PRES_SocketIsConnected(app_tls_clientData.socket)) {
                    app_tls_clientData.state = APP_TLS_CLIENT_CLOSE_CONNECTION;
                    break;
                }
                break;
            }
            if (app_tls_clientData.firstRxDataPacket == 0) {
                app_tls_clientData.firstRxDataPacket = SYS_TMR_SystemCountGet();
            }
            app_tls_clientData.lastRxDataPacket = SYS_TMR_SystemCountGet();
            memset(networkBuffer, 0, sizeof (networkBuffer));
            uint16_t res = NET_PRES_SocketRead(app_tls_clientData.socket, (uint8_t*) networkBuffer, sizeof (networkBuffer));
            SYS_CONSOLE_MESSAGE(networkBuffer);
            SYS_CONSOLE_MESSAGE("\r\n");
            
            if (strstr(networkBuffer, "/html") != 0 || strstr(networkBuffer, "}}") != 0) {
                app_tls_clientData.state = APP_TLS_CLIENT_CLOSE_CONNECTION;
            }
            app_tls_clientData.clearBytesReceived += res;
            app_tls_clientData.rawBytesReceived += res;
            break;
        
        case APP_TLS_CLIENT_CLOSE_CONNECTION:
        
            NET_PRES_SocketClose(app_tls_clientData.socket);
            SYS_CONSOLE_MESSAGE("Connection Closed\r\n");
            app_tls_clientData.state = APP_TLS_CLIENT_WAITING_FOR_COMMAND;
            break;
        
        case APP_TLS_CLIENT_WAITING_FOR_COMMAND:


        /* The default state should never be executed. */
        default:
            break;
    }
}

 

/*******************************************************************************
 End of File
 */
