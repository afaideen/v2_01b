<#--
/*******************************************************************************
  TCP Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    tcp.h.ftl

  Summary:
    TCP Freemarker Template File

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
-->
<#if CONFIG_TCPIP_USE_TCP == true>
/*** TCP Configuration ***/
#define TCPIP_TCP_MAX_SEG_SIZE_TX		        	${CONFIG_TCPIP_TCP_MAX_SEG_SIZE_TX}
#define TCPIP_TCP_MAX_SEG_SIZE_RX_LOCAL		    		${CONFIG_TCPIP_TCP_MAX_SEG_SIZE_RX_LOCAL}
#define TCPIP_TCP_MAX_SEG_SIZE_RX_NON_LOCAL			${CONFIG_TCPIP_TCP_MAX_SEG_SIZE_RX_NON_LOCAL}
#define TCPIP_TCP_SOCKET_DEFAULT_TX_SIZE			${CONFIG_TCPIP_TCP_SOCKET_DEFAULT_TX_SIZE}
#define TCPIP_TCP_SOCKET_DEFAULT_RX_SIZE			${CONFIG_TCPIP_TCP_SOCKET_DEFAULT_RX_SIZE}
<#if CONFIG_TCPIP_TCP_DYNAMIC_OPTIONS == true>
#define TCPIP_TCP_DYNAMIC_OPTIONS             			true
<#else>
#define TCPIP_TCP_DYNAMIC_OPTIONS             			false
</#if>
#define TCPIP_TCP_START_TIMEOUT_VAL		        	${CONFIG_TCPIP_TCP_START_TIMEOUT_VAL}
#define TCPIP_TCP_DELAYED_ACK_TIMEOUT		    		${CONFIG_TCPIP_TCP_DELAYED_ACK_TIMEOUT}
#define TCPIP_TCP_FIN_WAIT_2_TIMEOUT		    		${CONFIG_TCPIP_TCP_FIN_WAIT_2_TIMEOUT}
#define TCPIP_TCP_KEEP_ALIVE_TIMEOUT		    		${CONFIG_TCPIP_TCP_KEEP_ALIVE_TIMEOUT}
#define TCPIP_TCP_CLOSE_WAIT_TIMEOUT		    		${CONFIG_TCPIP_TCP_CLOSE_WAIT_TIMEOUT}
#define TCPIP_TCP_MAX_RETRIES		            		${CONFIG_TCPIP_TCP_MAX_RETRIES}
#define TCPIP_TCP_MAX_UNACKED_KEEP_ALIVES			${CONFIG_TCPIP_TCP_MAX_UNACKED_KEEP_ALIVES}
#define TCPIP_TCP_MAX_SYN_RETRIES		        	${CONFIG_TCPIP_TCP_MAX_SYN_RETRIES}
#define TCPIP_TCP_AUTO_TRANSMIT_TIMEOUT_VAL			${CONFIG_TCPIP_TCP_AUTO_TRANSMIT_TIMEOUT_VAL}
#define TCPIP_TCP_WINDOW_UPDATE_TIMEOUT_VAL			${CONFIG_TCPIP_TCP_WINDOW_UPDATE_TIMEOUT_VAL}
#define TCPIP_TCP_MAX_SOCKETS		                ${CONFIG_TCPIP_TCP_MAX_SOCKETS}
#define TCPIP_TCP_TASK_TICK_RATE		        	${CONFIG_TCPIP_TCP_TASK_TICK_RATE}
#define TCPIP_TCP_MSL_TIMEOUT		        	    ${CONFIG_TCPIP_TCP_MSL_TIMEOUT}
#define TCPIP_TCP_QUIET_TIME		        	    ${CONFIG_TCPIP_TCP_QUIET_TIME}
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->
