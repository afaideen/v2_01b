<#--
/*******************************************************************************
  RTCC System Task File

  File Name:
    sys_rtcc_tasks.c.ftl

  Summary:
    This file contains source code necessary to run the RTCC system service.

  Description:
    This file contains source code necessary to run the system.  
 *******************************************************************************/

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
 -->
<#if !(SYS_RTCC_TASKS?has_content)>
<#assign SYS_RTCC_TASKS = "TASK_CALL_NO_RTOS">
</#if>
<#if (SYS_RTCC_TASKS == "PROTO") && (CONFIG_SYS_RTCC_RTOS == "Standalone")>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void _SYS_RTCC_Tasks(ULONG thread_input);
<#else>
void _SYS_RTCC_Tasks(void);
</#if>
</#if>
<#if SYS_RTCC_TASKS == "CREATE_TASK">
<#if CONFIG_SYS_RTCC_RTOS == "Standalone">
<@RTOS_TASK_CREATE RTOS_NAME=CONFIG_3RDPARTY_RTOS_USED TASK_FUNC_NAME="_SYS_RTCC_Tasks" TASK_NAME="Sys RTCC Tasks" TASK_PRI=CONFIG_SYS_RTCC_RTOS_TASK_PRIORITY TASK_STK_SZ=CONFIG_SYS_RTCC_RTOS_TASK_SIZE/>

</#if>
</#if>
<#if SYS_RTCC_TASKS == "TASK_CALL_NO_RTOS">
<#if !CONFIG_SYS_RTCC_INTERRUPT_MODE>
    SYS_RTCC_Tasks(sysObj.sysRtcc);
</#if>
<#elseif (SYS_RTCC_TASKS == "TASK_CALL" && CONFIG_SYS_RTCC_RTOS != "Standalone")>
        SYS_RTCC_Tasks(sysObj.sysRtcc);
</#if>
<#if SYS_RTCC_TASKS == "LOCAL_FUNCTION">
 <#if CONFIG_SYS_RTCC_RTOS == "Standalone">
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void _SYS_RTCC_Tasks(ULONG thread_input)
<#else>
void _SYS_RTCC_Tasks(void)
</#if>
 {
<#if CONFIG_3RDPARTY_RTOS_USED == "uC/OS-III">
    OS_ERR os_err;
</#if>

    while(1)
    {
        /* Maintain the RTCC system state machine. */
        SYS_RTCC_Tasks(sysObj.sysRtcc);
<@RTOS_TASK_DELAY RTOS_NAME=CONFIG_3RDPARTY_RTOS_USED TASK_DELAY=CONFIG_SYS_RTCC_RTOS_DELAY/>
    }
 }
 </#if>
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
