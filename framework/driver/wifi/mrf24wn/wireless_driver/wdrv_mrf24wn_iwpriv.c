/*******************************************************************************
  MRF24WN Private Configuration Support

  File Name:
    wdrv_mrf24wn_iwpriv.c

  Summary:
    Configure optional (private) parameters of MRF24WN driver.

  Description:
    Functions in this module support the connection process for the
    MRF24W.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

/******************/
/*    INCLUDES    */
/******************/
#include "wdrv_mrf24wn_iwpriv.h"
#include "wdrv_mrf24wn_scan_helper.h"
#include "wdrv_mrf24wn_priv.h"
#include "wdrv_mrf24wn_api.h"

/*******************/
/*    VARIABLES    */
/*******************/
static uint8_t s_prescan_allowed = false;
static uint8_t s_prescan_inprogress = false; // WF_PRESCAN - This is used only to allow prescan once.

static bool _prescan_isfinished(void)
{
    if (s_prescan_inprogress && g_wdrv_priv.isScanDone) {
        s_prescan_inprogress = false;
        return true;
    }
    return false;
}

static IWPRIV_SCAN_STATUS _scanstatus_get(void)
{
    IWPRIV_SCAN_STATUS ret;

    WDRV_UsecDelay(1); // solving an application use case
    if(IS_SCAN_IN_PROGRESS(g_scanStatus.scanState)) {
        ret = IWPRIV_SCAN_IN_PROGRESS;
    } else if(IS_SCAN_STATE_DISPLAY(g_scanStatus.scanState) && (g_scanStatus.numberOfResults > 0)){
        ret = IWPRIV_SCAN_SUCCESSFUL;
    } else if (IS_SCAN_STATE_DISPLAY(g_scanStatus.scanState) && (g_scanStatus.numberOfResults == 0)) {
        ret = IWPRIV_SCAN_NO_AP_FOUND;
    } else {
        ret = IWPRIV_SCAN_IDLE;
    }
    return ret;
}

static void *_scanresult_get(uint16_t index)
{
    IWPRIV_GET_PARAM get_param;

    iwpriv_get(SCANRESULTS_COUNT_GET, &get_param);
    if (index < get_param.scan.numberOfResults &&
            index < (uint16_t)_WDRV_ScanResultsBufferSize_Get())
        return &(g_scanResults[index]);
    else
        return NULL;
}

static void _ssid_get(uint8_t *ssid, uint8_t *ssidLen)
{
    memcpy(ssid, WDRV_CONFIG_PARAMS(ssid), WDRV_CONFIG_PARAMS(ssidLen));
    ssid[WDRV_CONFIG_PARAMS(ssidLen)] = 0;
    *ssidLen = WDRV_CONFIG_PARAMS(ssidLen);
}

static void _leftclient_get(bool *updated, uint8_t *addr)
{
    bool connected;

    *updated = false;
    if (ClientCacheUpdated(&connected, addr))
        *updated = connected ? false: true;
}

static IWPRIV_CONN_STATUS _connstatus_get(void)
{
    IWPRIV_CONN_STATUS res = IWPRIV_CONNECTION_FAILED;
    WDRV_CONNECTION_STATES conn_status;

    conn_status = WDRV_ConnectStatus_Get();
    switch (conn_status) {
    case WDRV_CSTATE_NOT_CONNECTED:
        res = IWPRIV_CONNECTION_IDLE;
        break;
    case WDRV_CSTATE_CONNECTED_INFRASTRUCTURE:
        if (g_wdrv_priv.isConnReestablished) {
            g_wdrv_priv.isConnReestablished = false;
            res = IWPRIV_CONNECTION_REESTABLISHED;
            break;
        }
    case WDRV_CSTATE_CONNECTED_ADHOC:
        res = IWPRIV_CONNECTION_SUCCESSFUL;
        break;
    case WDRV_CSTATE_CONNECTION_IN_PROGRESS:
    case WDRV_CSTATE_RECONNECTION_IN_PROGRESS:
    case WDRV_CSTATE_CONNECTION_TEMPORARY_LOST:
        res = IWPRIV_CONNECTION_IN_PROGRESS;
        break;
    case WDRV_CSTATE_CONNECTION_PERMANENTLY_LOST:
        res = IWPRIV_CONNECTION_FAILED;
        break;
    default:
        WDRV_ASSERT(false, "Undefined connection status.");
    }
    return res;
}

static uint8_t _initstatus_get(void)
{
    if (g_wdrv_priv.initDriverDone)
        return IWPRIV_READY;
    else
        return IWPRIV_IN_PROGRESS;
}

static bool _is_servermode(void)
{
    if (WDRV_CONFIG_PARAMS(networkType) == WDRV_NETWORK_TYPE_SOFT_AP ||
            WDRV_CONFIG_PARAMS(networkType) == WDRV_NETWORK_TYPE_ADHOC)
        return true;
    else
        return false;
}

static void _prescan_option_set(bool scan)
{
    if (scan) {
        s_prescan_allowed = true;
        s_prescan_inprogress = false;
    } else {
        s_prescan_allowed = false;
        s_prescan_inprogress = false;
    }
}

static void _config_write(void *wifi_config)
{
    memcpy(p_wdrv_configData, wifi_config, sizeof(WDRV_CONFIG_DATA));
    //WDRV_CONFIG_DataSave();
}

static void _nettype_set(uint8_t netType)
{
    if (netType == WDRV_NETWORK_TYPE_INFRASTRUCTURE) {
        WDRV_EXT_CmdNetModeBSSSet();
    } else if (netType == WDRV_NETWORK_TYPE_ADHOC) {
        WDRV_EXT_CmdNetModeIBSSSet();
    } else if (netType == WDRV_NETWORK_TYPE_SOFT_AP) {
        WDRV_EXT_CmdNetModeAPSet();
    }
}

static IWPRIV_STATUS _mcastfilter_set(uint8_t *addr)
{
    if (WDRV_MRF24WN_MulticastFilterSet(NULL, (TCPIP_MAC_ADDR *)addr) == TCPIP_MAC_RES_OK)
        return IWPRIV_READY;
    return IWPRIV_ERROR;
}

static void _prescan_start(void)
{
    s_prescan_inprogress = true;
    WDRV_ScanStart();
}

static void _scan_start(void)
{
    WDRV_ScanStart();
    SYS_CONSOLE_MESSAGE("Scan started . . .\r\n");
}

static IWPRIV_STATUS _scanresults_display(void)
{
    if (IS_SCAN_STATE_DISPLAY(g_scanStatus.scanState)) {
        WDRV_ScanResultsDisplayManager();
        return IWPRIV_IN_PROGRESS;
    }
    return IWPRIV_READY;
}

static IWPRIV_STATUS _scanresults_save(void)
{
    static uint8_t saveIdx = 0;

    if ((g_scanStatus.numberOfResults == 0) ||
        (IS_SCAN_IN_PROGRESS(g_scanStatus.scanState)) ||
        (!IS_SCAN_STATE_VALID(g_scanStatus.scanState)))
        return IWPRIV_ERROR;
    if (saveIdx < g_scanStatus.numberOfResults) {
        WDRV_ScanResultsSaveManager(saveIdx++);
        return IWPRIV_IN_PROGRESS;
    } else if (saveIdx == g_scanStatus.numberOfResults) {
        saveIdx = 0;
        return IWPRIV_READY;
    } else {
        saveIdx = 0;
        return IWPRIV_ERROR;
    }
}

void iwpriv_get(IWPRIV_CMD cmd, IWPRIV_GET_PARAM *param)
{
    switch (cmd) {
    case PRESCAN_OPTION_GET:
        param->scan.prescanAllowed = s_prescan_allowed;
        break;
    case PRESCAN_ISFINISHED_GET:
        param->scan.prescanFinished = _prescan_isfinished();
        break;
    case SCANSTATUS_GET:
        param->scan.scanStatus = _scanstatus_get();
        break;
    case SCANRESULT_GET:
        param->scan.data = _scanresult_get(param->scan.index);
        break;
    case SCANRESULTS_COUNT_GET:
        param->scan.numberOfResults = g_scanStatus.numberOfResults;
        break;
    case CONFIG_GET:
        memcpy(param->config.data, p_wdrv_configData, sizeof(WDRV_CONFIG_DATA));
        break;
    case SSID_GET:
        _ssid_get(param->ssid.ssid,  &param->ssid.ssidLen);
        break;
    case NETWORKTYPE_GET:
        param->netType.type = WDRV_CONFIG_PARAMS(networkType);
        break;
    case CLIENTINFO_GET:
        _leftclient_get(&param->clientInfo.updated, param->clientInfo.addr);
        break;
    case CONNSTATUS_GET:
        param->conn.status = _connstatus_get();
        break;
    case DEVICEINFO_GET:
        ((WDRV_DEVICE_INFO *)param->devInfo.data)->deviceType = WDRV_MRF24WN0M_DEVICE;
        break;
    case INITSTATUS_GET:
        param->driverStatus.initStatus = _initstatus_get();
        break;
    case OPERATIONMODE_GET:
        param->opMode.isServer = _is_servermode();
        break;
    default:
        WDRV_ASSERT(false, "Invalid iwpriv get command");
        break;
    }
}

void iwpriv_set(IWPRIV_CMD cmd, IWPRIV_SET_PARAM *param)
{
    switch (cmd) {
    case PRESCAN_OPTION_SET:
        _prescan_option_set(param->scan.prescanAllowed);
        break;
    case CONFIG_SET:
        _config_write(param->config.data);
        break;
    case SSID_SET:
        WDRV_EXT_CmdSSIDSet(param->ssid.ssid, param->ssid.ssidLen);
        break;
    case NETWORKTYPE_SET:
        _nettype_set(param->netType.type);
        break;
    case ADHOCCTX_SET:
        /* TODO: implementation is TBD */
        break;
    case INITCONN_OPTION_SET:
        g_wdrv_priv.initConn = param->conn.initConnAllowed;
        break;
    case MULTICASTFILTER_SET:
        param->multicast.status = _mcastfilter_set(param->multicast.addr);
        break;
    case POWERSAVE_SET:
        WDRV_EXT_CmdPowerSavePut(param->powerSave.enabled);
        break;
    default:
        WDRV_ASSERT(false, "Invalid iwpriv set command");
        break;
    }
}

void iwpriv_execute(IWPRIV_CMD cmd, IWPRIV_EXECUTE_PARAM *param)
{
    switch (cmd) {
    case PRESCAN_START:
        _prescan_start();
        break;
    case SCAN_START:
        _scan_start();
        break;
    case SCANRESULTS_DISPLAY:
        param->scan.displayStatus = _scanresults_display();
        break;
    case SCANRESULTS_SAVE:
        param->scan.saveStatus = _scanresults_save();
        break;
    default:
        WDRV_ASSERT(false, "Invalid iwpriv execute command");
        break;
    }
}

// DOM-IGNORE-END
