/* 
 * File:   MiWiHelper.h
 * Author: han
 *
 * Created on 21 February 2022, 11:04
 */

#ifndef MIWIHELPER_H
#define	MIWIHELPER_H

#include "GenericTypeDefs.h"
#include "WirelessProtocols/MCHP_API.h"
#include "WirelessProtocols/Console.h"

BOOL CreateNewConnectionWithLeastNoise(DWORD scan_chnl);
BOOL CreateNewConnectionAtChannel(BYTE channel);
//BYTE JoinAvailableChannel(BYTE channel);
BYTE JoinAvailableChannel(DWORD channel, WORD short_addr);
WORD AutoSearchActiveConnection(DWORD scan_chnl);

#endif	/* MIWIHELPER_H */

