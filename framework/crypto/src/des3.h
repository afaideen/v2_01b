/**************************************************************************
  Crypto Framework Library Header

  Company:
    Microchip Technology Inc.

  File Name:
    des3.h
  
  Summary:
    Crypto Framework Library header for cryptographic functions.

  Description:
    This header file contains function prototypes and definitions of
    the data types and constants that make up the Cryptographic Framework
    Library for PIC32 families of Microchip microcontrollers.
**************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
Copyright ? 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND,
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
//DOM-IGNORE-END


#ifndef CRYPT_DES3_H
#define CRYPT_DES3_H

#include "crypto/src/types.h"

#ifndef NO_DES3

#ifdef HAVE_FIPS
/* included for fips @wc_fips */
#include "crypto/src/des3.h"
#endif



#ifdef __cplusplus
    extern "C" {
#endif

#ifndef HAVE_FIPS /* to avoid redifinition of macros */
#define WOLFSSL_3DES_CAVIUM_MAGIC 0xBEEF0003

enum {
    DES_ENC_TYPE    = 2,     /* cipher unique type */
    DES3_ENC_TYPE   = 3,     /* cipher unique type */
    DES_BLOCK_SIZE  = 8,
    DES_KS_SIZE     = 32,

    DES_ENCRYPTION  = 0,
    DES_DECRYPTION  = 1
};

#define DES_IVLEN 8
#define DES_KEYLEN 8
#define DES3_IVLEN 8
#define DES3_KEYLEN 24


#ifdef STM32F2_CRYPTO
enum {
    DES_CBC = 0,
    DES_ECB = 1
};
#endif


/* DES encryption and decryption */
typedef struct Des {
    word32 reg[DES_BLOCK_SIZE / sizeof(word32)];      /* for CBC mode */
    word32 tmp[DES_BLOCK_SIZE / sizeof(word32)];      /* same         */
    word32 key[DES_KS_SIZE];
} Des;


/* DES3 encryption and decryption */
typedef struct Des3 {
    word32 key[3][DES_KS_SIZE];
    word32 reg[DES_BLOCK_SIZE / sizeof(word32)];      /* for CBC mode */
    word32 tmp[DES_BLOCK_SIZE / sizeof(word32)];      /* same         */
#ifdef HAVE_CAVIUM
    int     devId;           /* nitrox device id */
    word32  magic;           /* using cavium magic */
    word64  contextHandle;   /* nitrox context memory handle */
#endif
} Des3;
#endif /* HAVE_FIPS */

WOLFSSL_API int  wc_Des_SetKey(Des* des, const byte* key,
                               const byte* iv, int dir);
WOLFSSL_API void wc_Des_SetIV(Des* des, const byte* iv);
WOLFSSL_API int  wc_Des_CbcEncrypt(Des* des, byte* out,
                                   const byte* in, word32 sz);
WOLFSSL_API int  wc_Des_CbcDecrypt(Des* des, byte* out,
                                   const byte* in, word32 sz);
WOLFSSL_API int  wc_Des_EcbEncrypt(Des* des, byte* out,
                                   const byte* in, word32 sz);

WOLFSSL_API int  wc_Des3_SetKey(Des3* des, const byte* key,
                                const byte* iv,int dir);
WOLFSSL_API int  wc_Des3_SetIV(Des3* des, const byte* iv);
WOLFSSL_API int  wc_Des3_CbcEncrypt(Des3* des, byte* out,
                                    const byte* in,word32 sz);
WOLFSSL_API int  wc_Des3_CbcDecrypt(Des3* des, byte* out,
                                    const byte* in,word32 sz);

#ifdef HAVE_CAVIUM
    WOLFSSL_API int  wc_Des3_InitCavium(Des3*, int);
    WOLFSSL_API void wc_Des3_FreeCavium(Des3*);
#endif


#ifdef __cplusplus
    } /* extern "C" */
#endif

#endif /* NO_DES3 */
#endif /* CRYPT_DES3_H */

