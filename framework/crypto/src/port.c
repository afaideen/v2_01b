/**************************************************************************
  Crypto Framework Library Source

  Company:
    Microchip Technology Inc.

  File Name:
    port.c
  
  Summary:
    Crypto Framework Libarary source for cryptographic functions.

  Description:
    This source file contains functions that make up the Cryptographic 
	Framework Library for PIC32 families of Microchip microcontrollers.
**************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
File Name:  port.c
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

#ifdef HAVE_CONFIG_H
    #include "config.h"
#endif
#include "system_config.h"

#include "crypto/src/settings.h"
#include "crypto/src/types.h"
#include "crypto/src/error-crypt.h"
#include "crypto/src/logging.h"

/* IPP header files for library initialization */
#ifdef HAVE_FAST_RSA
#include <ipp.h>
#include <ippcp.h>
#endif

#ifdef _MSC_VER
    /* 4996 warning to use MS extensions e.g., strcpy_s instead of strncpy */
    #pragma warning(disable: 4996)
#endif


/* Used to initialize state for wolfcrypt
   return 0 on success
 */
int wolfCrypt_Init()
{
    int ret = 0;
    #if WOLFSSL_CRYPT_HW_MUTEX
        /* If crypto hardware mutex protection is enabled, then initialize it */
        wolfSSL_CryptHwMutexInit();
    #endif

    /* if defined have fast RSA then initialize Intel IPP */
    #ifdef HAVE_FAST_RSA
        WOLFSSL_MSG("Attempting to use optimized IPP Library");
        if ((ret = ippInit()) != ippStsNoErr) {
            /* possible to get a CPU feature support status on optimized IPP
              library but still use default library and see competitve speeds */
            WOLFSSL_MSG("Warning when trying to set up optimization");
            WOLFSSL_MSG(ippGetStatusString(ret));
            WOLFSSL_MSG("Using default fast IPP library");
            ret = 0;
        }
    #endif

    return ret;
}


#if WOLFSSL_CRYPT_HW_MUTEX
/* Mutex for protection of cryptograpghy hardware */
static wolfSSL_Mutex wcCryptHwMutex;
static int wcCryptHwMutexInit = 0;

int wolfSSL_CryptHwMutexInit(void) {
    int ret = 0;
    if(wcCryptHwMutexInit == 0) {
        ret = InitMutex(&wcCryptHwMutex);
        if(ret == 0) {
            wcCryptHwMutexInit = 1;
        }
    }
    return ret;
}

int wolfSSL_CryptHwMutexLock(void) {
    int ret = BAD_MUTEX_E;

    /* Make sure HW Mutex has been initialized */
    wolfSSL_CryptHwMutexInit();

    if(wcCryptHwMutexInit) {
        ret = LockMutex(&wcCryptHwMutex);
    }
    return ret;
}

int wolfSSL_CryptHwMutexUnLock(void) {
    int ret = BAD_MUTEX_E;
    
    if(wcCryptHwMutexInit) {
        ret = UnLockMutex(&wcCryptHwMutex);
    }
    return ret;
}
#endif /* WOLFSSL_CRYPT_HW_MUTEX */


#ifdef SINGLE_THREADED

int InitMutex(wolfSSL_Mutex* m)
{
    (void)m;
    return 0;
}


int FreeMutex(wolfSSL_Mutex *m)
{
    (void)m;
    return 0;
}


int LockMutex(wolfSSL_Mutex *m)
{
    (void)m;
    return 0;
}


int UnLockMutex(wolfSSL_Mutex *m)
{
    (void)m;
    return 0;
}

#else /* MULTI_THREAD */

    #if defined(FREERTOS)  || defined(FREERTOS_TCP)

        int InitMutex(wolfSSL_Mutex* m)
        {
            int iReturn;

            *m = ( wolfSSL_Mutex ) xSemaphoreCreateMutex();
            if( *m != NULL )
                iReturn = 0;
            else
                iReturn = BAD_MUTEX_E;

            return iReturn;
        }

        int FreeMutex(wolfSSL_Mutex* m)
        {
            vSemaphoreDelete( *m );
            return 0;
        }

        int LockMutex(wolfSSL_Mutex* m)
        {
            /* Assume an infinite block, or should there be zero block? */
            xSemaphoreTake( *m, portMAX_DELAY );
            return 0;
        }

        int UnLockMutex(wolfSSL_Mutex* m)
        {
            xSemaphoreGive( *m );
            return 0;
        }

    #elif defined(WOLFSSL_SAFERTOS)

        int InitMutex(wolfSSL_Mutex* m)
        {
            vSemaphoreCreateBinary(m->mutexBuffer, m->mutex);
            if (m->mutex == NULL)
                return BAD_MUTEX_E;

            return 0;
        }

        int FreeMutex(wolfSSL_Mutex* m)
        {
            (void)m;
            return 0;
        }

        int LockMutex(wolfSSL_Mutex* m)
        {
            /* Assume an infinite block */
            xSemaphoreTake(m->mutex, portMAX_DELAY);
            return 0;
        }

        int UnLockMutex(wolfSSL_Mutex* m)
        {
            xSemaphoreGive(m->mutex);
            return 0;
        }


    #elif defined(WOLFSSL_PTHREADS)

        int InitMutex(wolfSSL_Mutex* m)
        {
            if (pthread_mutex_init(m, 0) == 0)
                return 0;
            else
                return BAD_MUTEX_E;
        }


        int FreeMutex(wolfSSL_Mutex* m)
        {
            if (pthread_mutex_destroy(m) == 0)
                return 0;
            else
                return BAD_MUTEX_E;
        }


        int LockMutex(wolfSSL_Mutex* m)
        {
            if (pthread_mutex_lock(m) == 0)
                return 0;
            else
                return BAD_MUTEX_E;
        }


        int UnLockMutex(wolfSSL_Mutex* m)
        {
            if (pthread_mutex_unlock(m) == 0)
                return 0;
            else
                return BAD_MUTEX_E;
        }

    #elif defined(THREADX)

        int InitMutex(wolfSSL_Mutex* m)
        {
            if (tx_mutex_create(m, "wolfSSL Mutex", TX_NO_INHERIT) == 0)
                return 0;
            else
                return BAD_MUTEX_E;
        }


        int FreeMutex(wolfSSL_Mutex* m)
        {
            if (tx_mutex_delete(m) == 0)
                return 0;
            else
                return BAD_MUTEX_E;
        }


        int LockMutex(wolfSSL_Mutex* m)
        {
            if (tx_mutex_get(m, TX_WAIT_FOREVER) == 0)
                return 0;
            else
                return BAD_MUTEX_E;
        }


        int UnLockMutex(wolfSSL_Mutex* m)
        {
            if (tx_mutex_put(m) == 0)
                return 0;
            else
                return BAD_MUTEX_E;
        }

    #elif defined(MICRIUM)

        int InitMutex(wolfSSL_Mutex* m)
        {
            #if (NET_SECURE_MGR_CFG_EN == DEF_ENABLED)
                if (NetSecure_OS_MutexCreate(m) == 0)
                    return 0;
                else
                    return BAD_MUTEX_E;
            #else
                return 0;
            #endif
        }


        int FreeMutex(wolfSSL_Mutex* m)
        {
            #if (NET_SECURE_MGR_CFG_EN == DEF_ENABLED)
                if (NetSecure_OS_FreeMutex(m) == 0)
                    return 0;
                else
                    return BAD_MUTEX_E;
            #else
                return 0;
            #endif
        }


        int LockMutex(wolfSSL_Mutex* m)
        {
            #if (NET_SECURE_MGR_CFG_EN == DEF_ENABLED)
                if (NetSecure_OS_LockMutex(m) == 0)
                    return 0;
                else
                    return BAD_MUTEX_E;
            #else
                return 0;
            #endif
        }


        int UnLockMutex(wolfSSL_Mutex* m)
        {
            #if (NET_SECURE_MGR_CFG_EN == DEF_ENABLED)
                if (NetSecure_OS_UnLockMutex(m) == 0)
                    return 0;
                else
                    return BAD_MUTEX_E;
            #else
                return 0;
            #endif

        }

    #elif defined(EBSNET)

        int InitMutex(wolfSSL_Mutex* m)
        {
            if (rtp_sig_mutex_alloc(m, "wolfSSL Mutex") == -1)
                return BAD_MUTEX_E;
            else
                return 0;
        }

        int FreeMutex(wolfSSL_Mutex* m)
        {
            rtp_sig_mutex_free(*m);
            return 0;
        }

        int LockMutex(wolfSSL_Mutex* m)
        {
            if (rtp_sig_mutex_claim_timed(*m, RTIP_INF) == 0)
                return 0;
            else
                return BAD_MUTEX_E;
        }

        int UnLockMutex(wolfSSL_Mutex* m)
        {
            rtp_sig_mutex_release(*m);
            return 0;
        }

    #elif defined(WOLFSSL_uITRON4)
				#include "stddef.h"
        #include "kernel.h"
        int InitMutex(wolfSSL_Mutex* m)
        {
            int iReturn;
            m->sem.sematr  = TA_TFIFO ;
            m->sem.isemcnt = 1 ;
            m->sem.maxsem  = 1 ;
            m->sem.name    = NULL ;

            m->id = acre_sem(&m->sem);
            if( m->id != E_OK )
                iReturn = 0;
            else
                iReturn = BAD_MUTEX_E;

            return iReturn;
        }

        int FreeMutex(wolfSSL_Mutex* m)
        {
            del_sem( m->id );
            return 0;
        }

        int LockMutex(wolfSSL_Mutex* m)
        {
            wai_sem(m->id);
            return 0;
        }

        int UnLockMutex(wolfSSL_Mutex* m)
        {
            sig_sem(m->id);
            return 0;
        }

        /****  uITRON malloc/free ***/
        static ID ID_wolfssl_MPOOL = 0 ;
        static T_CMPL wolfssl_MPOOL = {TA_TFIFO, 0, NULL, "wolfSSL_MPOOL"};

        int uITRON4_minit(size_t poolsz) {
            ER ercd;
            wolfssl_MPOOL.mplsz = poolsz ;
            ercd = acre_mpl(&wolfssl_MPOOL);
            if (ercd > 0) {
                ID_wolfssl_MPOOL = ercd;
                return 0;
            } else {
                return -1;
            }
        }

        void *uITRON4_malloc(size_t sz) {
            ER ercd;
            void *p ;
            ercd = get_mpl(ID_wolfssl_MPOOL, sz, (VP)&p);
            if (ercd == E_OK) {
                return p;
            } else {
                return 0 ;
            }
        }

        void *uITRON4_realloc(void *p, size_t sz) {
          ER ercd;
          void *newp ;
          if(p) {
              ercd = get_mpl(ID_wolfssl_MPOOL, sz, (VP)&newp);
              if (ercd == E_OK) {
                  XMEMCPY(newp, p, sz) ;
                  ercd = rel_mpl(ID_wolfssl_MPOOL, (VP)p);
                  if (ercd == E_OK) {
                      return newp;
                  }
              }
          }
          return 0 ;
        }

        void uITRON4_free(void *p) {
            ER ercd;
            ercd = rel_mpl(ID_wolfssl_MPOOL, (VP)p);
            if (ercd == E_OK) {
                return ;
            } else {
                return ;
            }
        }

#elif defined(WOLFSSL_uTKERNEL2)
        #include "tk/tkernel.h"
        int InitMutex(wolfSSL_Mutex* m)
        {
            int iReturn;
            m->sem.sematr  = TA_TFIFO ;
            m->sem.isemcnt = 1 ;
            m->sem.maxsem  = 1 ;

            m->id = tk_cre_sem(&m->sem);
            if( m->id != NULL )
                iReturn = 0;
            else
                iReturn = BAD_MUTEX_E;

            return iReturn;
        }

        int FreeMutex(wolfSSL_Mutex* m)
        {
            tk_del_sem( m->id );
            return 0;
        }

        int LockMutex(wolfSSL_Mutex* m)
        {
            tk_wai_sem(m->id, 1, TMO_FEVR);
            return 0;
        }

        int UnLockMutex(wolfSSL_Mutex* m)
        {
            tk_sig_sem(m->id, 1);
            return 0;
        }

        /****  uT-Kernel malloc/free ***/
        static ID ID_wolfssl_MPOOL = 0 ;
        static T_CMPL wolfssl_MPOOL =
                     {(void *)NULL,
        TA_TFIFO , 0,   "wolfSSL_MPOOL"};

        int uTKernel_init_mpool(unsigned int sz) {
            ER ercd;
            wolfssl_MPOOL.mplsz = sz ;
            ercd = tk_cre_mpl(&wolfssl_MPOOL);
            if (ercd > 0) {
                ID_wolfssl_MPOOL = ercd;
                return 0;
            } else {
                return -1;
            }
        }

        void *uTKernel_malloc(unsigned int sz) {
            ER ercd;
            void *p ;
            ercd = tk_get_mpl(ID_wolfssl_MPOOL, sz, (VP)&p, TMO_FEVR);
            if (ercd == E_OK) {
                return p;
            } else {
                return 0 ;
            }
        }

        void *uTKernel_realloc(void *p, unsigned int sz) {
          ER ercd;
          void *newp ;
          if(p) {
              ercd = tk_get_mpl(ID_wolfssl_MPOOL, sz, (VP)&newp, TMO_FEVR);
              if (ercd == E_OK) {
                  XMEMCPY(newp, p, sz) ;
                  ercd = tk_rel_mpl(ID_wolfssl_MPOOL, (VP)p);
                  if (ercd == E_OK) {
                      return newp;
                  }
              }
          }
          return 0 ;
        }

        void uTKernel_free(void *p) {
            ER ercd;
            ercd = tk_rel_mpl(ID_wolfssl_MPOOL, (VP)p);
            if (ercd == E_OK) {
                return ;
            } else {
                return ;
            }
        }

    #endif /* USE_WINDOWS_API */

#endif /* SINGLE_THREADED */
        
