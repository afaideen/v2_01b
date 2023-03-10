//*****************************************************************************
//
// File: libq_q31_Negate_q31.c
//
// Description: Negate 32-bit 2s-complement fractional value with saturation.
//
//*****************************************************************************/

#include "libq_C.h"


/******************************************************************************
 *
 * libq_q31_Negate_q31()
 *
 * Description:
 *   Negate 32-bit 2s-complement fractional value with saturation. The
 *   saturation is for handling the case where negating a MINFRACT32 is
 *   greater than MAXFRACT32, or the allowable range of values.
 *   This function relates to the ETSI L_negate function.
 *
 * Arguments:
 *   q31 a
 *     [in] input argument
 *
 * Return Value:
 *   q31 result
 *     [return]  Range: MINFRACT32 <= result <= MAXFRACT32
 *
 ******************************************************************************/
q31 libq_q31_Negate_q31(q31 a)
{
  q31 result;                   /* Value returned */

  /* If anything but the most negative 2's complement input value*/
  if (a != (q31)MINFRACT32)
  {
    /* Simply take the negative 2's complement value */
    result = (q31)(-a);
  }
  else
  {
    /* else for the max negative value, saturate to the most positive value */
    result = (q31)MAXFRACT32;
  }
  return (result);
}
