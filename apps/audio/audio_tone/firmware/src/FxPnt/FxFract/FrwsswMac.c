//*****************************************************************************
//
// File: FrwssdMac.c
//
// Description: Performs a Multiply-Accumulate function WITH saturation.
//
//*****************************************************************************

#include "FxConvert.h"
#include "FxFract.h"


/******************************************************************************
 *
 * FrwssdMac()
 *
 * Description:
 *   Performs a Multiply-Accumulate function WITH saturation.  This routine
 *   returns the fractional 32-bit result from the accumulator output
 *   'SAT(addOut_Q1d31)=outQ1d15' where 'multOut_Q1d31 + a_Q1d31 =
 *   addOut_Q1d31', and 'b_Q1d15 x c_Q1d15 = multOut_Q1d31'. The multiply is
 *   performed on the two 16-bit fractional input values 'b x c' which results
 *   in a 32-bit value.  This result is left shifted by one to account for the
 *   extra sign bit inherent in the fractional-type multiply.  The shifted
 *   number represents a Q1d31 number with the lsb set to '0'.  This Q1d31
 *   number is added with the 32-bit fractional input argument 'a'.  Saturation
 *   is applied on the output of the accumulator to keep the value within the
 *   32-bit fractional range and then this value is returned.  This instruction
 *   is equalivalent to performing the following functions:
 *         FrwsswMac(a,b,c) ====> FrwwwAdd(a, FrsswMult(b,c)));
 *   The FrwssdMac() function is for fractional Qtype format data only and it 
 *   therefore will not give the correct results for true integers. 
 *
 *   This function relates to the ETSI L_mac function.
 *
 * Arguments:
 *   Fract32 a
 *     [in]  32-bit accumulator operand 1
 *   Fract16 b
 *     [in]  16-bit multiplication operand 1
 *   Fract16 c
 *     [in]  16-bit multiplication operand 2
 *
 * Return Value:
 *   Fract32 result
 *     [return] 32-bit  
 *
 ******************************************************************************/
Fract32 FrwsswMac(Fract32 a, Fract16 b, Fract16 c) 
{  
  Fract32 result;                    /* Value returned */
  Fract32 multOut;   /* Temp var for holding result from Multiply */
 
  /* Multiply the 2 fractional inputs, 'b_Q1d15 x c_Q1d15 => multOut_Q1d31'.
   * The 1 bit shift to the left to get rid of the extra sign bit is
   * automatically taken care of in the 'frssdMult' function.  */
  multOut = FrsswMult(b,c);
  /* Accumulate the multiplier output multOut_Q1d31 with the 2's complement
   * 32-bit fractional Input, a_Q1d31'.  It is assumed that the binary point
   * of the 32-bit 'a' input is in the same position as the multOut_Q1d31
   * value.  */
  result = FrwwwAdd(a, multOut);
  return (result);
}
