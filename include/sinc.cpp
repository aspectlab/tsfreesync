/*******************************************************************************
 * sinc.cpp - Modulated sinc pulse generator code.
 *
 * M.Overdick, J.Canfield, and A.G. Klein
 * Last Major Revision: 7/14/2016
 ******************************************************************************/
#include "includes.hpp"
#include <math.h>

    // Sinc macro
#define SINC(x) std::sin(x) / x

    // Initializes oversampled sinc pulse
extern void Sinc_Init(FP32 bw, FP32 cbw, INT32U spb, FP32 precision, FP32 fs);

    // Undersamples and creates delayed sinc pulse
extern void Sinc_Gen(CINT16 * table, INT16U ampl, INT16U spb, FP32 tau);

static std::vector< CINT16 > Sinc_Table;
static FP32 Ratio;

/*******************************************************************************
 * Sinc_Init() - Innitialized sinc pulse generation, must be called before
 * caling Sinc_Gen(). This function creates an oversampled sinc pulse used to
 * generate undersampled sinc pulses with a programmable precision.
 *
 * ARGUMENTS:
 *      FP32 bw         -   Bandwidth of the sinc pulse, relative to fs
 *      FP32 cbw        -   Carrier bandwidth, relative to fs
 *      INT32U spb      -   Samples per buffer
 *      FP32 precision  -   The smallest shift possible (in seconds)
 *      FP32 fs         -   The sample rate of the real-world transmission
 *
 * RETURNS: (none)
 *
 * M.Overdick, J.Canfield, and A.G. Klein
 * Last Major Revision: 7/14/2016
 ******************************************************************************/
void Sinc_Init(FP32 bw, FP32 cbw, INT32U spb, FP32 precision, FP32 fs){
     INT32U length  =   spb/(fs*precision);
     FP32   w0      =   0;
     FP32   eta     =   0;
     INT64  i,j;

     Ratio = 1/(fs*precision);
     w0 = cbw*PI*Ratio;
     eta = bw*PI*Ratio;

     Sinc_Table.resize(length);

     for(i = -length/2; i < length/2; i++){
        j = i+length/2;
        Sinc_Table[j] = CINT16(0x7FFF * SINC(eta*i), 0) *
                        CINT16(std::cos(w0*i), std::sin(w0*i));
     }
 }

 /*******************************************************************************
  * Sinc_Gen() -
  *
  * 
  * M.Overdick, J.Canfield, and A.G. Klein
  * Last Major Revision: 7/14/2016
  ******************************************************************************/
void Sinc_Gen(CINT16 * table, INT16U ampl, INT16U spb, FP32 tau){
        // Counters
    INT32   i       = 0;
    INT32U  j       = 0;
    INT32U  shift   = 0;
    INT32U  last    = 0;
    INT16U  scale   = ampl/0x7FFF;

        // Make shift always positive
    if(tau < 0){
        tau = spb + tau;
    }else{}

    shift = tau*Ratio;

    for(i = shift; i < spb*Ratio; i = i + Ratio){
        table[j] = Sinc_Table[i] * CINT16(scale,0);
        j++;
    }
    last = i - Ratio;

    for(i = spb*Ratio-last; i < shift-Ratio; i = i + Ratio){
        table[j] = Sinc_Table[i] * CINT16(scale,0);
        j++;
    }
}
