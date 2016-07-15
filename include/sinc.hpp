/*******************************************************************************
 * sinc.hpp - Modulated sinc pulse generator header.
 *
 * M.Overdick, J.Canfield, and A.G.Klein
 * Last Major Revision: 7/14/2016
 ******************************************************************************/

#ifndef SINC_HPP
#define SINC_HPP

    // Initializes oversampled sinc pulse
void Sinc_Init(FP32 bw, FP32 cbw, INT32U spb, FP32 precision, FP32 fs);

    // Undersamples and creates delayed sinc pulse
void Sinc_Gen(CINT16 * table, INT16U ampl, INT16U spb, FP32 tau);

#else
#endif
