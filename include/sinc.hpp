/*******************************************************************************
 * sinc.hpp - Modulated sinc pulse generator.
 *
 * This source file is based on Ettus Research's wavetable.hpp.
 * This header file generates a modulated sinc pulse that is zero-padded
 * and stored into a vector.
 *
 * M.Overdick, A.G.Klein, and J.Canfield
 * Last Major Revision: 5/12/2016
 ******************************************************************************/

#include "includes.hpp"
#include <math.h>

#ifndef SINC_HPP
#define SINC_HPP
    // Sinc function macro
#define SINC(x) ( std::sin(x) / (x) )

    // Function Prototype
void Sinc_Gen(CINT16 * table, INT16U ampl, FP32 bw, FP32 cbw, INT16U spb, FP32 tau);

    // Function Code
void Sinc_Gen(CINT16 * table, INT16U ampl, FP32 bw, FP32 cbw, INT16U spb, FP32 tau){
            // Counters
    INT16  i   = 0;
    FP32   j   = 0;
    FP32   w0  = cbw*PI;    // w0 controls frequency shift
    FP32   eta = bw*PI;     // eta controls BW of sinc pulse

    FP32  ipart;            // Integer part of tau
    FP32  fpart;            // Fractional part of tau

        // Make shift always positive
    if(tau < 0){
        tau = spb + tau;
    }else{}

        // Split tau into fractional and integer parts
    ipart = std::floor(tau);
    fpart = (tau - ipart)*-1;

        // Compute first part of pulse
    for (i = ipart; i < spb; i++) {
        j = (i - ipart) - (spb/2) + fpart;
        if (j != 0) {
            // table[i] = CINT16(ampl * SINC(eta*j) * std::cos(w0*j),
            //                   ampl * SINC(eta*j) * std::sin(w0*j));
            table[i] = CINT16(ampl * SINC(eta*j) * std::cos(w0*j),
                              0.0);
        } else {  // avoid division by zero
            table[i] = CINT16(ampl, 0.0);
        }
    }

        // Wrap pulse arround buffer
    for (i = 0; i < ipart; i++){
        j = (i - ipart) + spb - (spb/2) + fpart;
        if (j != 0) {
            // table[i] = CINT16(ampl * SINC(eta*j) * std::cos(w0*j),
            //                   ampl * SINC(eta*j) * std::sin(w0*j));
            table[i] = CINT16(ampl * SINC(eta*j) * std::cos(w0*j),
                              0.0);
        } else {  // avoid division by zero
            table[i] = CINT16(ampl, 0.0);
        }
    }
}

#else
#endif
