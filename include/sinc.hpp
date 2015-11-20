/***********************************************************************
 * sinc.hpp - Modulated sinc pulse generator.
 * 
 * This source file is based on Ettus Research's wavetable.hpp.
 * This header file generates a modulated sinc pulse that is zero-padded
 * and stored into a vector. 
 * 
 * 
 * VERSION 08.20.15:1 -- initial version by Mitchell Overdick 
 * VERSION 09.07.15:2 -- modified by A.G.Klein to include parameterizable
 *                       relative bandwidth, etc, and to specify
 *                       pulse width in a way that avoids bad windowing
 * VERSION 10.13.15:1 -- added code to calculate shifted sinc pulse by τ
 *                       switched to types from types.hpp, now computes
 *                       using integer math.
 * 
 **********************************************************************/

#include "includes.hpp"
#include <math.h>

#ifndef SINC_HPP
#define SINC_HPP

    // Function Prototype
void Sinc_Gen(CINT16 * table, INT16U ampl, FP32 bw, FP32 cbw, INT16U pulse_length, INT16U spb, FP32 tau);

    // Function Code
void Sinc_Gen(CINT16 * table, INT16U ampl, FP32 bw, FP32 cbw, INT16U pulse_length, INT16U spb, FP32 tau){
            // Counters
    INT16U  i   = 0;
    FP32    j   = 0;
    FP64    w0  = cbw*PI;   // w0 controls frequency shift 
    FP64    eta = bw*PI;    // eta controls BW of sinc pulse
    
    FP64  ipart;          // Integer part of tau
    FP64  fpart;          // Fractional part of tau
    
        // Make shift always positive
    if(tau < 0){
        tau = spb + tau;
    }else{}
    
    fpart = modf(tau, &ipart);     // Split tau into fractional and integer parts
    
        // Compute first part of pulse
    for (i = ipart; i < spb; i++) {
        j = (i - ipart) - (spb/2);        
        if (j+fpart != 0) {
            table[i] = CINT16(ampl * std::sin(eta*j+fpart) / (eta*j+fpart) * std::cos(w0*j+fpart), 
                              ampl * std::sin(eta*j+fpart) / (eta*j+fpart) * std::sin(w0*j+fpart));
        } else {  // avoid division by zero 
            table[i] = CINT16(ampl, 0.0);
        }   
    }
    
        // Wrap pulse arround buffer
    for (i = 0; i < ipart; i++){
        j = (i - ipart) + spb - (spb/2);        
        if (j+fpart != 0) {
            table[i] = CINT16(ampl * std::sin(eta*j+fpart) / (eta*j+fpart) * std::cos(w0*j+fpart), 
                              ampl * std::sin(eta*j+fpart) / (eta*j+fpart) * std::sin(w0*j+fpart));
        } else {  // avoid division by zero 
            table[i] = CINT16(ampl, 0.0);
        }   
    }
    

}

#else
#endif

