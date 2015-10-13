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

#include <cmath>
#include <complex>
#include <vector>

#ifndef SINC_HPP
#define SINC_HPP

    // Counters
static INT16U   i;
static FP32     j;


class sinc_table_class{
public:
    sinc_table_class(INT16U ampl, FP32 bw, FP32 cbw, INT16U pulse_length, INT16U spb, FP32 tau):
        _sinc_table(spb)
    {
        FP64   w0 = cbw*std::acos(-1.0);                // w0 controls frequency shift 
        FP64   eta = bw*std::acos(-1.0);                // eta controls BW of sinc pulse
        INT16U totwidth = 2*floor(pulse_length/bw);     // technically 2*pulse_length/bw+1, but first and last points
                                                        //    of pulse will be a zero, and we don't count final zero.
        INT16U Lpadding = floor((spb-totwidth)/2);      // amount of left padding in buffer (before pulse)

        for (i = 0; i < spb; i++){
            if(i >= Lpadding+totwidth || i < Lpadding){         // zero padding
                _sinc_table[i] = CINT16(0.0, 0.0);
            } else {                                            // centered pulse
                j = i - Lpadding - floor(pulse_length/bw) + tau;        
                if (j != 0) {
                    _sinc_table[i] = CINT16(ampl * std::sin(eta*j) / (eta*j) * std::cos(w0*j), 
                                            ampl * std::sin(eta*j) / (eta*j) * std::sin(w0*j));
                } else {  // avoid division by zero 
                    _sinc_table[i] = CINT16(ampl, 0.0);
                }   
            }
        }
    }

    inline CINT16 operator()(const INT16U index) const{
        return _sinc_table[index];
    }
private:
    std::vector< CINT16 > _sinc_table;
};
#else
#endif

