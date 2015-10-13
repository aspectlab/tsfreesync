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
 * 
 **********************************************************************/

#include <cmath>
#include <complex>
#include <vector>

#ifndef SINC_HPP
#define SINC_HPP
static signed int j;                // Used for generating sinc pulse

class sinc_table_class{
public:
    sinc_table_class(const float ampl, const float bw, const float cbw, const int pulse_length, const int spb):
// why static stuff? keep OO implementation?
        _sinc_table(spb)
    {
        static const double tau = cbw*std::acos(-1.0);          // tau controls frequency shift 
        static const double lambda = bw*std::acos(-1.0);        // lambda controls BW of sinc pulse
        static const int totwidth=2*floor(pulse_length/bw);     // technically 2*pulse_length/bw+1, but first and last points
                                                                //    of pulse will be a zero, and we don't count final zero.
        static const int Lpadding = floor((spb-totwidth)/2);    // amount of left padding in buffer (before pulse)

        for (size_t i = 0; i < spb; i++){
            if(i >= Lpadding+totwidth || i < Lpadding){         // zero padding
                _sinc_table[i] = std::complex<float>(0.0, 0.0);
            } else {                                            // centered pulse
                j = i - Lpadding - floor(pulse_length/bw);        
                if (j != 0) {
                    _sinc_table[i] = std::complex<float>(ampl * std::sin(lambda*j) / (lambda*j) * std::cos(tau*j), 
                                                         ampl * std::sin(lambda*j) / (lambda*j) * std::sin(tau*j));
                } else {  // avoid division by zero 
                    _sinc_table[i] = std::complex<float>(ampl, 0.0);
                }   
            }
        }
    }

    inline std::complex<float> operator()(const size_t index) const{
        return _sinc_table[index];
    }
private:
    std::vector<std::complex<float> > _sinc_table;
};
#else
#endif

