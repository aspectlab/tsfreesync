/*******************************************************************************
 * sinc.cpp - Modulated sinc pulse generator code.
 *
 * M.Overdick, J.Canfield, and A.G. Klein
 * Last Major Revision: 7/20/2016
 ******************************************************************************/
#include "includes.hpp"
#include <iostream>
#include <fstream>

#define SCALAR          0x7FFF       // Scalar for precomputed sinc pulse amplitude

    // Sinc macro
#define SINC(x)         ( std::sin(x) / (x) )

    // Initializes oversampled sinc pulse
extern void Sinc_Init(FP32 bw, FP32 cbw, INT32U spb, INT16U ratio);

    // Undersamples and creates delayed sinc pulse
extern void Sinc_Gen(CINT16 * table, INT16U ampl, INT16U spb, FP32 tau);

    // Checks for existance of "OS_Sinc.dat" and copy contents
static bool Sinc_Read();

static std::vector< CINT16 > Sinc_Table;    // Vector that holds oversampled pulse
static INT16U Sinc_Ratio;                    // Ratio between real sampling frequency
                                            // and oversampling frequency

/*******************************************************************************
 * Sinc_Init() - Innitialized sinc pulse generation, must be called before
 * caling Sinc_Gen(). This function creates an oversampled sinc pulse used to
 * generate undersampled sinc pulses with a programmable precision.
 *
 * ARGUMENTS:
 *      FP32 bw         -   Bandwidth of the sinc pulse, relative to fs
 *      FP32 cbw        -   Carrier bandwidth, relative to fs
 *      INT32U spb      -   Samples per buffer
 *      INT16U ratio    -   Oversampling ratio
 *
 *
 * RETURNS:(none)
 *
 *
 * M.Overdick, J.Canfield, and A.G. Klein
 * Last Major Revision: 7/21/2016
 ******************************************************************************/
void Sinc_Init(FP32 bw, FP32 cbw, INT32U spb, INT16U ratio){
     INT32U length  =   0;          // The length (n samples) of sinc pulse
     FP32   w0      =   0;          // Normalized bandwidth of carrier
     FP32   eta     =   0;          // Normalized bandwidth of sinc pulse
     INT64  j, i;                   // Counter
     bool error = true;             // Error return

     Sinc_Ratio = ratio;

     length = spb*Sinc_Ratio;            // Compute length
     Sinc_Table.resize(length);     // Resize Sinc_Table vector to "length"

        // Check if sinc pulse has already been computed before.
     if(!Sinc_Read()){
        std::cout << "Generating oversampled sinc pulse..." << std::endl;

        // Compute normalized bandwidths
        w0 = cbw*PI;
        eta = bw*PI;

        for(j = 0; j < length; j++){
            i = j/Sinc_Ratio - spb/2;    // i is the oversampled step size

                // When i is not zero, generate sinc pulse as usual
            if(i != 0){
                Sinc_Table[j] = CINT16(SCALAR * ( std::sin(eta*i) / (eta*i) ) * std::cos(w0*i), \
                                SCALAR * ( std::sin(eta*i) / (eta*i) ) * std::sin(w0*i));

                // Avoid divide by zero when i is zero
            }else{
                Sinc_Table[j] = CINT16(SCALAR, 0);
            }

                // Display percentage to prove the program hasn't crahsed
            std::cout << boost::format("\r\t%3.1f Percent Complete") % (FP32(100*j)/FP32(length)) << std::flush;
        }

        std::cout << "\r\tdone!                    " << std::endl << std::endl;

            // Write template sinc pulse to file
        std::cout << "Writing oversampled Sinc to file..." << std::flush;
        writebuff < CINT16 >("./OS_Sinc.dat", &Sinc_Table.front(), Sinc_Table.size());
        std::cout << "done!" << std::endl << std::endl;
    }else{
            // If OS_Sinc.dat is present, tell the user...
        std::cout << "Using provided oversampled sinc pulse from file." << std::endl;
        std::cout << "\tDelete \"OS_Sinc.dat\" to regenerate file." << std::endl << std::endl;
    }
 }

/*******************************************************************************
 * Sinc_Gen() - Loads array with delayed sinc pulse.
 *
 * ARGUMENTS:
 *     CINT16* table   -   A pointer to the first element of an array of length
 *                         spb.
 *     INT16   ampl    -   The peak amplitude of the delayed sinc pulse, choose
 *                         a number between 0 - 32767.
 *     INT16U  spb     -   Samples per buffer.
 *     FP32    delay   -   The delay of the pulse, between +/- spb/2, 0 results
 *                         in a centered pulse
 *
 * RETURNS:
 *     A delayed sinc pulse saved in the table passed in the first argument
 *
 * M.Overdick, J.Canfield, and A.G. Klein
 * Last Major Revision: 7/14/2016
 ******************************************************************************/
void Sinc_Gen(CINT16 * table, INT16 ampl, INT16U spb, FP32 delay){
        // Counters
    INT32   i       = 0;                    // Counter
    INT32U  j       = 0;                    // Counter
    INT32U  shift   = 0;                    // Starting point of undersampling
    INT32U  remain  = 0;                    // Leftover space in over sample vector
    FP32    scale   = FP32(ampl)/SCALAR;    // Scalar for pulse amplitude
    // std::cout << "Ratio " << Sinc_Ratio << std::endl;
    // std::cout << "Delay " << delay << std::endl;
    // delay = std::fmod(delay+spb,spb);       // Keep delay within boundaries and positive
        // Make shift always positive
    while(delay >= spb/2){
        delay = std::fmod(delay-spb,spb);
    }
    while(delay < 0){
        delay = std::fmod(delay+spb,spb);
    }

    // std::cout << "MDELAY " << delay << std::endl;
    // std::cout << "MULT "   << delay*Sinc_Ratio << std::endl;
    shift = delay*Sinc_Ratio;        // Compute starting point of undersampling
    // std::cout << "SHIFT " << shift << std::endl;

        // Generate first part of delayed pulse
    for(i = shift; i < spb*Sinc_Ratio; i = i + Sinc_Ratio){
        table[j] = CINT16(Sinc_Table[i].real()*scale, Sinc_Table[i].imag()*scale);
        j++;
    }

    remain = spb*Sinc_Ratio - i - Sinc_Ratio; // Remember last position calculated

        // Generate second part of delayed pulse
    for(i = Sinc_Ratio - remain; i < INT32(shift-Sinc_Ratio); i = i + Sinc_Ratio){
        table[j] = CINT16(Sinc_Table[i].real()*scale, Sinc_Table[i].imag()*scale);
        j++;
    }
}

/*******************************************************************************
 * Sinc_Read() - Checks for precomputed sinc pulse file, if it exists, it Loads
 *               the content of that file into Sinc_Table and returns true. If
 *               the file does not exist, it returns false.
 *
 * M.Overdick, J.Canfield, and A.G. Klein
 * Last Major Revision: 7/18/2016
 ******************************************************************************/
static bool Sinc_Read(){
    bool success = false;   // Return variable

        // Open file to use for oversampled sinc pulse
    std::ifstream infile("OS_Sinc.dat", std::ifstream::binary);

        // Check if file exists, if so, copy it into array
    if(infile.is_open()){
        while(!infile.eof()){
            infile.read((char*)&Sinc_Table.front(), Sinc_Table.size()*sizeof(CINT16));
        }
        infile.close();
        success = true;
    }else{}

    return success;
}
