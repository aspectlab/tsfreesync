/*******************************************************************************
 * writebuff.cpp
 *
 * This source file contains a function to write arrays of various datatypes to
 * a file. All datatypes used must be instantiated at the bottom of this source
 * file.
 *
 * M.Overdick
 * Last Major Revision: 7/26/2016
 ******************************************************************************/

#include "includes.hpp"
#include <fstream>

/*******************************************************************************
 * writebuff() - Writes a file of given name from a buffer of given size
 * ARGUMENTS:
 *      const char* fname   -   A string for the filename to output
 *      data_type *pbuff    -   A pointer to the first element of a buffer
 *      int size            -   The size (length) of the buffer
 *
 * RETURNS: (none)
 *
 * M.Overdick
 * Last Major Revision: 7/26/2016
 ******************************************************************************/
template <typename data_type> void writebuff(const char* fname, data_type *pbuff, int size){
    std::ofstream outfile;                          // File object
    outfile.open(fname, std::ofstream::binary);     // Open the file

        // Write contents of buffer to file
    if (outfile.is_open()){
        outfile.write((const char*)pbuff, size*sizeof(data_type));
    }else{}

        // Close the file if it is still open
    if (outfile.is_open()){
        outfile.close();
    }else{}
}

    // Instantiate template types
template void writebuff<CINT16>(const char* fname, CINT16 *pbuff, int size);
template void writebuff<INT32U>(const char* fname, INT32U *pbuff, int size);
template void writebuff<FP32>(const char* fname, FP32 *pbuff, int size);
template void writebuff<FP64>(const char* fname, FP64 *pbuff, int size);
