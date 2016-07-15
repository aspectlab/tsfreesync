/*******************************************************************************
 * writebuff.cpp
 *
 * This source file contains several functions that are written to take an array
 * and write it to a file. The different functions are used for different
 * arrays of different data types.
 *
 * M.Overdick
 * Last Major Revision: 7/14/2016
 ******************************************************************************/

#include "includes.hpp"
#include <fstream>

/*******************************************************************************
 * writebuff_INT32U() - Writes a file of given name from a buffer of given size
 * ARGUMENTS:
 *      const char* fname   -   A string for the filename to output
 *      INT32U *pbuff       -   A pointer to the first element of a buffer
 *      int size            -   The size (length) of the buffer
 *
 * RETURNS: (none)
 *
 * M.Overdick
 * Last Major Revision: 7/14/2016
 ******************************************************************************/
void writebuff_INT32U(const char* fname, INT32U *pbuff, int size){
    std::ofstream outfile;
    outfile.open(fname, std::ofstream::binary);

    if (outfile.is_open()){
        outfile.write((const char*)pbuff, size*sizeof(INT32U));
    }else{}

    if (outfile.is_open()){
        outfile.close();
    }else{}
}

/*******************************************************************************
 * writebuff_CINT16() - Writes a file of given name from a buffer of given size
 * ARGUMENTS:
 *      const char* fname   -   A string for the filename to output
 *      CINT16 *pbuff       -   A pointer to the first element of a buffer
 *      int size            -   The size (length) of the buffer
 *
 * RETURNS: (none)
 *
 * M.Overdick
 * Last Major Revision: 7/14/2016
 ******************************************************************************/
void writebuff_CINT16(const char* fname, CINT16 *pbuff,int size){
    std::ofstream outfile;
    outfile.open(fname, std::ofstream::binary);

    if (outfile.is_open()){
        outfile.write((const char*)pbuff, size*sizeof(CINT16));
    }else{}

    if (outfile.is_open()){
        outfile.close();
    }else{}
}

/*******************************************************************************
 * writebuff_CINT16() - Writes a file of given name from a buffer of given size
 * ARGUMENTS:
 *      const char* fname   -   A string for the filename to output
 *      FP32 *pbuff       -   A pointer to the first element of a buffer
 *      int size            -   The size (length) of the buffer
 *
 * RETURNS: (none)
 *
 * M.Overdick
 * Last Major Revision: 7/14/2016
 ******************************************************************************/
void writebuff_FP32(const char* fname, FP32 *pbuff,int size){
    std::ofstream outfile;
    outfile.open(fname, std::ofstream::binary);

    if (outfile.is_open()){
        outfile.write((const char*)pbuff, size*sizeof(FP32));
    }else{}

    if (outfile.is_open()){
        outfile.close();
    }else{}
}
