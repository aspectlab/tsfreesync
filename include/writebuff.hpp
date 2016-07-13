/***********************************************************************
 * writebuff.hpp - Writes a buffer to a file
 *
 * ARGUMENTS:
 *
 * fname    -   String for filename, "./fname.dat" places file in
 *              working directory of terminal, not relative to program
 *              directory.
 *
 * *pbuff   -   A pointer to the first element of the buffer to be
 *              written.
 *
 * size     -   The size of the buffer (in number of entries).
 *
 * M.Overdick, J.Canfield
 * Last Major Revision: 7/13/16
 **********************************************************************/

#include "includes.hpp"
#include <fstream>

#ifndef WRITEBUFF_HPP    // Prevents including file twice
#define WRITEBUFF_HPP

void writebuff_INT32U(
    const char* fname,
    INT32U *pbuff,
    int size
){
    std::ofstream outfile;
    outfile.open(fname, std::ofstream::binary);

    if (outfile.is_open()){
        outfile.write((const char*)pbuff, size*sizeof(INT32U));
    }else{}

    if (outfile.is_open()){
        outfile.close();
    }else{}
}

void writebuff_CINT16(
    const char* fname,
    CINT16 *pbuff,
    int size
){
    std::ofstream outfile;
    outfile.open(fname, std::ofstream::binary);

    if (outfile.is_open()){
        outfile.write((const char*)pbuff, size*sizeof(CINT16));
    }else{}

    if (outfile.is_open()){
        outfile.close();
    }else{}
}

void writebuff_FP32(
    const char* fname,
    FP32 *pbuff,
    int size
){
    std::ofstream outfile;
    outfile.open(fname, std::ofstream::binary);

    if (outfile.is_open()){
        outfile.write((const char*)pbuff, size*sizeof(FP32));
    }else{}

    if (outfile.is_open()){
        outfile.close();
    }else{}
}

#endif /* #ifndef WRITEBUFF_HPP */
