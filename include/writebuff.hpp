/***********************************************************************
 * writebuff.hpp - Writes a buffer to a file
 *
 * M.Overdick, J.Canfield
 * Last Major Revision: 7/14/16
 **********************************************************************/


#ifndef WRITEBUFF_HPP    // Prevents including file twice
#define WRITEBUFF_HPP

    // Writes a buffer filled with 32 bit unsigned integers to file
void writebuff_INT32U(
    const char* fname,      // A string for the filename
    INT32U *pbuff,          // A pointer to the first element
    int size                // Length of the buffer
);

    // Writes a buffer filled with complex 16 bit integers to file
void writebuff_CINT16(
    const char* fname,      // A string for the filename
    CINT16 *pbuff,          // A pointer to the first element
    int size                // Length of the buffer
);

    // Writes a buffer filled with 32 bit floating point numbers to file
void writebuff_FP32(
    const char* fname,      // A string for the filename
    FP32 *pbuff,            // A pointer to the first element
    int size                // Length of the buffer
);

#endif /* #ifndef WRITEBUFF_HPP */
