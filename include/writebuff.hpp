/***********************************************************************
 * writebuff.hpp - Writes a buffer to a file
 *
 * M.Overdick, J.Canfield
 * Last Major Revision: 7/26/16
 **********************************************************************/


#ifndef WRITEBUFF_HPP    // Prevents including file twice
#define WRITEBUFF_HPP

    // Template of writebuff
template <typename data_type> void writebuff(
    const char* fname,      // A string for the filename
    data_type *pbuff,       // A pointer to the first element
    int size                // Length of the buffer
);

#endif /* #ifndef WRITEBUFF_HPP */
