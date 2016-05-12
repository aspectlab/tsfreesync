/*******************************************************************************
 * types.hpp - Holds a table of intuitive type definitions
 *
 * M.Overdick
 * Last Major Revision: 9/14/2015
 ******************************************************************************/

#include "includes.hpp"
#include <complex>

#ifndef TYPES_HPP
#define TYPES_HPP

/*******************************************************************************
 * Fundamental types
 ******************************************************************************/

    /** INTEGERS **************************************************************/
    // Unsigned
typedef unsigned char               INT8U;      // Unsigned 8 bit integer
typedef unsigned int                INT16U;     // Unsigned 16 bit integer
typedef unsigned long int           INT32U;     // Unsigned 32 bit integer
typedef unsigned long long int      INT64U;     // Unsigned 64 bit integer

    // Signed
typedef signed char                 INT8;       // Signed 8 bit integer
typedef signed short int            INT16;      // Signed 16 bit integer
typedef signed long int             INT32;      // Signed 32 bit integer
typedef signed long long int        INT64;      // Signed 64 bit integer

    /** FLOATING POINT ********************************************************/
typedef float                       FP32;       // Signed 32 bit float
typedef double                      FP64;       // Signed 64 bit float

/*******************************************************************************
 * Special types
 ******************************************************************************/

    /** COMPLEX INTEGER *******************************************************/
typedef std::complex< INT8 >        CINT8;      // Complex 8 bit integer
typedef std::complex< INT16 >       CINT16;     // Complex 8 bit integer
typedef std::complex< INT32 >       CINT32;     // Complex 8 bit integer
typedef std::complex< INT64 >       CINT64;     // Complex 8 bit integer

    /** COMPLEX FLOAT *********************************************************/
typedef std::complex < FP32 >       CFP32;      // Complex 32 bit float
typedef std::complex < FP64 >       CFP64;      // Complex 32 bit float

#endif
