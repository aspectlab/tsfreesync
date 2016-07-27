/*******************************************************************************
 * includes.hpp - Master includes file
 *
 * M.Overdick
 * Last Major Revision: 5/12/2016
 ******************************************************************************/

#ifndef INCLUDES_HPP    // Prevents including file twice
#define INCLUDES_HPP

/*******************************************************************************
 * Generic Defined Constants
 ******************************************************************************/
#define PI      3.14159265358979323846
#define TWOPI   6.28318530717958647692

/*******************************************************************************
 * Generic Project Includes
 ******************************************************************************/
#include "types.hpp"                        // Master type definition file
#include <iostream>                         // Input/Output to terminal
#include <csignal>                          // Signal handlers
#include <cmath>                            // Mathematical funcitons
#include <math.h>                           // Math
#include <complex>                          // Complex numbers
#include <vector>                           // Standard vector type

/*******************************************************************************
 * UHD Includes
 ******************************************************************************/
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>

/*******************************************************************************
 * Boost Includes
 ******************************************************************************/
#include <boost/format.hpp>                 // String formatting lib
#include <boost/thread.hpp>                 // Thread handling lib
#include <boost/assign/list_of.hpp>

/*******************************************************************************
 * Project Specific Includes
 ******************************************************************************/
#include "sinc.hpp"                         // Sinc pulse generator
#include "writebuff.hpp"                    // Writes buffer to file
#include "checklo.hpp"                      // Checks LO Lock status

#endif /* ifndef INCLUDES_HPP */
