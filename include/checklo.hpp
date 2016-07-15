/*******************************************************************************
 * checklo.hpp - Check lo lock status on USRP
 *
 * This file holds the function to check locked sensor written at Ettus
 * Research. This header file was created to remove clutter from source code.
 *
 ******************************************************************************/

#ifndef CHECKLO_HPP
#define CHECKLO_HPP

    // Global data type
typedef boost::function<uhd::sensor_value_t (const std::string&)> get_sensor_fn_t;

    // Function prototype
bool check_locked_sensor(std::vector<std::string> sensor_names, const char* sensor_name, get_sensor_fn_t get_sensor_fn, double setup_time);

#else
#endif
