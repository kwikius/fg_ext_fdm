#ifndef FG_EXT_SENSORS_HPP_INCLUDED
#define FG_EXT_SENSORS_HPP_INCLUDED

#include <quan/time.hpp>
#include <quan/frequency.hpp>
#include <fgfs_telnet.hpp>
#include <fgfs_fdm_in.hpp>
/**
 * @brief get FlightGear current framerate via telnet
**/
 quan::frequency::Hz get_frame_rate(fgfs_telnet & t);
/**
* @brief get locally stored framerate ( dont telnet to Flightgear)
**/
quan::frequency::Hz get_frame_rate();

/**
   3D accelerometer - vector body_frame
   3D gyro          - vector body_frame 
   
   3D compass       - vector world_frame  
   altitude         - world frame asl
   GPS_position     - world frame
   ground height    - world frame asl
   Airspeed         - body_frame
   Thrust           - body_frame

   Battery
   Voltage
   Current
  
   
**/


#endif // FG_EXT_SENSORS_HPP_INCLUDED
