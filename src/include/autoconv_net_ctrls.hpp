// net_ctrls.hxx -- defines a common net I/O interface to the flight
//                  sim controls
//
// Written by Curtis Olson - http://www.flightgear.org/~curt
// Started July 2001.
//
// This file is in the Public Domain, and comes with no warranty.
//
// $Id$


#ifndef AUTOCONV_NET_CTRLS_HXX
#define AUTOCONV_NET_CTRLS_HXX

#include <net_ctrls.hxx>
#include <quan/length.hpp>
#include <quan/velocity.hpp>
#include <quan/angle.hpp>
#include <quan/temperature.hpp>
#include <quan/pressure.hpp>
#include <quan/network_variable.hpp>


// NOTE: this file defines an external interface structure.  Due to
// variability between platforms and architectures, we only used fixed
// length types here.  Specifically, integer types can vary in length.
// I am not aware of any platforms that don't use 4 bytes for float
// and 8 bytes for double.

//    !!! IMPORTANT !!!
/* There is some space reserved in the protocol for future use.
 * When adding a new type, add it just before the "reserved" definition
 * and subtract the size of this new type from the RESERVED_SPACE definition
 * (1 for (u)int32_t or float and 2 for double).
 *	
 * This way the protocol will be forward and backward compatible until
 * RESERVED_SPACE becomes zero.
 */

//#define RESERVED_SPACE 25
//const uint32_t FG_NET_CTRLS_VERSION = 27;


/* 
  Defines a structure containing the control parameters
  
*/

class autoconv_FGNetCtrls {

public:

    autoconv_FGNetCtrls(): version{FG_NET_CTRLS_VERSION}{}

    static constexpr uint32_t max_engines = FGNetCtrls::FG_MAX_ENGINES;
    static constexpr uint32_t max_wheels = FGNetCtrls::FG_MAX_WHEELS;
    static constexpr uint32_t max_tanks = FGNetCtrls::FG_MAX_TANKS;

    // distinguish booleans
    using  bool32 = uint32_t;
private:
    quan::network_variable<uint32_t> version;		         // increment when data values change
public:
    // Aero controls
    quan::network_variable<double> aileron;		         // -1 ... 1
    quan::network_variable<double> elevator;		         // -1 ... 1
    quan::network_variable<double> rudder;		         // -1 ... 1
    quan::network_variable<double> aileron_trim;	         // -1 ... 1
    quan::network_variable<double> elevator_trim;	         // -1 ... 1
    quan::network_variable<double> rudder_trim;		         // -1 ... 1
    quan::network_variable<double> flaps;		         //  0 ... 1
    quan::network_variable<double> spoilers;
    quan::network_variable<double> speedbrake;

    // Aero control faults
    quan::network_variable<bool32> flaps_power;                 // true = flaps serviceable
    quan::network_variable<bool32> flap_motor_ok;               // true = motor ok

    // Engine controls
    quan::network_variable<uint32_t> num_engines;		 // number of valid engines
    quan::network_variable<bool32> master_bat[max_engines];
    quan::network_variable<bool32> master_alt[max_engines];
    quan::network_variable<uint32_t> magnetos[max_engines];
    quan::network_variable<bool32> starter_power[max_engines];// true = starter power
    quan::network_variable<double> throttle[max_engines];     //  0 ... 1
    quan::network_variable<double> mixture[max_engines];      //  0 ... 1
    quan::network_variable<double> condition[max_engines];    //  0 ... 1
    quan::network_variable<bool32> fuel_pump_power[max_engines];// true = on
    quan::network_variable<double> prop_advance[max_engines]; //  0 ... 1
    quan::network_variable<uint32_t> feed_tank_to[4];
    quan::network_variable<uint32_t> reverse[4];


    // Engine faults
    quan::network_variable<bool32> engine_ok[max_engines];
    quan::network_variable<bool32> mag_left_ok[max_engines];
    quan::network_variable<bool32> mag_right_ok[max_engines];
    quan::network_variable<bool32> spark_plugs_ok[max_engines];  // false = fouled plugs
    quan::network_variable<uint32_t> oil_press_status[max_engines];// 0 = normal, 1 = low, 2 = full fail
    quan::network_variable<bool32> fuel_pump_ok[max_engines];

    // Fuel management
    quan::network_variable<uint32_t> num_tanks;                      // number of valid tanks
    quan::network_variable<bool32> fuel_selector[max_tanks];    // false = off, true = on
    quan::network_variable<uint32_t> xfer_pump[5];                   // specifies transfer from array
                                             // value tank to tank specified by
                                             // int value
    quan::network_variable<bool32> cross_feed;                     // false = off, true = on

    // Brake controls
    quan::network_variable<double> brake_left;
    quan::network_variable<double> brake_right;
    quan::network_variable<double> copilot_brake_left;
    quan::network_variable<double> copilot_brake_right;
    quan::network_variable<double> brake_parking;
    
    // Landing Gear
    quan::network_variable<bool32> gear_handle; // true=gear handle down; false= gear handle up

    // Switches
    quan::network_variable<uint32_t> master_avionics;
    
        // nav and Comm
    quan::network_variable<double>	comm_1;
    quan::network_variable<double>	comm_2;
    quan::network_variable<double>	nav_1;
    quan::network_variable<double>	nav_2;

    // wind and turbulance
    quan::network_variable<quan::velocity_<double>::knot> wind_speed_kt;
    quan::network_variable<quan::angle_<double>::deg> wind_dir_deg;
    quan::network_variable<double> turbulence_norm;

    // temp and pressure
    quan::network_variable<quan::temperature_<double>::C > temp_c;
    quan::network_variable<quan::pressure_<double>::inHg> press_inhg;

    // other information about environment
    quan::network_variable<quan::length_<double>::m> hground;		         // ground elevation (meters)
    quan::network_variable<quan::angle_<double>::deg> magvar;		         // local magnetic variation in degs.

    // hazards
    quan::network_variable<uint32_t> icing;                      // icing status could me much
                                         // more complex but I'm
                                         // starting simple here.

    // simulation control
    quan::network_variable<uint32_t> speedup;		         // integer speedup multiplier
    quan::network_variable<uint32_t> freeze;		         // 0=normal
				         // 0x01=master
				         // 0x02=position
				         // 0x04=fuel

    // --- New since FlightGear 0.9.10 (FG_NET_CTRLS_VERSION = 27)

    // --- Add new variables just before this line.
private:
    quan::network_variable<uint32_t> reserved[RESERVED_SPACE];	 // 100 bytes reserved for future use.
};


#endif // _NET_CTRLS_HXX


