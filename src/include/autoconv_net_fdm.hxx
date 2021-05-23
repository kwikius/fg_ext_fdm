// net_fdm.hxx -- defines a common net I/O interface to the flight
//                dynamics model
//
// Written by Curtis Olson - http://www.flightgear.org/~curt
// Started September 2001.
//
// This file is in the Public Domain, and comes with no warranty.
//
// $Id$


#ifndef AUTOCONV_NET_FDM_HXX
#define AUTOCONV_NET_FDM_HXX

//#include <time.h> // time_t

#include <quan/network_variable.hpp>
#include <net_fdm.h>


// NOTE: this file defines an external interface structure.  Due to
// variability between platforms and architectures, we only used fixed
// length types here.  Specifically, integer types can vary in length.
// I am not aware of any platforms that don't use 4 bytes for float
// and 8 bytes for double.
/*
##########################################################################
   note that the elements automagically convert to from network byte order
   no need to use conversion functions
##########################################################################
*/

//const uint32_t FG_NET_FDM_VERSION = 24;


// Define a structure containing the top level flight dynamics model
// parameters

class autoconv_FGNetFDM {

    static_assert(sizeof(int) == 4,"");
    static_assert(sizeof(double) == 8,""); 

    autoconv_FGNetFDM()
    : version{FG_NET_FDM_VERSION}{}

public:

    static constexpr uint32_t max_engines = FGNetFDM::FG_MAX_ENGINES;
    static constexpr uint32_t max_wheels = FGNetFDM::FG_MAX_WHEELS;
    static constexpr uint32_t max_tanks = FGNetFDM::FG_MAX_TANKS;

    quan::network_variable<uint32_t> m_version;		// increment when data values change
    quan::network_variable<uint32_t> m_padding;		// padding

    // Positions
    quan::network_variable<double> longitude;		// geodetic (radians)
    quan::network_variable<double> latitude;		// geodetic (radians)
    quan::network_variable<double> altitude;		// above sea level (meters)
    quan::network_variable<float> agl;			// above ground level (meters)
    quan::network_variable<float> phi;			// roll (radians)
    quan::network_variable<float> theta;		// pitch (radians)
    quan::network_variable<float> psi;			// yaw or true heading (radians)
    quan::network_variable<float> alpha;                // angle of attack (radians)
    quan::network_variable<float> beta;                 // side slip angle (radians)

    // Velocities
    quan::network_variable<float> phidot;		// roll rate (radians/sec)
    quan::network_variable<float> thetadot;		// pitch rate (radians/sec)
    quan::network_variable<float> psidot;		// yaw rate (radians/sec)
    quan::network_variable<float> vcas;		        // calibrated airspeed
    quan::network_variable<float> climb_rate;		// feet per second
    quan::network_variable<float> v_north;              // north velocity in local/body frame, fps
    quan::network_variable<float> v_east;               // east velocity in local/body frame, fps
    quan::network_variable<float> v_down;               // down/vertical velocity in local/body frame, fps
    quan::network_variable<float> v_body_u;    // ECEF velocity in body frame
    quan::network_variable<float> v_body_v;    // ECEF velocity in body frame 
    quan::network_variable<float> v_body_w;    // ECEF velocity in body frame
    
    // Accelerations
    quan::network_variable<float> A_X_pilot;		// X accel in body frame ft/sec^2
    quan::network_variable<float> A_Y_pilot;		// Y accel in body frame ft/sec^2
    quan::network_variable<float> A_Z_pilot;		// Z accel in body frame ft/sec^2

    // Stall
    quan::network_variable<float> stall_warning; // 0.0 - 1.0 indicating the amount of stall
    quan::network_variable<float> slip_deg;		 // slip ball deflection

    // Pressure
    
    // Engine status
    quan::network_variable<uint32_t> num_engines;	     // Number of valid engines
    quan::network_variable<uint32_t> eng_state[FG_MAX_ENGINES];// Engine state (off, cranking, running)
    quan::network_variable<float> rpm[FG_MAX_ENGINES];	     // Engine RPM rev/min
    quan::network_variable<float> fuel_flow[FG_MAX_ENGINES]; // Fuel flow gallons/hr
    quan::network_variable<float> fuel_px[FG_MAX_ENGINES];   // Fuel pressure psi
    quan::network_variable<float> egt[FG_MAX_ENGINES];	     // Exhuast gas temp deg F
    quan::network_variable<float> cht[FG_MAX_ENGINES];	     // Cylinder head temp deg F
    quan::network_variable<float> mp_osi[FG_MAX_ENGINES];    // Manifold pressure
    quan::network_variable<float> tit[FG_MAX_ENGINES];	     // Turbine Inlet Temperature
    quan::network_variable<float> oil_temp[FG_MAX_ENGINES];  // Oil temp deg F
    quan::network_variable<float> oil_px[FG_MAX_ENGINES];    // Oil pressure psi

    // Consumables
    quan::network_variable<uint32_t> num_tanks;		// Max number of fuel tanks
    quan::network_variable<float> fuel_quantity[FG_MAX_TANKS];

    // Gear status
    quan::network_variable<uint32_t> num_wheels;
    quan::network_variable<uint32_t> wow[FG_MAX_WHEELS];
    quan::network_variable<float> gear_pos[FG_MAX_WHEELS];
    quan::network_variable<float> gear_steer[FG_MAX_WHEELS];
    quan::network_variable<float> gear_compression[FG_MAX_WHEELS];

    // Environment
    quan::network_variable<uint32_t> cur_time;           // current unix time
                                 // FIXME: make this uint64_t before 2038
    quan::network_variable<int32_t> warp;                // offset in seconds to unix time
    quan::network_variable<float> visibility;            // visibility in meters (for env. effects)

    // Control surface positions (normalized values)
    quan::network_variable<float> elevator;
    quan::network_variable<float> elevator_trim_tab;
    quan::network_variable<float> left_flap;
    quan::network_variable<float> right_flap;
    quan::network_variable<float> left_aileron;
    quan::network_variable<float> right_aileron;
    quan::network_variable<float> rudder;
    quan::network_variable<float> nose_wheel;
    quan::network_variable<float> speedbrake;
    quan::network_variable<float> spoilers;
};


#endif // AUTOCONV_NET_FDM_HXX
