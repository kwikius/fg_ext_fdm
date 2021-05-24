
// Copyright (C) 2021 Andy Little 
//
// derived from net_fdm.hxx 
// Copyrigh (c) Curtis Olson - http://www.flightgear.org/~curt
// Started September 2001.
//
// This file is in the Public Domain, and comes with no warranty.

#ifndef AUTOCONV_NET_FDM_HXX
#define AUTOCONV_NET_FDM_HXX

#include <net_fdm.hxx>

#include <quan/network_variable.hpp>

#include <quan/length.hpp>
#include <quan/volume_flow.hpp>
#include <quan/velocity.hpp>
#include <quan/volume.hpp>
#include <quan/pressure.hpp>
#include <quan/reciprocal_time.hpp>
#include <quan/acceleration.hpp>
#include <quan/temperature.hpp>
#include <quan/angle.hpp>


// NOTE: this file defines an external interface structure.  Due to
// variability between platforms and architectures, we only used fixed
// length types here.  Specifically, integer types can vary in length.
// I am not aware of any platforms that don't use 4 bytes for float
// and 8 bytes for double.
/*
##########################################################################
   NOTE: the elements of autoconvFGNetFDM automagically convert to<->from network byte order
   no need to use hton and ntoh conversion functions
   see <quan/network_variable.hpp> for implementation
##########################################################################
*/

class autoconv_FGNetFDM {

   static_assert(sizeof(int) == 4,"");
   static_assert(sizeof(double) == 8,""); 

public:

   template <typename T = float>
   using rad = typename quan::angle_<T>::rad;

   template <typename T = float>
   using deg = typename quan::angle_<T>::deg;

   template <typename T = float>
   using meters = typename quan::length_<T>::m;

   template <typename T = float>
   using rad_per_s = typename quan::reciprocal_time_<
      rad<T>
   >::per_s ;

   template <typename T = float>
   using rev_per_min = typename quan::reciprocal_time_<
      typename quan::angle_<T>::rev
   >::per_min ;

   template <typename T = float>
   using ft_per_s = typename quan::velocity_<T>::ft_per_s;

   template <typename T = float>
   using knots = typename quan::velocity_<T>::knot;

   template <typename T = float>
   using gallons = typename quan::volume_<T>::gal;
   
   template <typename T = float>
   using ft_per_s2 = typename  quan::acceleration_<T>::ft_per_s2;

   template <typename T = float>
   using gal_per_hr = typename quan::volume_flow_<T>::gal_per_hr;

   template <typename T>
   using deg_F = typename quan::temperature_<T>::F;

   /**
      @todo temperature::F
   **/

    autoconv_FGNetFDM()
    : version{FG_NET_FDM_VERSION}
    {}

    static constexpr uint32_t max_engines = FGNetFDM::FG_MAX_ENGINES;
    static constexpr uint32_t max_wheels = FGNetFDM::FG_MAX_WHEELS;
    static constexpr uint32_t max_tanks = FGNetFDM::FG_MAX_TANKS;
private:
    quan::network_variable<uint32_t> version;		            // incremented when data values change

    quan::network_variable<uint32_t> padding;		            // padding
public:
    // Positions
    quan::network_variable<rad<double> > longitude;		      // geodetic (radians)
    quan::network_variable<rad<double> > latitude;		         // geodetic (radians)
    quan::network_variable<meters<double> > altitude;		      // above sea level (meters)

    quan::network_variable<meters<> > agl;			            // above ground level (meters)
    quan::network_variable<rad<> > phi;			               // roll (radians)
    quan::network_variable<rad<> > theta;		                  // pitch (radians)
    quan::network_variable<rad<> > psi;			               // yaw or true heading (radians)
    quan::network_variable<rad<> > alpha;                      // angle of attack (radians)
    quan::network_variable<rad<> > beta;                       // side slip angle (radians)

    // Velocities
    quan::network_variable<rad_per_s<> > phidot;		         // roll rate (radians/sec)
    quan::network_variable<rad_per_s<> > thetadot;		         // pitch rate (radians/sec)
    quan::network_variable<rad_per_s<> > psidot;		         // yaw rate (radians/sec)
    quan::network_variable<knots<> > vcas;		               // calibrated airspeed knots
    quan::network_variable<ft_per_s<> > climb_rate;		      // feet per second
    quan::network_variable<ft_per_s<> > v_north;               // north velocity in local/body frame, fps
    quan::network_variable<ft_per_s<> > v_east;                // east velocity in local/body frame, fps
    quan::network_variable<ft_per_s<> > v_down;                // down/vertical velocity in local/body frame, fps
    quan::network_variable<ft_per_s<> > v_body_u;              // ECEF velocity in body frame fps
    quan::network_variable<ft_per_s<> > v_body_v;              // ECEF velocity in body frame fps
    quan::network_variable<ft_per_s<> > v_body_w;              // ECEF velocity in body frame fps
    
    // Accelerations
    quan::network_variable<ft_per_s2<> > A_X_pilot;		      // X accel in body frame ft/sec^2
    quan::network_variable<ft_per_s2<> > A_Y_pilot;		      // Y accel in body frame ft/sec^2
    quan::network_variable<ft_per_s2<> > A_Z_pilot;		      // Z accel in body frame ft/sec^2

    // Stall
    quan::network_variable<float> stall_warning;               // 0.0 - 1.0 indicating the amount of stall
    quan::network_variable<quan::angle_<float>::deg> slip_deg; // slip ball deflection degrees?

    // Engine status
    quan::network_variable<uint32_t> num_engines;	            // Number of valid engines
    quan::network_variable<uint32_t> eng_state[max_engines];   // Engine state (off, cranking, running)
    quan::network_variable<rev_per_min<> > rpm[max_engines];	// Engine RPM rev/min
    quan::network_variable<gal_per_hr<float> > fuel_flow[max_engines];      // Fuel flow gallons/hr
    quan::network_variable<quan::pressure_<float>::psi> fuel_px[max_engines];        // Fuel pressure psi
    quan::network_variable<deg_F<float> > egt[max_engines];	         // Exhaust gas temp deg F
    quan::network_variable<deg_F<float> > cht[max_engines];	         // Cylinder head temp deg F
    quan::network_variable<quan::pressure_<float>::psi> mp_osi[max_engines];         // Manifold pressure
    quan::network_variable<deg_F<float> > tit[max_engines];	         // Turbine Inlet Temperature
    quan::network_variable<deg_F<float> > oil_temp[max_engines];       // Oil temp deg F
    quan::network_variable<quan::pressure_<float>::psi> oil_px[max_engines];         // Oil pressure psi

    // Consumables
    quan::network_variable<uint32_t>   num_tanks;		            // Max number of fuel tanks
    quan::network_variable<gallons<> > fuel_quantity[max_tanks];

    // Gear status
    quan::network_variable<uint32_t> num_wheels;
    quan::network_variable<uint32_t> wow[max_wheels];  // assume it's a bool true if wheel is pushed up by ground?
    quan::network_variable<float> gear_pos[max_wheels];
    quan::network_variable<float> gear_steer[max_wheels];
    quan::network_variable<float> gear_compression[max_wheels];

    // Environment
    quan::network_variable<uint32_t> cur_time;           // current unix time
                                 // FIXME: make this uint64_t before 2038
    quan::network_variable<int32_t> warp;                // offset in seconds to unix time
    quan::network_variable<meters<> > visibility;            // visibility in meters (for env. effects)

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
