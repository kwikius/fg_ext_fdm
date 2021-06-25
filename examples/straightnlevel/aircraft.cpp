
#include <quan/constrain.hpp>


#include <quan/time.hpp>
#include <quan/length.hpp>
#include <quan/torque.hpp>
#include <quan/mass.hpp>
#include <quan/moment_of_inertia.hpp>
//#include <straight_n_level/pid/get_sl_torque.hpp>

#include "aircraft.hpp"

QUAN_USING_ANGULAR_VELOCITY

aircraft::aircraft()
: m_pose{1,0,0,0},
m_angular_velocity{rad_per_s{0},rad_per_s{0},rad_per_s{0}}
{}

namespace {

   QUAN_QUANTITY_LITERAL(angle,deg)
   QUAN_QUANTITY_LITERAL(angle,rad)
   QUAN_QUANTITY_LITERAL(moment_of_inertia,kg_m2)
   QUAN_QUANTITY_LITERAL(length,m)
   QUAN_QUANTITY_LITERAL(mass,kg)
   QUAN_QUANTITY_LITERAL(torque, N_m)
   QUAN_QUANTITY_LITERAL(time,ms)
   QUAN_QUANTITY_LITERAL(time,s)

   /// @brief for differential error stopping time from current angular velocity
   auto constexpr tstop = 1.0_s;

   ///  @brief correcting angular accel limit
   auto constexpr accelK = 1.0/quan::pow<2>(tstop);
}

quan::time::s aircraft::get_Kd() const  { return tstop*1.175 ;}

quan::reciprocal_time2::per_s2 aircraft::get_Kp() const { return accelK;}

namespace{


   /// @brief point masses on each axis
   quan::three_d::vect<quan::mass::kg> constexpr mass = {
      0.7_kg, //along x axis
      0.8_kg, //along y axis
      0.1_kg // along z axis
   };

   /// @brief point mass distances on each axis
   quan::three_d::vect<quan::length::m> constexpr  dist = {
     0.5_m, //along x axis
     0.8_m, //along y axis
     0.1_m // along z axis
   };

}

/// @brief point mass inertias on each axis
quan::three_d::vect<quan::moment_of_inertia::kg_m2> 
aircraft::get_inertia()const
{
  return {
       mass.x * quan::pow<2>(dist.x),
       mass.y * quan::pow<2>(dist.y),
       mass.z * quan::pow<2>(dist.z)
   };
}

namespace{

   /// @brief scaling torque per degree of control deflection per axis
   auto constexpr torque_per_deg = quan::three_d::make_vect(
      1.0_N_m/ 1_rad, // aileron
      1.0_N_m/ 1_rad,// elevator
      1.0_N_m/ 1_rad// rudder
   );

   /// @brief limit of allowable control deflection
   quan::angle::rad constexpr control_defl_lim = 45_deg;


   quan::three_d::vect<quan::torque::N_m> constexpr max_torque = torque_per_deg * quan::angle::rad{control_defl_lim};

}

float aircraft::get_roll_control_value() const
{
   return quan::constrain(m_control_torque.x / max_torque.x, -1.0,1.0);
}
float aircraft::get_pitch_control_value() const
{
   return quan::constrain(m_control_torque.y / max_torque.y, -1.0,1.0);
}

float aircraft::get_yaw_control_value() const
{
   return quan::constrain(m_control_torque.z / max_torque.z, -1.0,1.0);
}

//quan::three_d::vect<quan::angle::deg> aircraft::get_control_deflections() const
//{
//   return{  
//      quan::constrain(-m_control_torque.x/torque_per_deg.x,-control_defl_lim,control_defl_lim),
//      quan::constrain(-m_control_torque.y/torque_per_deg.y,-control_defl_lim,control_defl_lim),
//      quan::constrain(-m_control_torque.z/torque_per_deg.z,-control_defl_lim,control_defl_lim)
//   };
//}
