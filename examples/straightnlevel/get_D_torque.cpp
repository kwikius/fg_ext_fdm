

#include <quan/max.hpp>
#include "get_sl_torque.hpp"

/**
*  @brief derive differential aileron torque
**/

QUAN_USING_ANGULAR_VELOCITY
namespace {
     QUAN_QUANTITY_LITERAL(time,s)
}

quan::three_d::vect<quan::torque::N_m> 
get_D_torque(
   quan::three_d::vect<quan::angular_velocity::rad_per_s> const & turn_rate, 
   quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & inertia_v,
   quan::time::s const & tstop
)
{
   auto const tstopL = quan::max(tstop,0.01_s);
   quan::three_d::vect<quan::torque::N_m> torque = {
      turn_rate.x *( inertia_v.y + inertia_v.z) / tstopL,
      turn_rate.y *( inertia_v.x + inertia_v.z) / tstopL,
      turn_rate.z *( inertia_v.x + inertia_v.y) / tstopL
   };  

   return torque;
}