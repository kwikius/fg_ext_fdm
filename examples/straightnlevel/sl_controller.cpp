#include <sl_controller.hpp>

#include <quan/constrain.hpp>

#include <quan/mass.hpp>
#include <quan/length.hpp>

#include <quan/three_d/rotation.hpp>
#include <quan/angular_velocity.hpp>
#include <quan/atan2.hpp>

#include <quan/three_d/make_vect.hpp>
#include <quan/three_d/rotation_from.hpp>

#include "aircraft.hpp"
#include "get_sl_torque.hpp"

QUAN_USING_ANGULAR_VELOCITY

namespace {
   aircraft the_aircraft;


   /// @brief local quantity literals
   QUAN_QUANTITY_LITERAL(angle,deg)
   QUAN_QUANTITY_LITERAL(time,ms)

   /// @brief World Frame axis unit vectors
   auto constexpr W = make_vect(
      quan::three_d::vect<double>{1,0,0},
      quan::three_d::vect<double>{0,1,0},
      quan::three_d::vect<double>{0,0,1} // n.b +z is down
   );

   /// @brief defines what a full joystick +ve deflection means per axis
   quan::three_d::vect<rad_per_s> const max_angular_velocity
    = {
       90.0_deg_per_s,   //roll
        90.0_deg_per_s,  //pitch
          90.0_deg_per_s  //yaw
   };

   /// @brief derive new angular velocity from joystick positions
   quan::three_d::vect<rad_per_s>
   get_angular_velocity( autoconv_FGNetFDM const & fdm)
   {
      return {
         fdm.phidot.get(),
         fdm.thetadot.get(),
         fdm.psidot.get()
      };
   }

   quan::three_d::quat<double>
   get_pose( autoconv_FGNetFDM const & fdm)
   {
      quan::three_d::vect<quan::angle::deg> const euler 
         = {-fdm.phi.get(),fdm.theta.get(),fdm.psi.get()};
      return quan::three_d::quat_from_euler<double>(euler);
   }

   /// @brief update aircraft pose from current anguar velocity for next timestep
   void update_body_frame(autoconv_FGNetFDM const & fdm,quan::time::ms const & time_step)
   {

      the_aircraft.set_angular_velocity(get_angular_velocity(fdm));
      the_aircraft.set_pose(get_pose(fdm));
/*
      auto const turn = angular_velocity * time_step;
      auto const magturn = magnitude(turn);
      if ( magturn > 0.001_deg){
         auto const qturn = quatFrom(unit_vector(turn),magturn);
         auto const qpose = unit_quat(hamilton_product(the_aircraft.get_pose(),qturn));

         the_aircraft.set_pose(qpose);
         the_aircraft.set_angular_velocity(angular_velocity);
      }
*/
      auto const & qpose = the_aircraft.get_pose();

      /// @brief Body Frame axis unit vectors
      auto const body_frame_v = make_vect(
         qpose * W.x,
         qpose * W.y,
         qpose * W.z
      );

      auto const & inertia_v = the_aircraft.get_inertia();
      // accumulate torque PID terms
      quan::three_d::vect<quan::torque::N_m> torque = 
         get_P_torque(body_frame_v,inertia_v,the_aircraft.get_Kp())
        //  + get_I_torque(body_frame_v,inertia_v,time_step)
            + get_D_torque(the_aircraft.get_angular_velocity(),inertia_v,the_aircraft.get_Kd()) 
      ;
      the_aircraft.set_control_torque(torque);
   }

   /// @brief simulation time step 
   quan::time::ms constexpr sim_time_step = 33_ms;
}

bool sl_controller::pre_update(autoconv_FGNetFDM const & fdm, quan::time::ms const & time_step) 
{
   update_body_frame(fdm,time_step);
   return true;
}

sl_controller::float_type sl_controller::get_roll() const
{
   return the_aircraft.get_roll_control_value();
} 

sl_controller::float_type sl_controller::get_pitch() const 
{
   return the_aircraft.get_pitch_control_value();
}

sl_controller::float_type sl_controller::get_yaw() const  
{
   return the_aircraft.get_yaw_control_value();
}

