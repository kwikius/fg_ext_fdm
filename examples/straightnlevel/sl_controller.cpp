#include <sl_controller.hpp>

#include <quan/constrain.hpp>

#include <quan/mass.hpp>
#include <quan/length.hpp>
#include <quan/out/angle.hpp>
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
   QUAN_QUANTITY_LITERAL(angle,rad)
   QUAN_QUANTITY_LITERAL(time,ms)
   QUAN_QUANTITY_LITERAL(time,s)

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

        - fdm.phidot.get(),
         fdm.thetadot.get(),
         -fdm.psidot.get()
      };
   }

   quan::three_d::quat<double>
   get_pose( autoconv_FGNetFDM const & fdm)
   {
      quan::three_d::vect<quan::angle::rad> const euler 
         = {-fdm.phi.get(),fdm.theta.get(),-fdm.psi.get()};
      return quan::three_d::quat_from_euler<double>(euler);
   }

   quan::three_d::vect<quan::angle::deg> target_pose
   ={
      10_deg,
         0_deg,
            90_deg
   };
}

bool sl_controller::pre_update(autoconv_FGNetFDM const & fdm, quan::time::ms const & time_step) 
{
   the_aircraft.set_angular_velocity(get_angular_velocity(fdm));
   the_aircraft.set_pose(get_pose(fdm));

   auto const & qpose = the_aircraft.get_pose();

   quan::angle::deg target_heading = 25_deg;
   quan::angle::deg current_heading = fdm.psi.get();

   if ( target_heading > 180_deg){
     target_heading = target_heading - 360_deg;
   }
   if ( current_heading > 180_deg){
     current_heading = current_heading - 360_deg;
   }
   quan::angle::deg heading_diff = (target_heading - current_heading);
   if ( heading_diff > 180_deg){
      heading_diff -= 360_deg;
   }else{
      if (heading_diff <= -180_deg){
         heading_diff += 360_deg;
      }
   }

#if defined FG_EASYSTAR
   double const diff_roll_gain = 0.5; 
   quan::time::s const g1 = 10_s;
   quan::time::s const g2 = 10_s;
#else
   double const diff_roll_gain = 0.25; 
   quan::time::s g1 = 4_s;
#endif

   rad_per_s const max_yaw_rate = 30.0_deg_per_s;
   rad_per_s const target_yaw_rate = quan::constrain(heading_diff/g2, - max_yaw_rate, max_yaw_rate);

   quan::angle::deg diff_roll0 = -quan::constrain(heading_diff * diff_roll_gain,-45_deg, 45_deg);
   quan::angle::deg diff_roll1 = quan::constrain( ( target_yaw_rate-fdm.psidot.get()) * g1,- 90_deg,90_deg);
   quan::angle::deg diff_roll = diff_roll0 - diff_roll1   ;

      std::cout << "hd = " << heading_diff <<'\n';
      std::cout << "dr0 = " << diff_roll0 <<'\n';
      std::cout << "dr1 = " << diff_roll1 <<"\n";
      std::cout << "dr = " << diff_roll <<"\n\n";
   quan::three_d::vect<quan::angle::deg> target_pose = {
      diff_roll, 
#if defined FG_EASYSTAR
      -0.2_deg,
#else
      0_deg,
#endif
      0_deg
   };
    // pose without yaw
   quan::three_d::quat<double> qpose1 = 
   quan::three_d::quat_from_euler<double>(
      quan::three_d::vect<quan::angle::rad>{-fdm.phi.get(),fdm.theta.get(),0.0_rad}
   );
   
   auto const qtarget_pose = unit_quat(quat_from_euler<double>(target_pose));

   auto const qdiff1 = unit_quat(hamilton_product(qpose1, conjugate(qtarget_pose)));
   /// @brief Body Frame axis unit vectors
   auto const body_frame_v = make_vect(
      qdiff1 * W.x,
      qdiff1 * W.y,
      qdiff1 * W.z
   );

   auto const & inertia_v = the_aircraft.get_inertia();
   // accumulate torque PID terms
   quan::three_d::vect<quan::torque::N_m> torque = 
      get_P_torque(body_frame_v,inertia_v,the_aircraft.get_Kp())
      // + get_I_torque(body_frame_v,inertia_v,time_step)
         + get_D_torque(the_aircraft.get_angular_velocity(),inertia_v,the_aircraft.get_Kd()) 
   ;
   the_aircraft.set_control_torque(torque);
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
   return 0.0;//the_aircraft.get_yaw_control_value();
}

