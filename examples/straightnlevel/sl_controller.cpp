
#include <time.h>
#include <sl_controller.hpp>

#include <quan/constrain.hpp>
#include <quan/mass.hpp>
#include <quan/length.hpp>
#include <quan/out/angle.hpp>
#include <quan/out/time.hpp>
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
   QUAN_QUANTITY_LITERAL(reciprocal_time,per_s)

   /// @brief World Frame axis unit vectors
   auto constexpr W = make_vect(
      quan::three_d::vect<double>{1,0,0},
      quan::three_d::vect<double>{0,1,0},
      quan::three_d::vect<double>{0,0,1} // n.b +z is down
   );

   /// @brief derive new angular velocity from joystick positions
   quan::three_d::vect<rad_per_s>
   get_angular_velocity( autoconv_FGNetFDM const & fdm)
   {
      return {
         -fdm.phidot.get(),
          fdm.thetadot.get(),
         -fdm.psidot.get()
      };
   }

   //// @brief target heading updated at intervals in autopilot mode
   quan::angle::deg targetHeading = 45_deg;
   /// @brief periodic change of heading 
   quan::angle::deg constexpr heading_incr = 90_deg;

   uint64_t gettime()
   {
      timespec ts;
      clock_gettime(CLOCK_MONOTONIC,&ts);
      return static_cast<uint64_t>(ts.tv_sec) * 1000 + static_cast<uint64_t>(ts.tv_nsec)/1000000;
   }
   uint64_t frame_start_time = 0;

   quan::angle::deg 
   constrain_angle(quan::angle::deg a)
   {
      while ( a  > 180_deg){
         a -= 360_deg;
      }
      while(a <= -180_deg){
         a+= 360_deg;
      }
      return a;
   }
}

bool sl_controller::pre_update(autoconv_FGNetFDM const & fdm, quan::time::ms const & time_step) 
{
   auto const new_frame_start_time = gettime();

   if ( (new_frame_start_time - frame_start_time) >= 60000){
      targetHeading = constrain_angle(targetHeading + heading_incr);
      frame_start_time = new_frame_start_time;
      printf("New heading : % 6.2f deg\n",targetHeading.numeric_value());
   }

   quan::angle::deg const currentHeading = constrain_angle(fdm.psi.get());
   quan::angle::deg const headingError = constrain_angle(targetHeading - currentHeading);

#if defined FG_EASYSTAR
   quan::time::s const yawRateErrorToRollAngleGain = 13_s;
   //target yaw rate gain 
   auto const headingErrorToYawRateGain = 0.125_per_s;

   quan::angle::deg const glide_pitch_angle = -0.2_deg;
   // max_yaw_rate
   rad_per_s const max_yaw_rate = 90.0_deg_per_s;
#else
   quan::time::s const yawRateErrorToRollAngleGain = 4_s;
   quan::angle::deg const glide_pitch_angle = 0_deg;
   rad_per_s const max_yaw_rate = 90.0_deg_per_s;
#endif

   /// @brief target yaw rate to turn the aircraft to target heading
   rad_per_s const target_yaw_rate = 
   quan::constrain(
      headingError*headingErrorToYawRateGain, 
         -max_yaw_rate, 
          max_yaw_rate
      );

   /// @brief control correction to apply to ailerons to get desired yaw rate
   quan::angle::deg const roll_rate_correction = 
   -quan::constrain( 
      ( target_yaw_rate-fdm.psidot.get()) * yawRateErrorToRollAngleGain,
           -90_deg,
            90_deg
      );

   /// @brief desired pose for desired turn as euler angles
   // Note that we ignore yaw throughout, just relying on roll to turn the aircraft
   quan::three_d::vect<quan::angle::deg> target_pose = {
      roll_rate_correction, 
      glide_pitch_angle,
      0_deg
   };

   /// @brief current pose as quaternion ignoring yaw
   quan::three_d::quat<double> qCurrentPose = 
      quan::three_d::quat_from_euler<double>(
         quan::three_d::vect<quan::angle::rad>{
            -fdm.phi.get(),
            fdm.theta.get(),
            0.0_rad
         }
      );
   
   auto const qTargetPose = unit_quat(quat_from_euler<double>(target_pose));

   auto const qPoseError = unit_quat(hamilton_product(qCurrentPose, conjugate(qTargetPose)));
   /// @brief Body Frame axis unit vectors
   auto const body_frame_v = make_vect(
      qPoseError * W.x,
      qPoseError * W.y,
      qPoseError * W.z
   );

   auto const & inertia_v = the_aircraft.get_inertia();
   // accumulate torque PID terms
   quan::three_d::vect<quan::torque::N_m> torque = 
      get_P_torque(
            body_frame_v,inertia_v,the_aircraft.get_Kp()
      )
      // Note: It is easier to let the actual values sit with some deflection to avoid integrator windup
      // + get_I_torque(body_frame_v,inertia_v,time_step)
      + get_D_torque(
            get_angular_velocity(fdm),
            inertia_v,
            the_aircraft.get_Kd()
         );

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

// we dont need yaw . We can control the aircraft via pitch and roll
sl_controller::float_type sl_controller::get_yaw() const  
{
   return 0.0;//the_aircraft.get_yaw_control_value();
}

