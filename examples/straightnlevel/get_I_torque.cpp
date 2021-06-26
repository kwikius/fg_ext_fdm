


#include <quan/constrain.hpp>
#include <quan/three_d/rotation.hpp>
#include <quan/angular_velocity.hpp>
#include <quan/atan2.hpp>
#include <quan/three_d/vect.hpp>
#include <quan/three_d/make_vect.hpp>
#include <quan/abs.hpp>
#include "get_sl_torque.hpp"

namespace {

   QUAN_QUANTITY_LITERAL(torque, N_m)
   QUAN_QUANTITY_LITERAL(angle, deg)

#if defined QUAN_STRAIGHT_N_LEVEL_FILTER
   /// @todo approx this by linear function maybe?
   constexpr int ScaleDownPowN = 8;
#endif

   /// @todo torque integral constant time_step is divided by
   auto constexpr kIntegral = quan::pow<3>(quan::time::s{0.5});
   constexpr auto torque_lim_All = 0.01_N_m;

   /// @ brief limiting torque per axis
   quan::three_d::vect<quan::torque::N_m> constexpr torque_lim = {
      0_N_m,
      0.01_N_m,
      0.0_N_m
   };

   quan::torque::N_m 
   get_I_torque_xx(
      quan::three_d::vect< quan::three_d::vect<double> >const & body_frame_v, 
      quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & inertia_v, 
      quan::reciprocal_time2::per_s2 const & accelK
   ){

  /// @brief get z-rotation of Body x-axis to align body_frame_v.x vertically with W.x in x z plane
      auto const rotBWx = quan::three_d::z_rotation(-quan::atan2(body_frame_v.x.y,body_frame_v.x.x));
      auto const BWx = quan::three_d::make_vect(
         rotBWx(body_frame_v.x), 
         rotBWx(body_frame_v.y),
         rotBWx(body_frame_v.z)
      );

      /// @brief make a limit for x and y error angles inversely proportional to how far BWX.x is from W.x
#if defined QUAN_STRAIGHT_N_LEVEL_FILTER
      quan::angle::rad const errorAngleLim = quan::atan2(quan::abs(BWx.x.x),quan::abs(BWx.x.z))/2;
#else
      quan::angle::rad constexpr errorAngleLim = 45_deg;
#endif
      /// @brief y roll error component 
      quan::angle::rad const rxy = quan::constrain(quan::atan2(BWx.y.z,BWx.y.y),-errorAngleLim,errorAngleLim);
      /// @brief z roll error component
      quan::angle::rad const rxz = quan::constrain(-quan::atan2(BWx.z.y,BWx.z.z),-errorAngleLim,errorAngleLim);
      /// resultant torque is scaled by abs cosine of angle of Bwx with W.x Bw_x.x.x 
#if defined QUAN_STRAIGHT_N_LEVEL_FILTER
      quan::torque::N_m torque_x = quan::pow<ScaleDownPowN>(quan::abs(BWx.x.x)) * (rxy * inertia_v.y + rxz * inertia_v.z  ) * accelK;
#else
    quan::torque::N_m torque_x = (rxy * inertia_v.y + rxz * inertia_v.z  ) * accelK;
#endif
      return torque_x;
   }

 /// @brief derive integral torque for elevator to return to straight and level flight
   quan::torque::N_m 
   get_I_torque_yy(
      quan::three_d::vect< quan::three_d::vect<double> >const & body_frame_v, 
      quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & inertia_v, 
      quan::reciprocal_time2::per_s2 const & accelK
   )
   {
      /// @brief get zrotation of y-axis to align horizontally with world y in yz plane
      auto const rotBWy = quan::three_d::z_rotation(quan::atan2(body_frame_v.y.x,body_frame_v.y.y));
      /// @brief get axes to this frame
      auto const BWy = quan::three_d::make_vect(
         rotBWy(body_frame_v.x), 
         rotBWy(body_frame_v.y),
         rotBWy(body_frame_v.z)
      );
#if defined QUAN_STRAIGHT_N_LEVEL_FILTER
      quan::angle::rad const errorAngleLim = quan::atan2(abs(BWy.y.y),abs(BWy.y.z))/2;
#else
      quan::angle::rad constexpr errorAngleLim = 45_deg;
#endif
      // x component
      quan::angle::rad const ryx = quan::constrain(-quan::atan2(BWy.x.z,BWy.x.x),-errorAngleLim,errorAngleLim);
      // z component
      quan::angle::rad const ryz = quan::constrain(quan::atan2(BWy.z.x,BWy.z.z),-errorAngleLim,errorAngleLim);

#if defined QUAN_STRAIGHT_N_LEVEL_FILTER
      // scale by abs cosine of angle of Bwy with W.y  
      quan::torque::N_m const torque_y = quan::pow<ScaleDownPowN>(quan::abs(BWy.y.y)) * (ryx * inertia_v.x + ryz * inertia_v.z  ) * accelK;
#else
      quan::torque::N_m const torque_y =  (ryx * inertia_v.x + ryz * inertia_v.z  ) * accelK;
#endif

      return torque_y;
   }

/// @brief derive integral torque from rudder to return to straight and level flight
   quan::torque::N_m get_I_torque_zz(
      quan::three_d::vect< quan::three_d::vect<double> >const & body_frame_v, 
      quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & inertia_v, 
      quan::reciprocal_time2::per_s2 const & accelK
   )
   {
     // get xrotation of z-axis to align vertically with world z in zx plane
      auto const rotBWz = quan::three_d::x_rotation(quan::atan2(body_frame_v.z.y,body_frame_v.z.z));
      auto const BWz = quan::three_d::make_vect(
         rotBWz(body_frame_v.x), 
         rotBWz(body_frame_v.y),
         rotBWz(body_frame_v.z)
      );
#if defined QUAN_STRAIGHT_N_LEVEL_FILTER
      quan::angle::rad const errorAngleLim = quan::atan2(abs(body_frame_v.z.z),quan::sqrt(quan::pow<2>(body_frame_v.z.x) + quan::pow<2>(body_frame_v.z.y)))/2;
#else
      quan::angle::rad constexpr errorAngleLim = 45_deg;
#endif
      // x component
      quan::angle::rad const rzx = quan::constrain(quan::atan2(BWz.x.y,BWz.x.x),-errorAngleLim,errorAngleLim);
      // z component
      quan::angle::rad const rzy = quan::constrain(-quan::atan2(BWz.y.x,BWz.y.y),-errorAngleLim,errorAngleLim);

#if defined QUAN_STRAIGHT_N_LEVEL_FILTER
      // scale by abs cosine of angle of body_frame_v.x with W.z
      quan::torque::N_m const torque_z = quan::pow<ScaleDownPowN>(quan::abs(body_frame_v.z.z)) * (rzx * inertia_v.x + rzy * inertia_v.y ) * accelK;
#else
      quan::torque::N_m const torque_z = (rzx * inertia_v.x + rzy * inertia_v.y ) * accelK;
#endif

      return torque_z;
   }

   /// @brief torque integrator
   quan::three_d::vect<quan::torque::N_m> torque_integral = {
      0.0_N_m,0.0_N_m,0.0_N_m
   };
}

quan::three_d::vect<quan::torque::N_m>
get_I_torque(
   quan::three_d::vect< quan::three_d::vect<double> > const & body_frame_v, 
   quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & inertia_v, 
   quan::time::ms const & dt
)
{
   auto const accelK = dt/kIntegral ;
   quan::three_d::vect<quan::torque::N_m> const torque_time_step = {
      get_I_torque_xx(body_frame_v,inertia_v,accelK),
      get_I_torque_yy(body_frame_v,inertia_v,accelK),
      get_I_torque_zz(body_frame_v,inertia_v,accelK)
   };

   torque_integral.x = quan::constrain(torque_integral.x + torque_time_step.x , -torque_lim.x,torque_lim.x);
   torque_integral.y = quan::constrain(torque_integral.y + torque_time_step.y, -torque_lim.y,torque_lim.y);
   torque_integral.z = quan::constrain(torque_integral.z + torque_time_step.z , -torque_lim.z,torque_lim.z);

   return torque_integral;
}