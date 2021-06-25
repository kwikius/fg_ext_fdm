

#include <quan/constrain.hpp>
#include <quan/three_d/rotation.hpp>
#include <quan/angular_velocity.hpp>
#include <quan/atan2.hpp>
#include <quan/three_d/vect.hpp>
#include <quan/three_d/make_vect.hpp>
#include <quan/abs.hpp>
#include "get_sl_torque.hpp"

namespace {

   QUAN_QUANTITY_LITERAL(angle, deg)

   /// @brief derive proportional torque required from ailerons to return to straight and level flight.
   quan::torque::N_m 
   get_P_torque_x(
      quan::three_d::vect< quan::three_d::vect<double> >const & body_frame_v, 
      quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & inertia_v, 
      quan::reciprocal_time2::per_s2 const & accelK
   )
   {
      /// @brief get z-rotation of Body x-axis to align body_frame_v.x vertically with W.x in x z plane
      auto const rotBWx = quan::three_d::z_rotation(-quan::atan2(body_frame_v.x.y,body_frame_v.x.x));
      auto const BWx = make_vect(
         rotBWx(body_frame_v.x), 
         rotBWx(body_frame_v.y),
         rotBWx(body_frame_v.z)
      );
#if defined QUAN_STRAIGHT_N_LEVEL_FILTER
      /// @brief make a limit for x and y error angles inversely proportional to how far BWX.x is from W.x
      quan::angle::rad const theta_lim = quan::atan2(quan::abs(BWx.x.x),quan::abs(BWx.x.z))/2;
#else
      quan::angle::rad const theta_lim = 45_deg;
#endif
      /// @brief y roll error component 
      quan::angle::rad const rxy = quan::constrain(quan::atan2(BWx.y.z,BWx.y.y),-theta_lim,theta_lim);
      /// @brief z roll error component
      quan::angle::rad const rxz = quan::constrain(-quan::atan2(BWx.z.y,BWx.z.z),-theta_lim,theta_lim);

#if defined QUAN_STRAIGHT_N_LEVEL_FILTER
      /// resultant torque is scaled scale by quan::abs cosine of angle of Bwx with W.x Bw_x.x.x 
      quan::torque::N_m const torque_x = quan::abs(BWx.x.x) * (rxy * inertia_v.y + rxz * inertia_v.z  ) * accelK;
#else
      quan::torque::N_m const torque_x = (rxy * inertia_v.y + rxz * inertia_v.z  ) * accelK;
#endif
      return torque_x;
   }

 /// @brief derive proportional torque for elevator to return to straight and level flight
   quan::torque::N_m 
   get_P_torque_y(
      quan::three_d::vect< quan::three_d::vect<double> >const & body_frame_v, 
      quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & inertia_v, 
      quan::reciprocal_time2::per_s2 const & accelK
   )
   {
      /// @brief get zrotation of y-axis to align horizontally with world y in yz plane
      /// and rotate axes to this frame
      auto const rotBWy = quan::three_d::z_rotation(quan::atan2(body_frame_v.y.x,body_frame_v.y.y));
      auto const BWy = make_vect(
         rotBWy(body_frame_v.x), 
         rotBWy(body_frame_v.y),
         rotBWy(body_frame_v.z)
      );
#if defined QUAN_STRAIGHT_N_LEVEL_FILTER
      // limit the error angle to at most +- 45 deg but scaled inverse to how far Bwy.y is from W.y
      quan::angle::rad const theta_lim = quan::atan2(quan::abs(BWy.y.y),quan::abs(BWy.y.z))/2;
#else
      quan::angle::rad const theta_lim = 45_deg;
#endif
      // x component
      quan::angle::rad const ryx = quan::constrain(-quan::atan2(BWy.x.z,BWy.x.x),-theta_lim,theta_lim);
      // z component
      quan::angle::rad const ryz = quan::constrain(quan::atan2(BWy.z.x,BWy.z.z),-theta_lim,theta_lim);

#if defined QUAN_STRAIGHT_N_LEVEL_FILTER
      // scale by quan::abs cosine of angle of Bwy with W.y  
      quan::torque::N_m const torque_y = quan::abs(BWy.y.y) * (ryx * inertia_v.x + ryz * inertia_v.z  ) * accelK;
#else
      quan::torque::N_m const torque_y = (ryx * inertia_v.x + ryz * inertia_v.z  ) * accelK;
#endif
      return torque_y;
   }

   /// @brief derive proportional torque from rudder to return to strraight and level flight
   quan::torque::N_m 
   get_P_torque_z(
      quan::three_d::vect< quan::three_d::vect<double> >const & body_frame_v, 
      quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & inertia_v, 
      quan::reciprocal_time2::per_s2 const & accelK
   )
   {
     // get xrotation of z-axis to align vertically with world z in zx plane
      auto const rotBWz = quan::three_d::x_rotation(quan::atan2(body_frame_v.z.y,body_frame_v.z.z));
      auto const BWz = make_vect(
         rotBWz(body_frame_v.x), 
         rotBWz(body_frame_v.y),
         rotBWz(body_frame_v.z)
      );

#if defined QUAN_STRAIGHT_N_LEVEL_FILTER
      quan::angle::rad const theta_lim = quan::atan2(quan::abs(body_frame_v.z.z),quan::sqrt(quan::pow<2>(body_frame_v.z.x) + quan::pow<2>(body_frame_v.z.y)))/2;
#else
      quan::angle::rad const theta_lim = 45_deg;
#endif
      // x component
      quan::angle::rad const rzx = quan::constrain(quan::atan2(BWz.x.y,BWz.x.x),-theta_lim,theta_lim);
      // z component
      quan::angle::rad const rzy = quan::constrain(-quan::atan2(BWz.y.x,BWz.y.y),-theta_lim,theta_lim);

#if defined QUAN_STRAIGHT_N_LEVEL_FILTER
      // scale by quan::abs cosine of angle of body_frame_v.x with W.z
      quan::torque::N_m torque_z = quan::abs(body_frame_v.z.z) * (rzx * inertia_v.x + rzy * inertia_v.y ) * accelK;
#else
       quan::torque::N_m torque_z = (rzx * inertia_v.x + rzy * inertia_v.y ) * accelK;
#endif
      return torque_z;
   }

} // namespace

quan::three_d::vect<quan::torque::N_m> 
get_P_torque(
   quan::three_d::vect< quan::three_d::vect<double> > const & body_frame_v, 
   quan::three_d::vect<quan::moment_of_inertia::kg_m2> const & inertia_v, 
   quan::reciprocal_time2::per_s2 const & accelK
)
{
   return {
      get_P_torque_x(body_frame_v,inertia_v,accelK),
      get_P_torque_y(body_frame_v,inertia_v,accelK),
      get_P_torque_z(body_frame_v,inertia_v,accelK)
   };
}


