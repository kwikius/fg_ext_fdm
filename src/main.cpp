

#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <net_fdm.hxx>
#include <arpa/inet.h>
#include <byte_order.hpp>

#include <quan/joystick.hpp>
#include <quan/angle.hpp>
#include <quan/three_d/out/vect.hpp>
#include <quan/three_d/out/quat.hpp>
#include <quan/time.hpp>
#include <quan/length.hpp>
#include <quan/reciprocal_time.hpp>
#include <quan/frequency.hpp>
#include <quan/abs.hpp>
#include <quan/constants/constant.hpp>

/*
  Derived from https://sourceforge.net/p/flightgear/flightgear/ci/next/tree/examples/netfdm/
  Copyright David Calkin
*/

/**
 * @brief Control Flighgear externally . Plane is suspended in space
 * Use the joystick to pose the model in roll , pitch and yaw
 * @todo  Currently Strictly only works when the model is in a stable upright pose
 * Otherwise stick movements arent correlated with the attitude of the model
 * Could instead use a quaternion to add the stick rotation
 * then convert to angles for output
**/

namespace {

   QUAN_QUANTITY_LITERAL(time,us);
   QUAN_QUANTITY_LITERAL(angle,deg);
   QUAN_QUANTITY_LITERAL(angle,rad);
   QUAN_QUANTITY_LITERAL(length,m);   

   using rad_per_s = quan::reciprocal_time_<
      quan::angle::rad 
   >::per_s ;

   using deg_per_s = quan::reciprocal_time_<
      quan::angle::deg 
   >::per_s ;

   constexpr inline 
   deg_per_s operator "" _deg_per_s ( long double v)
   {
      return deg_per_s{quan::angle::deg{v}};
   }

   using pose_t = quan::three_d::vect<quan::angle::rad>; 
   using turn_rate_t = quan::three_d::vect<rad_per_s>; 

   /**
    * @brief simulation frame period
    * @todo we could provide the option to update in realtime 
    * and record that here
   */
   auto constexpr update_period = 20000.0_us;

   /**
    * @brief simulation update frequency
    **/
   quan::frequency::Hz constexpr update_frequency = 1.0 / update_period; 

   /**
    *  @brief current model pose in World Frame
    *  @param x = roll  ( x axis)
    *  @param y = pitch ( y axis)
    *  @param z = yaw   ( z axis)
    **/
   pose_t pose;  

   /**
    *  @brief current model turn rate in World Frame
    *  updated from joystick.
    *
    *  @param x = roll rate
    *  @param y = pitch rate
    *  @param z = yaw rate
    **/
   turn_rate_t turn_rate;

   /**
    *  @brief max model +- turn rate for full stick move
    *  @param x = roll rate
    *  @param y = pitch rate
    *  @param z = yaw rate
    **/
   turn_rate_t const max_turn_rate
    = {
       180.0_deg_per_s,   //roll
        180.0_deg_per_s,  //pitch
          360.0_deg_per_s  //yaw
   };

   /**
    *  @brief Joystick to Flightgear output values
    *  
    *    aileron stick increasing values means roll clockwise looking from pilots view
    *    elevator stick increasing values means pitch down at nose
    *    rudder stick increasing values means yaw nose right
    * 
    *    N.b these values are specific to my TX setup
    **/
    static constexpr double joystick_half_range = 32767.0;
    static constexpr uint8_t roll_idx = 0;  // roll on ch 0
    static constexpr uint8_t pitch_idx = 1; // pitch on ch 1
    static constexpr uint8_t throttle_idx = 2;
    static constexpr uint8_t yaw_idx = 3;   // yaw on ch 3

    /**
      @brief joystick channel direction , either 1 or -1
     **/
    static constexpr int32_t js_sign []= {
         1,   // roll
        -1,   // pitch
         1,   // throttle
        -1   // yaw
    };

   bool setup_socket();
   void update_turnrate(quan::joystick const & js);
   void update_world_frame(pose_t & result);
   void update_model_frame(pose_t & result);
   void update(FGNetFDM & fdm, pose_t const & pose); 
   void usleep(quan::time::us const & t);
   void run();
}

int main(int argc, char ** argv)
{
   if ( setup_socket()){
      run();
   }
   return 0;
}

namespace {

   /**
    * @brief socket IP and port where FG is listening
    **/
   int sendSocket = -1;
   struct sockaddr_in sendAddr;
   constexpr char  fg_ip[] = "127.0.0.1";
   int constexpr fg_port = 5500;

   bool setup_socket()
   {
      memset(&sendAddr,0,sizeof(sendAddr));
      sendAddr.sin_family = AF_INET;
      sendAddr.sin_port = htons(fg_port);

      if ( inet_pton(AF_INET, fg_ip, &sendAddr.sin_addr) != 1){
         fprintf(stderr,"convert ip address failed\n");
         exit(-1);
      }

      sendSocket = socket(AF_INET,SOCK_DGRAM,0);
      if (sendSocket  >= 0){
         return true;
      }else {
         perror("socket() failed");
         exit(errno);
         return false;
      }
   }

   void setup(FGNetFDM & fdm)
   {
      memset(&fdm,0,sizeof(fdm));

      fdm.version = htonl(FG_NET_FDM_VERSION);

      quan::angle::rad constexpr latitude = 50.7381_deg;
      quan::angle::rad constexpr longitude = 0.2494_deg;

      auto constexpr altitude = 250.0_m; // above sea level
      auto constexpr visibility = 150.0_m;

      fdm.latitude = htond(latitude.numeric_value());
      fdm.longitude = htond(longitude.numeric_value());
      fdm.altitude = htond(altitude.numeric_value());
      fdm.num_engines = htonl(1);
      fdm.num_tanks = htonl(1);
      fdm.fuel_quantity[0] = htonf(100.0);
      fdm.num_wheels = htonl(3);
      fdm.cur_time = htonl(time(0));
      fdm.warp = htonl(1);
      fdm.visibility = htonf(visibility.numeric_value());
   }

   /**
    * @brief use model frame or worldframe for joystick 
   **/
   bool use_model_frame = true;

   void update(pose_t & result,quan::joystick const & js)
   {
     update_turnrate(js);

     if ( use_model_frame){
        update_model_frame(result);
     }else{
        update_world_frame(result);
     }
   }
   
   void run()
   {
      pose_t pose;
      FGNetFDM fdm;
      setup(fdm);
      quan::joystick js{"/dev/input/js0"};

      for(;;){
         usleep(update_period);
         update(pose,js);
         update(fdm,pose);
         sendto(sendSocket,(void *)&fdm,sizeof(fdm),0,(struct sockaddr *)&sendAddr,sizeof(sendAddr));
      }
    }

    void update_turnrate(quan::joystick const & js)
    {

     /**
      * @brief turn raw joystick number into a representation 
      * of the joystick axis in range  -1 to 1
      **/
      auto get_js_percent = [&js](int32_t i)->double {
         return static_cast<double>(js.get_channel(i) * js_sign[i]) / joystick_half_range ;
      };
     /**
      * @brief work out stick percent
      **/
      quan::three_d::vect<double> const stick_percent = {
         get_js_percent(roll_idx), 
         get_js_percent(pitch_idx),
         get_js_percent(yaw_idx)
      };
      
      /// @brief calc turn rates vector
      for ( int32_t i = 0; i < 3; ++i){
         turn_rate[i] = max_turn_rate[i] * stick_percent[i] ;
      }
    }

   /**
    * @brief update current pose according to joystick positions
   **/
   void update_world_frame( pose_t & result)
   {
     
     /**
      * @brief increment pose vector with new turn_rate
      **/
      result += turn_rate * update_period;

     /**
      * @brief cap pose vector to 0..360 degrees
      **/
      for ( int32_t i = 0; i < 3; ++i){
         // result[i] = modulo(result[i]);
         while ( result[i] < 0.0_deg){
            result[i] += 360.0_deg;
         }
         while ( result[i] >= 360.0_deg){
            result[i] -= 360.0_deg;
         }
      }
      fprintf(stdout,"\rroll = %3f pitch = %3f yaw = %3f"
         ,quan::angle::deg{result[0]}.numeric_value()
         ,quan::angle::deg{result[1]}.numeric_value()
         ,quan::angle::deg{result[2]}.numeric_value()
      );
      fflush(stdout);
   }

   void update(FGNetFDM & fdm, pose_t const & pose )  
   {
/*
    TODO. Should probably provide either 1 or other of postion or angular velocity but not both
    see https://forum.flightgear.org/viewtopic.php?t=2654
*/
      fdm.phi = htonf(static_cast<float>(pose.x.numeric_value())) ;
      fdm.theta = htonf(static_cast<float>(pose.y.numeric_value()) );
      fdm.psi = htonf(static_cast<float>(pose.z.numeric_value()));

      fdm.phidot = htonf(static_cast<float>(turn_rate.x.numeric_value())) ;
      fdm.thetadot = htonf(static_cast<float>(turn_rate.y.numeric_value()) );
      fdm.psidot = htonf(static_cast<float>(turn_rate.z.numeric_value()));
      
   };

/*
  from simgear https://github.com/FlightGear/simgear/math/SGQuat.hxx
  Copyright (C) 2006-2009  Mathias Froehlich - Mathias.Froehlich@web.de
   /// Return a quaternion from euler angles
  static SGQuat fromEulerRad(T z, T y, T x)
  {
    SGQuat q;
    T zd2 = T(0.5)*z; T yd2 = T(0.5)*y; T xd2 = T(0.5)*x;
    T Szd2 = sin(zd2); T Syd2 = sin(yd2); T Sxd2 = sin(xd2);
    T Czd2 = cos(zd2); T Cyd2 = cos(yd2); T Cxd2 = cos(xd2);
    T Cxd2Czd2 = Cxd2*Czd2; T Cxd2Szd2 = Cxd2*Szd2;
    T Sxd2Szd2 = Sxd2*Szd2; T Sxd2Czd2 = Sxd2*Czd2;
    q.w() = Cxd2Czd2*Cyd2 + Sxd2Szd2*Syd2;
    q.x() = Sxd2Czd2*Cyd2 - Cxd2Szd2*Syd2;
    q.y() = Cxd2Czd2*Syd2 + Sxd2Szd2*Cyd2;
    q.z() = Cxd2Szd2*Cyd2 - Sxd2Czd2*Syd2;
    return q;
  }

crrcsim quaternion.cpp

  double sphi   = sin(0.5*eulerAngle.r[0]);  //s.x
  double cphi   = cos(0.5*eulerAngle.r[0]);  //c.x
  double stheta = sin(0.5*eulerAngle.r[1]);  //s.y
  double ctheta = cos(0.5*eulerAngle.r[1]);  //c.y
  double spsi   = sin(0.5*eulerAngle.r[2]);  //s.z
  double cpsi   = cos(0.5*eulerAngle.r[2]);  //c.z

  e0.init(+cpsi*ctheta*cphi +spsi*stheta*sphi, 0);
      e0.init(c.z * c.y * c.x + s.x * s.y * s.z )
         cx * cz * c.y + s.x * s.z * s.y

  e1.init(+cpsi*ctheta*sphi -spsi*stheta*cphi, 0);
      e1.init(c.z * c.y * s.x - s.z * s.y * c.x)
         s.x * c.z * c.y - c.x * s.z * s.y  

  e2.init(+cpsi*stheta*cphi +spsi*ctheta*sphi, 0);
      e2.init(c.z * s.y * c.x + s.z * c.y * s.x)
          c.x * c.z * s.y + s.x * s.z * c.y
  
  e3.init(-cpsi*stheta*sphi +spsi*ctheta*cphi, 0);
     e3.init(-c.z*s.y *s.x + s.z * c.y * c.x);
         c.x * s.z * c.y - s.x * c.z * s.y
*/

   quan::three_d::quat<double> 
   quat_from_ZYXeuler(pose_t const & pose)
   {
      using v3d = quan::three_d::vect<double>;

      v3d const v = 0.5 * pose;
      v3d const s{ sin(v.x),sin(v.y),sin(v.z)};
      v3d const c{ cos(v.x),cos(v.y),cos(v.z)};

      double const cXcZ = c.x * c.z;
      double const cXsZ = c.x * s.z;
      double const sXsZ = s.x * s.z;
      double const sXcZ = s.x * c.x;

      return { 
         cXcZ * c.y + sXsZ * s.y,
         sXcZ * c.y - cXsZ * s.y,
         cXcZ * s.y + sXsZ * c.y,
         cXsZ * c.y - sXcZ * s.y
      };
   }

/*
  from simgear https://github.com/FlightGear/simgear/math/SGQuat.hxx
  Copyright (C) 2006-2009  Mathias Froehlich - Mathias.Froehlich@web.de
  /// write the euler angles into the references
  void getEulerRad(T& zRad, T& yRad, T& xRad) const
  {
    T sqrQW = w()*w();
    T sqrQX = x()*x();
    T sqrQY = y()*y();
    T sqrQZ = z()*z();

    T num = 2*(y()*z() + w()*x());
    T den = sqrQW - sqrQX - sqrQY + sqrQZ;
    if (fabs(den) <= SGLimits<T>::min() &&
        fabs(num) <= SGLimits<T>::min())
      xRad = 0;
    else
      xRad = atan2(num, den);

    T tmp = 2*(x()*z() - w()*y());
    if (tmp <= -1)
      yRad = T(0.5)*SGMisc<T>::pi();
    else if (1 <= tmp)
      yRad = -T(0.5)*SGMisc<T>::pi();
    else
      yRad = -asin(tmp);

    num = 2*(x()*y() + w()*z());
    den = sqrQW + sqrQX - sqrQY - sqrQZ;
    if (fabs(den) <= SGLimits<T>::min() &&
        fabs(num) <= SGLimits<T>::min())
      zRad = 0;
    else {
      T psi = atan2(num, den);
      if (psi < 0)
        psi += 2*SGMisc<T>::pi();
      zRad = psi;
    }
  }

crrcsim
  void CRRCMath::Quaternion_003::updateEuler()
{
  double Phi, Theta, Psi;

  Theta = asin( -1*mat.v[0][2] );

  if( mat.v[0][0] == 0 )
    Psi = 0;
  else
    Psi = atan2( mat.v[0][1], mat.v[0][0] );

  if( mat.v[2][2] == 0 )
    Phi = 0;
  else
    Phi = atan2( mat.v[1][2], mat.v[2][2] );

  // Resolve Psi to 0 - 359.9999 
  if (Psi < 0 )      Psi += 2*M_PI;
  if (Psi >= 2*M_PI) Psi -= 2*M_PI;
  
  // Resolve Phi to 0 - 359.9999 
  if (Phi < 0 )      Phi += 2*M_PI;
  if (Phi >= 2*M_PI) Phi -= 2*M_PI;  
    
  euler.r[0] = Phi;
  euler.r[1] = Theta;
  euler.r[2] = Psi;
}

*/
   quan::three_d::vect<double> 
   quat_to_ZYXeuler(quan::three_d::quat<double> const & q)
   {
      quan::three_d::quat<double> q2 {q.w*q.w,q.x*q.x,q.y * q.y, q.z * q.z};

      double constexpr lim = std::numeric_limits<double>::min();
      pose_t result;
      {
         double const numx = 2.0 * ( q.y * q.z + q.w * q.x);
         double const denx = q2.w - q2.x - q2.y + q2.z;
         if ( (quan::abs(denx) <= lim) && (quan::abs(numx) <= lim) ){
           result.x = 0.0;
         }else{
           result.x = atan2(numx,denx);
         }
      }
      {
         double const tmp = 2.0 * (q.x * q.z - q.w * q.y);
         if ( tmp <= -1.0 ){
            result.y = quan::constant::pi/2.0;
         }else {
            if ( tmp >= 1.0 ){
               result.y = -quan::constant::pi/2.0;
            }else{
               result.y = -asin(tmp);
            }
         }
      }
      {
         double const numz = 2.0 * (q.x * q.y + q.w * q.z);
         double const denz = q2.w + q2.x - q2.y - q2.z;
         if( (quan::abs(denz) <= lim) && (quan::abs(numz) <= lim) ){
            result.z = 0;
         }else{
            double psi = atan2(numz, denz);
            if (psi < 0){
               psi += 2.0*quan::constant::pi;
            }
            result.z = psi;
         }
      }
      return result;
   }

   quan::three_d::quat<double> qpose{1.0,0.0,0.0,0.0};
   void update_model_frame( pose_t & pose)
   {
      auto const turn = turn_rate * update_period;
      auto const magturn = magnitude(turn);
      if ( magturn > 0.001_rad){
         auto const qturn = quatFrom(unit_vector(turn),magturn);
         qpose = unit_quat(hamilton_product(qpose,conjugate(qturn)));
         pose = quat_to_ZYXeuler(qpose);
      }
      auto result = pose;

      fprintf(stdout,"\rroll = %3f pitch = %3f yaw = %3f"
         ,result[0].numeric_value()
         ,result[1].numeric_value()
         ,result[2].numeric_value()
      );
      fflush(stdout);
              
   }

   void usleep(quan::time::us const & t)
   {
      ::usleep(static_cast<unsigned long>(t.numeric_value()));
   }
}
