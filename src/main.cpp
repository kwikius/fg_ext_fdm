

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
  Copyright (C) 2021 Andy Little see GNU GENERAL PUBLIC LICENSE V3

  see COPYING 

  Derived from :

  https://sourceforge.net/p/flightgear/flightgear/ci/next/tree/examples/netfdm/
  Copyright (C) David Calkin

  simgear https://github.com/FlightGear/simgear/math/SGQuat.hxx
  Copyright (C) 2006-2009  Mathias Froehlich - Mathias.Froehlich@web.de

  crrcsim https://sourceforge.net/projects/crrcsim
  Copyright (C) Jens Wilhelm Wulf
*/

/**
 * @brief Control Flighgear externally . Plane is suspended in space
 * Use the joystick to pose the model in roll , pitch and yaw
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
    * @todo make cmd line param
   **/
   bool use_model_frame = true;

   void update(pose_t & result,quan::joystick const & js)
   {
     update_turnrate(js);

     if (use_model_frame){
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
      * @brief function to turn raw joystick number into a representation 
      * of the joystick axis in range  -1 to 1  ...
      **/
      auto get_js_percent = [&js](int32_t i)->double {
         return static_cast<double>(js.get_channel(i) * js_sign[i]) / joystick_half_range ;
      };
     /**
      * @brief ... so work out stick percent
      **/
      quan::three_d::vect<double> const stick_percent = {
         get_js_percent(roll_idx), 
         get_js_percent(pitch_idx),
         get_js_percent(yaw_idx)
      };
      
      /// @brief calc turn rates vector per element
      for ( int32_t i = 0; i < 3; ++i){
         turn_rate[i] = max_turn_rate[i] * stick_percent[i] ;
      }
    }

     /**
      * @brief modulo pose vector to 0..360 degrees
      **/
    void normalise_pose(pose_t & pose)
    {
      for ( int32_t i = 0; i < 3; ++i){
         while ( pose[i] < 0.0_deg){
            pose[i] += 360.0_deg;
         }
         while ( pose[i] >= 360.0_deg){
            pose[i] -= 360.0_deg;
         }
      }
    }
    /**
     * @brief put pose to stdout
     **/
    void print_pose(pose_t & pose)
    {
      quan::three_d::vect<quan::angle::deg> const deg = pose;
      fprintf(stdout,"\rroll = %3f pitch = %3f yaw = %3f"
         ,deg.x.numeric_value()
         ,deg.y.numeric_value()
         ,deg.z.numeric_value()
      );
      fflush(stdout);
    }

   /**
    * @brief update current pose using world frame according to joystick positions
   **/
   void update_world_frame( pose_t & pose)
   {
      pose += turn_rate * update_period;
      normalise_pose(pose);
      print_pose(pose);
   }
   /**
    *   @brief update fdm structuer with current pose
    **/
   void update(FGNetFDM & fdm, pose_t const & pose )  
   {
      fdm.phi = htonf(static_cast<float>(pose.x.numeric_value())) ;
      fdm.theta = htonf(static_cast<float>(pose.y.numeric_value()) );
      fdm.psi = htonf(static_cast<float>(pose.z.numeric_value()));
   };

   /**
    * @brief quaternion holding current pose. ( only reqd if using model_frame)
   **/
   quan::three_d::quat<double> qpose{1.0,0.0,0.0,0.0};

   /**
    * @brief update model position from current turnrate using modelframe
   **/
   void update_model_frame( pose_t & pose)
   {
      auto const turn = turn_rate * update_period;
      auto const magturn = magnitude(turn);
      if ( magturn > 0.001_rad){
         auto const qturn = quatFrom(unit_vector(turn),magturn);
         qpose = unit_quat(hamilton_product(qpose,qturn));
         pose = euler_from_quat(qpose);
         normalise_pose(pose);
      }
      print_pose(pose);  
   }

   /**
    * wrap usleep with quan::time type
   **/
   void usleep(quan::time::us const & t)
   {
      ::usleep(static_cast<unsigned long>(t.numeric_value()));
   }
}
