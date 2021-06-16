

#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <autoconv_net_fdm.hpp>
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
#include <quan/fs/get_file_dir.hpp>

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
 * @brief Control Flightgear externally . Plane is suspended in space
 * Use the joystick to pose the model in roll , pitch and yaw
**/

namespace {

   static_assert( sizeof(autoconv_FGNetFDM) == sizeof(FGNetFDM) ,"");

   QUAN_QUANTITY_LITERAL(time,us);
   QUAN_QUANTITY_LITERAL(angle,deg);
   QUAN_QUANTITY_LITERAL(angle,rad);
   QUAN_QUANTITY_LITERAL(length,m); 
   QUAN_QUANTITY_LITERAL(volume,gal);    

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

   constexpr inline 
   rad_per_s operator "" _rad_per_s ( long double v)
   {
      return deg_per_s{quan::angle::rad{v}};
   }

   using pose_t = quan::three_d::vect<quan::angle::rad>; 

   using turn_rate_t = quan::three_d::vect<rad_per_s>; 

   using stick_percent_t = quan::three_d::vect<double>;

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
    *  @brief current model pose in World/Model Frame dependent on cmd line args
    *  @param x = roll  ( x axis)
    *  @param y = pitch ( y axis)
    *  @param z = yaw   ( z axis)
    **/
   pose_t pose;  

   /**
    *  @brief current model joystick percent of full range
    *    between +1 .. -1. Only for pitch roll and yaw
    *  
    *  @param x = roll 
    *  @param y = pitch 
    *  @param z = yaw
    **/
   stick_percent_t stick_percent;
   
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

   bool setup_fdm_out_socket();
   void update_turnrate(quan::joystick const & js);
   void update_world_frame(pose_t & result);
   void update_model_frame(pose_t & result);
   void update(autoconv_FGNetFDM & fdm, pose_t const & pose); 
   void usleep(quan::time::us const & t);
   void run();

   int process_args(int argc, char ** argv);
}

int main(int argc, char ** argv)
{
   int pid = fork();
   if (pid == 0){
     ///@brief run flightgear in child process
      auto const path = quan::fs::get_file_dir(argv[0]) + "/exec_flightgear.sh";
      return system(path.c_str());
   }else{
      if (process_args(argc, argv) != 0){
         return 1;
      }
     
      if ( setup_fdm_out_socket()){
         run();
      }
      return 0;
   }
}

namespace {

   bool use_model_frame = true;

   /**
    * @brief process command line arguments
   **/
   int process_args(int argc, char ** argv)
   {
      for(;;){
         int const c = getopt(argc, argv, "r:");
         if ( c == -1){
            return 0;
         }
         switch(c){
            case 'r':{
               char const* arg = optarg;
               if ( strcmp(arg,"euler") == 0){
                  use_model_frame = false;
               }else {
                  if (strcmp(arg,"quat") == 0){
                     use_model_frame = true;
                  }else{
                     fprintf(stderr,"Unknown rotation type for -r \"%s\", options are \"euler\" or \"quat\"\n",arg);
                     return -1;
                  }
               }
            }
            break;
            case '?':{
               if (optopt == 'r'){
                  fprintf (stderr, "Option -r requires an argument.\n");
               } else {
                  if (isprint (optopt)){
                     fprintf (stderr, "Unknown option `-%c'.\n", optopt);
                  } else{
                     fprintf (stderr,"Unknown option character `\\x%x'.\n",optopt);
                  }
               }   
            }// fall through
            default:
            return -1;
         }
      }
   }

   /**
    * @brief socket IP and port where FlightGear is listening
    **/
   int fdmSendSocket = -1;
   struct sockaddr_in fdmSendAddr;
   constexpr char localhost[] = "127.0.0.1";
   int constexpr fdmSendPort = 5500;

   bool setup_fdm_out_socket()
   {
      memset(&fdmSendAddr,0,sizeof(fdmSendAddr));
      fdmSendAddr.sin_family = AF_INET;
      fdmSendAddr.sin_port = htons(fdmSendPort);

      if ( inet_pton(AF_INET, localhost, &fdmSendAddr.sin_addr) != 1){
         fprintf(stderr,"convert ip address failed\n");
         exit(-1);
      }

      fdmSendSocket = socket(AF_INET,SOCK_DGRAM,0);
      if (fdmSendSocket  >= 0){
         return true;
      }else {
         perror("socket() failed");
         exit(errno);
         return false;
      }
   }

   void setup(autoconv_FGNetFDM & fdm)
   {
    /** ##############################################
     * N.B the host to network conversion is automatic
     * no need to use hton#(v)
     * ###############################################
    **/
      fdm.latitude =           50.7381_deg;
      fdm.longitude =           0.2494_deg;
      fdm.altitude =          250.0_m;  //a.s.l
      fdm.num_engines =         1;
      fdm.num_tanks =           1;
      fdm.fuel_quantity[0] =  100.0_gal;
      fdm.num_wheels =          3;
      fdm.cur_time =       time(0);
      fdm.warp =                1;
      fdm.visibility =       1500.0_m;
   }

   /**
    * @brief use model frame or worldframe for joystick 
    * @todo make cmd line param
   **/
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
      autoconv_FGNetFDM fdm;
      setup(fdm);
      quan::joystick js{"/dev/input/js0"};

      for(;;){
         usleep(update_period);
         update(pose,js);
         update(fdm,pose);
         sendto(fdmSendSocket,(void *)&fdm,sizeof(fdm),0,(struct sockaddr *)&fdmSendAddr,sizeof(fdmSendAddr));
      }
    }

    void update_stick_percent(quan::joystick const & js)
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
      stick_percent = {
         get_js_percent(roll_idx), 
         get_js_percent(pitch_idx),
         get_js_percent(yaw_idx)
      };
    }

   void update_turnrate(quan::joystick const & js)
   {
      update_stick_percent(js);

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
    * @brief quaternion holding current pose. ( only used if using model_frame)
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
    *   @brief update fdm structure with current pose
    **/
   void update(autoconv_FGNetFDM & fdm, pose_t const & pose )  
   {
    /** ##############################################
     * N.B the host to network conversion is automatic
     * no need to use hton#(v)
     * ###############################################
    **/
      fdm.phi = pose.x;
      fdm.theta = pose.y;
      fdm.psi = pose.z;
      fdm.left_aileron = stick_percent.x;
      fdm.right_aileron = -stick_percent.x;
      fdm.elevator = -stick_percent.y;
      fdm.rudder = stick_percent.z;
      fdm.left_flap = 0.f;
      fdm.right_flap = 0.f;
   };

   /**
    * wrap usleep with quan::time type
   **/
   void usleep(quan::time::us const & t)
   {
      ::usleep(static_cast<unsigned long>(t.numeric_value()));
   }
}
