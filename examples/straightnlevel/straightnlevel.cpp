
#include <cstring>
#include <string>

#include <iostream>
#include <chrono>
#include <thread>

#include <quan/utility/timer.hpp>
#include <quan/out/angle.hpp>
#include <quan/fs/get_file_dir.hpp>
#include <quan/constrain.hpp>
#include <quan/conversion/chrono.hpp>

#include <fgfs_telnet.hpp>
#include <fgfs_fdm_in.hpp>
#include <manual_flight_controller.hpp>
#include <sl_controller.hpp>
#include <flight_mode.hpp>
#include <joystick.hpp>
#include <sensors.hpp>

#include <quan/three_d/vect.hpp>
#include <quan/three_d/quat.hpp>
#include <quan/angular_velocity.hpp>
/*
 Copyright (C) Andy Little 2021
 Derived from https://sourceforge.net/p/flightgear/flightgear/ci/next/tree/scripts/example/fgfsclient.cxx
*/

/**
 * @file
 *  FlightGear IO. Basic requirements for SITL.
 * Read NET_FDM data structure from Flightgear. Display some values.
 * Write control values to FlightGear from joystick using telnet
**/

QUAN_USING_ANGULAR_VELOCITY

namespace {


  using namespace std::chrono_literals;

   /**
    * @brief user defined time literal e.g 100_us
    **/
   QUAN_QUANTITY_LITERAL(time,ms);
   QUAN_QUANTITY_LITERAL(time,s);

   quan::time::ms time_step = 100_ms;

   // indirect system floating point type e.g for microcontrollers rpi etc
   using float_type = quan::quantity_traits::default_value_type;
 
   /**
   *  @brief Display some FDM data to stdout to show what is what
   *  @param fdm - The readonly fdm retrieved from FlightGear
   **/
   void output_fdm(autoconv_FGNetFDM const & fdm)
   {
      quan::angle::deg const roll = fdm.phi.get();
      quan::angle::deg const pitch = fdm.theta.get();
      quan::angle::deg const yaw = fdm.psi.get();
/*
      deg_per_s const roll_rate = fdm.phidot.get();
      deg_per_s const pitch_rate = fdm.thetadot.get();
      deg_per_s const yaw_rate = fdm.psidor.get();
*/
      fprintf(stdout,"\rx=%6.1f y=%6.1f z=%6.1ff,ts= %6.1f",
         roll.numeric_value(),
         pitch.numeric_value(),
         yaw.numeric_value(),
         time_step.numeric_value()
      );
      fflush(stdout);
   }

   /**
    * @brief telnet read is not available immediately at start up
   **/
   void wait_initialised(fgfs_telnet & telnet)
   {
      for(;;){
         try {
            quan::frequency::Hz const frame_rate = get_frame_rate(telnet);
            return;
         }catch(...){
            fprintf(stdout,"Waiting for Telnet interface to be ready\n");
            std::this_thread::sleep_for( 1s);
            continue;
         }
      }
   }

   // call after telnet and fdm constructed
   bool setup(fgfs_fdm_in const & fdm, fgfs_telnet & telnet)
   {
     // Now Telnet interface takes a while until it can be read
      wait_initialised(telnet);
      {  // required to init framerate
         quan::frequency::Hz const frame_rate = get_frame_rate(telnet);
         fprintf(stdout,"framerate = %f Hz\n",frame_rate.numeric_value());
      }
      return true;
   }
}

int main(const int argc, const char *argv[])
{
   int pid = fork();
   if (pid == 0){
     ///@brief run flightgear in child process
      auto const path = quan::fs::get_file_dir(argv[0]) + "/exec_flightgear.sh";
      return system(path.c_str());
   }else{
     if ( pid > 0){
         try {
            
            fprintf(stdout, "Flightgear fc demo\n");

            // create the joystick
            //joystick_t js();

            // create the class to receive fdm from FlightGear
            fgfs_fdm_in fdm_in("localhost",5600);

            // wait for FlightGear to start sending net_fdm packets
            while ( !fdm_in.poll(1.0_s) ){
               fprintf(stdout, "Waiting for FlightGear to start...\n");
            }

            // Only once fdm is running, telnet setup should succeed
            fgfs_telnet telnet_out("localhost", 5501);
            
            // initialise sensors etc 
            setup(fdm_in,telnet_out);
            /**
             * Have joystick input, fdm and telnet so...
             * Create manual controller and plug in telnet and joystick. 
             **/
            manual_flight_controller mfc(telnet_out,"/dev/input/js0");

            sl_controller slc{telnet_out};

            flight_mode cur_flight_mode = flight_mode::Manual;
            abc_flight_controller* fc = &mfc;

            fprintf(stdout,"FlightGear running\n");

            // OK start control loop.
            // Joystick should now be controlling aircraft in FlightGear
            for (;;){
               auto const now = std::chrono::steady_clock::now();
               time_step = 1.f/get_frame_rate(telnet_out);
               
               /**
                 * @brief We should run at Flightgear fdm update rate, set on cmdline in --telnet... to 30 times a sec
                 * Here is the elastic, if FlightGear is late
               **/
               if( fdm_in.poll(10.0_s)){
                  fdm_in.update();
                  output_fdm(fdm_in.get_fdm());
                  auto fm = get_flight_mode();
                  if (fm != cur_flight_mode){
                     cur_flight_mode = fm;
                     if(fm == flight_mode::Manual) {
                        fc = &mfc;
                     }else{
                        fc = &slc;
                     }
                  }
                  if (!fc->update(fdm_in.get_fdm(),time_step)){
                     fprintf(stdout,"flight controller update failed - quitting\n");
                     break;
                  }
               }else{
                  fprintf(stdout,"FlightGear FDM update more than 10 s late");
               }
               // wake up just before the next fdm packet is available from FlightGear (hopefully!)
               //@todo wakeup on SIGIO ?, 
               std::this_thread::sleep_until(now + 19ms);
            }
            return EXIT_SUCCESS;
         } catch (const char s[]) {
            std::cerr << "Error: " << s << ": " << strerror(errno) << std::endl;
            return EXIT_FAILURE;
         } catch (std::exception & e){
            std::cerr << "Error: " << e.what() << std::endl;
            return EXIT_FAILURE;
         } catch (...) {
            std::cerr << "Error: unknown exception" << std::endl;
            return EXIT_FAILURE;
         }
      }else{
         std::cout << "fork failed\n";
         return -1;
      }
   }
}

