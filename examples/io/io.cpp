
#include <cstring>
#include <string>

#include <iostream>
#include <chrono>
#include <thread>

#include <quan/utility/timer.hpp>
#include <quan/out/angle.hpp>
#include <quan/fs/get_file_dir.hpp>
#include <quan/constrain.hpp>

#include "fgfs_telnet.hpp"
#include "fgfs_fdm_in.hpp"
#include <flight_controller.hpp>
#include <joystick.hpp>
/*
 Copyright (C) Andy Little 2021
 Derived from https://sourceforge.net/p/flightgear/flightgear/ci/next/tree/scripts/example/fgfsclient.cxx
*/

/**
 * @file
 *  FlightGear IO. Basic requirements for SITL.
 * On joystick ch 8 mode command fly straight and level
**/

namespace {

   /**
    * @brief user defined time literal e.g 100_us
    **/
   QUAN_QUANTITY_LITERAL(time,us);
   QUAN_QUANTITY_LITERAL(time,s);

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
      auto const alt = fdm.altitude.get(); //
      auto const agl = fdm.agl.get();
      quan::velocity::m_per_s const airspeed = fdm.vcas.get();
      fprintf(stdout,"\rr=%6.1f p=%6.1f y=%6.1f a=%6.1f ag=%6.1f v= %5.1f",
         roll.numeric_value(),
         pitch.numeric_value(),
         yaw.numeric_value(),
         alt.numeric_value(),
         agl.numeric_value(),
         airspeed.numeric_value()
      );
      fflush(stdout);
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
            using namespace std::chrono_literals;

            fprintf(stdout, "Flightgear io demo\n");

            // create the joystick
            joystick_t js("/dev/input/js0");

            // create the class to receive fdm from FlightGear
            fgfs_fdm_in fdm_in("localhost",5600);

            // wait for FlightGear to start sending packets
            while ( !fdm_in.poll(1.0_s) ){
               fprintf(stdout, "Waiting for FlightGear to start...\n");
            }

            // Only once FlightGear is running, telnet setup should succeed
            fgfs_telnet telnet_out("localhost", 5501);

            /**
             * Have joystick input, fdm and telnet so...
             * Create flight controller and plug in telnet and joystick. 
             **/
            flight_controller fc(telnet_out,js);

            //for luck, check we are still getting data from FlightGear...
            while(!fdm_in.poll(1.0_s)){
                fprintf(stdout,"FlightGear initialising...\n");
            }

            fprintf(stdout,"FlightGear running\n");

            // OK start control loop.
            // Joystick should now be controlling aircraft in FlightGear
            for (;;){
               auto const now = std::chrono::steady_clock::now();
               /**
                 * @brief We should run at Flightgear fdm update rate, set on cmdline in --telnet... to 50 times a sec
                 * Here is the elastic, if FlightGear is late
               **/
               if( fdm_in.poll(10.0_s)){
                  fdm_in.update();
                  output_fdm(fdm_in.get_fdm());
                  if (!fc.update(fdm_in.get_fdm())){
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

