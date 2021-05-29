
#include <cstring>

#include "fgfs_telnet.hpp"
#include "fgfs_fdm_in.hpp"

#include <quan/joystick.hpp>
#include <quan/utility/timer.hpp>
#include <quan/out/angle.hpp>
#include <quan/fs/get_file_dir.hpp>
#include <string>

#include <iostream>
#include <chrono>
#include <thread>
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

/**
* @todo
https://www.mail-archive.com/flightgear-users@lists.sourceforge.net/msg06993.html
"Via the "telnet" interface you can set "/sim/freeze/master" to true or false
in order to pause the sim."
**/

namespace {

   /**
    * @brief user defined time literal e.g 100_us
    **/
   QUAN_QUANTITY_LITERAL(time,us);
   QUAN_QUANTITY_LITERAL(time,s);

   /**
    *  @brief overload sleep function with squan time type;
    **/
   void usleep(quan::time::us const & t)
   {
      ::usleep(static_cast<unsigned long>(t.numeric_value()));
   }

  /**
   * @brief range of raw Taranis joystick input is nominally += 32767
   **/
   static constexpr double joystick_half_range = 32767.0;
   
   /**
    * @brief channel numbers of joystick channels
   **/
   static constexpr uint8_t roll_idx = 0;  // roll on ch 0
   static constexpr uint8_t pitch_idx = 1; // pitch on ch 1
   static constexpr uint8_t throttle_idx = 2;
   static constexpr uint8_t yaw_idx = 3;   // yaw on ch 3

   /**
    * @brief joystick channel direction , either 1 or -1
    * dependent on joystick setup
   **/
   static constexpr int32_t js_sign []= {
      1,   // roll
      1,   // pitch
      1,   // throttle
      -1   // yaw
   };

   /**
    *  @brief Read values from joystick, convert then
    *  send control values using FlightGear telnet protocol
    *  to access the flightgear property system.
    *  @return true if controls 
    **/
   bool set_controls(fgfs_telnet & telnet_out, quan::joystick const & js )
   {
      try{
         auto get_js_percent = [&js](int32_t i)->double {
            return static_cast<double>(js.get_channel(i) * js_sign[i]) / joystick_half_range ;
         };
         telnet_out.set("/controls/flight/aileron",get_js_percent(roll_idx));
         telnet_out.set("/controls/flight/elevator",get_js_percent(pitch_idx));
         telnet_out.set("/controls/flight/rudder",get_js_percent(yaw_idx));
         telnet_out.set("/controls/engines/engine[0]/throttle", get_js_percent(throttle_idx) + 1.0);
         return true;
      }catch(...){
         // probably flightgear stopped
         return false;
      }
   }
}

namespace {

   /**
   *  @brief Display some FDM data to stdout
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
     // run flightgear in child process
      auto const path =  quan::fs::get_file_dir(argv[0]) + "/exec_flightgear.sh";
      return system(path.c_str());
   }else{
     if ( pid > 0){
         try {
            using namespace std::chrono_literals;
            fprintf(stdout, "Flightgear io demo\n");
            quan::joystick joystick_in{"/dev/input/js0"};

            fgfs_fdm_in fdm_in("localhost",5600);

            // wait for FlightGear to start sending packets
            while ( !fdm_in.poll_fdm(1.0_s) ){
               fprintf(stdout, "Waiting for FlightGear to start...\n");
            }

            // Once FlightGear is running then telnet should succeed
            fgfs_telnet telnet_out("localhost", 5501);
            // but needs time to warm up
            while(!fdm_in.poll_fdm(1.0_s)){
                fprintf(stdout,"FlightGear initialising...\n");
            }
            fprintf(stdout,"FlightGear running\n");

            for (;;){
               if( fdm_in.poll_fdm(10.0_s)){
                  fdm_in.update();
                  output_fdm(fdm_in.get_fdm());
                  if (set_controls(telnet_out,joystick_in)){
                     std::this_thread::sleep_until( std::chrono::steady_clock::now() + 20ms);
                  }else{
                     fprintf(stdout,"set controls failed - quitting\n");
                     break;
                  }
               }else{
                  fprintf(stdout,"Lost FlighGear FDM updates");
                  break;
               }
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
  // }
}

