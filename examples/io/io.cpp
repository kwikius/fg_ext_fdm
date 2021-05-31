
#include <cstring>

#include "fgfs_telnet.hpp"
#include "fgfs_fdm_in.hpp"
#include <flight_control_source.hpp>

#include <quan/joystick.hpp>
#include <quan/utility/timer.hpp>
#include <quan/out/angle.hpp>
#include <quan/fs/get_file_dir.hpp>
#include <quan/constrain.hpp>
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
     * @todo For SITL we really need to separate controls out from joystick
     * dont use joystick indices for controls!
     * controls may come from autopilot
     * Separate raw_joystick , normalised_joystick_controls
     * or separate actual controls . indexes are clumsy
     * Nice to know if control values changed too
     * Control values should be normalised
     * Two types of control values signed and unsigned
     * flight_control.source = joystick or autopilot
   **/

  /**
   * @brief range of raw Taranis joystick input is nominally +- 32767
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
    * @brief joystick channel direction of raw joystick input, 
    * either 1 or -1 dependent on joystick setup
   **/
   static constexpr int32_t js_sign []= {
      1,   // roll
      1,   // pitch
      1,   // throttle
      -1   // yaw
   };

   /**
     * @brief FlightGear flight_dimension outputs can be signed or unsigned.
     * Signed control surface   output range -1.0 to 1.0
     * Unsigned control surface output range  0.0 to 1.0 
    **/
    static constexpr bool control_is_signed []= {
      true,   // roll
      true,   // pitch
      false,  // throttle
      true    // yaw
   };

   /// @brief cache last values sent so we only need to update if changed
   double controls_cache [4] = {0.0,0.0,0.0,0.0};

   /**
    *  @brief Read values from joystick, convert then, if changed,
    *  send control values using FlightGear telnet protocol
    *  to access the flightgear property system.
    *  @return true if successful
    *  @todo. better to send controls from virtual control_source
    **/
   bool set_controls(fgfs_telnet & telnet_out, quan::joystick const & js )
   {
      try{
         /// @brief convert raw joystick value to flightgear value
         auto get_js_percent = [&js](int32_t i)->double {
            if (control_is_signed[i]){
               return quan::constrain(
                  static_cast<double>(js.get_channel(i) * js_sign[i]) / joystick_half_range
                  ,-1.0
                  ,1.0
               );
            }else{
               return quan::constrain(
                 (static_cast<double>(js.get_channel(i) * js_sign[i]) / joystick_half_range + 1.0) / 2.0 
                  ,0.0
                  ,1.0
               );
            }
         };

         /// @brief only send control values to FlightGear if actually changed
         auto out_cached = [&telnet_out,&get_js_percent](const char* prop, int32_t control_idx) -> bool {
            double const v = get_js_percent(control_idx);
            if ( v == controls_cache[control_idx]){
               return true;
            }else{
               controls_cache[control_idx] = v;
               return telnet_out.set(prop,v);
            }
         };

         out_cached("/controls/flight/aileron",roll_idx);
         out_cached("/controls/flight/elevator",pitch_idx);
         out_cached("/controls/flight/rudder",yaw_idx);
         out_cached("/controls/engines/engine[0]/throttle",throttle_idx);

         return true;
      }catch(...){
         // probably flightgear stopped
         fprintf(stderr,"Exception in set_controls\n");
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
      auto const path = quan::fs::get_file_dir(argv[0]) + "/exec_flightgear.sh";
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

            // Once FlightGear is running, telnet setup should succeed
            fgfs_telnet telnet_out("localhost", 5501);
            // but FlighGear needs time to warm up
            while(!fdm_in.poll_fdm(1.0_s)){
                fprintf(stdout,"FlightGear initialising...\n");
            }
            fprintf(stdout,"FlightGear running\n");

            for (;;){
               auto const now = std::chrono::steady_clock::now();
               /**
                 *@brief We should run at Flighgear fdm update rate, set on cmdline
                 * Here is the elastic, if FlightGear is late
               **/
               if( fdm_in.poll_fdm(10.0_s)){
                  fdm_in.update();
                  output_fdm(fdm_in.get_fdm());
                  if (!set_controls(telnet_out,joystick_in)){
                     fprintf(stdout,"set controls failed - quitting\n");
                     break;
                  }
               }else{
                  fprintf(stdout,"FlightGear FDM update more than 10 s late");
               }
               // wake up just before the next fdm packet is available from FlightGear (hopefully!)
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

