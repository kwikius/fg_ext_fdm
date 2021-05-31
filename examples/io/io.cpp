
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
#include <joystick_control_source.hpp>
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

namespace {

   /**
    * @brief user defined time literal e.g 100_us
    **/
   QUAN_QUANTITY_LITERAL(time,us);
   QUAN_QUANTITY_LITERAL(time,s);

   // indirect system floating point type e.g for microcontrollers
   using float_type = quan::quantity_traits::default_value_type;
 
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

struct joystick_t{

  joystick_t( const char * path)
  : m_js{path}
  , roll{m_js}
  , pitch{m_js}
  , yaw{m_js}
  , throttle{m_js}
  {}
private:
   quan::joystick m_js;
public:
   joystick_control_source<FlightDimension::Roll> const roll;
   joystick_control_source<FlightDimension::Pitch> const pitch;
   joystick_control_source<FlightDimension::Yaw> const yaw;
   joystick_control_source<FlightDimension::Throttle> const throttle;
};

namespace {

   void set_flight_controls( flight_controller & fc, joystick_t const & js)
   {
      fc.set_roll_source(js.roll);
      fc.set_pitch_source(js.pitch);
      fc.set_yaw_source(js.yaw);
      fc.set_throttle_source(js.throttle);
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

            joystick_t js("/dev/input/js0");

            fgfs_fdm_in fdm_in("localhost",5600);

            // wait for FlightGear to start sending packets
            while ( !fdm_in.poll_fdm(1.0_s) ){
               fprintf(stdout, "Waiting for FlightGear to start...\n");
            }

            // Once FlightGear is running, telnet setup should succeed
            fgfs_telnet telnet_out("localhost", 5501);

            // OK have joystick input, fdm and telnet so start flight controller
            flight_controller fc;

            set_flight_controls(fc,js);

            /// @brief store current values of controls
            float_type flight_controls_cache[8] = {0.0};

            /// @brief only set a property if its value changed
            auto set_control = [&telnet_out] ( const char * prop, float_type const & latest, float_type& cached)
            {
               if ( latest == cached){
                  return true;
               }else{
                  cached = latest;
                  return telnet_out.set(prop,latest);
               }
            };
  
            /// @brief so output all the modified flight controls to FlightGear
            auto set_controls = [ &set_control, &fc, &flight_controls_cache ]() -> bool
            {
               try {
                  return 
                     set_control("/controls/flight/aileron",fc.get_roll(),flight_controls_cache[static_cast<int>(FlightDimension::Roll)]) &&
                     set_control("/controls/flight/elevator",fc.get_pitch(),flight_controls_cache[static_cast<int>(FlightDimension::Pitch)]) &&
                     set_control("/controls/flight/rudder",fc.get_yaw(),flight_controls_cache[static_cast<int>(FlightDimension::Yaw)]) &&
                     set_control("/controls/engines/engine[0]/throttle",fc.get_throttle(),flight_controls_cache[static_cast<int>(FlightDimension::Throttle)]);
               }catch(...){
                  fprintf(stderr,"set controls failed\n");
                  return false;
               }
            };
            // but FlighGear needs time to warm up
            while(!fdm_in.poll_fdm(1.0_s)){
                fprintf(stdout,"FlightGear initialising...\n");
            }
            fprintf(stdout,"FlightGear running\n");

            for (;;){
               auto const now = std::chrono::steady_clock::now();
               /**
                 * @brief We should run at Flightgear fdm update rate, set on cmdline
                 * Here is the elastic, if FlightGear is late
               **/
               if( fdm_in.poll_fdm(10.0_s)){
                  fdm_in.update();
                  output_fdm(fdm_in.get_fdm());
                  if (!set_controls()){
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

