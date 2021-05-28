



#include <stdio.h>
#include <errno.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <autoconv_net_fdm.hpp>
#include <quan/out/angle.hpp>

/*
  use socket to read fdm state of FlightGear
  and send ctrls via telnet
*/

#include <cstring>
#include <cstdlib>

#include <iostream>
#include "fgfs_telnet.hpp"
#include "fgfs_fdm_in.hpp"

#include <quan/joystick.hpp>
#include <quan/utility/timer.hpp>

/*
 Control FlightGear using telnet to write properties from joystick
 Read fdm data structure from Flightgear
 Basic requirements for SITL
 derived from https://sourceforge.net/p/flightgear/flightgear/ci/next/tree/scripts/example/fgfsclient.cxx
*/
namespace {

   QUAN_QUANTITY_LITERAL(time,us);

   void usleep(quan::time::us const & t)
   {
      ::usleep(static_cast<unsigned long>(t.numeric_value()));
   }

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
   1,   // pitch
   1,   // throttle
   -1   // yaw
   };

   void set_controls(fgfs_telnet & f, quan::joystick & js )
   {
      auto get_js_percent = [&js](int32_t i)->double {
         return static_cast<double>(js.get_channel(i) * js_sign[i]) / joystick_half_range ;
      };
      f.set("/controls/flight/aileron",get_js_percent(roll_idx));
      f.set("/controls/flight/elevator",get_js_percent(pitch_idx));
      f.set("/controls/flight/rudder",get_js_percent(yaw_idx));
      f.set("/controls/engines/engine[0]/throttle", get_js_percent(throttle_idx) + 1.0);
   }
}

namespace {

   void output_fdm(autoconv_FGNetFDM const & fdm)
   {
      // the double cast is necessary since the variables are encapsulated in a network_variable
      // which automatically converts from network to host byte order
      // first the value is cast to the exact underlying type of the network varaible
      // then that type can also be converted according to its rules
      quan::angle::deg const roll = static_cast<quan::angle_<float>::rad>(fdm.phi);
      quan::angle::deg const pitch = static_cast<quan::angle_<float>::rad>(fdm.theta);
      quan::angle::deg const yaw = static_cast<quan::angle_<float>::rad>(fdm.psi);
      auto const alt =  fdm.altitude; //
      auto const agl = fdm.agl;
      quan::velocity::m_per_s airspeed = static_cast<quan::velocity_<float>::knot>(fdm.vcas);
      fprintf(stdout,"\rr=%6.1f p=%6.1f y=%6.1f",
         roll.numeric_value(),
         pitch.numeric_value(),
         yaw.numeric_value()
      );
      fflush(stdout);
   }
}

int main(const int argc, const char *argv[])
{
   int  socket_fd = 0;
   try {
      fprintf(stdout, "Flightgear io\n");

      fgfs_telnet f("localhost", 5501);
      fgfs_fdm_in fdm_in("localhost",5600);

      quan::joystick js{"/dev/input/js0"};

      quan::timer<> timer;
      quan::time::us t = timer();
      for (;;){
         fdm_in.update();
         output_fdm(fdm_in.get_fdm());
         set_controls(f,js);
         auto const dt = timer() - t;
         t = timer();
         usleep(18000_us - dt );
      }
      
      return EXIT_SUCCESS;

   } catch (const char s[]) {
      close(socket_fd);
      std::cerr << "Error: " << s << ": " << strerror(errno) << std::endl;
      return EXIT_FAILURE;

   } catch (std::exception & e){
      close(socket_fd);
      std::cerr << "Error: " << e.what() << std::endl;
      return EXIT_FAILURE;
   } catch (...) {
      close(socket_fd);
      std::cerr << "Error: unknown exception" << std::endl;
      return EXIT_FAILURE;
   }
}