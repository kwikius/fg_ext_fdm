

#include <cstring>
#include <cstdlib>

#include <iostream>
#include "fgfs_telnet.hpp"

#include <quan/joystick.hpp>
#include <quan/utility/timer.hpp>

/*
 Control FlightGear using telnet to write properties from joystick
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
      set_float(f,"/controls/flight/aileron",get_js_percent(roll_idx));
      set_float(f,"/controls/flight/elevator",get_js_percent(pitch_idx));
      set_float(f,"/controls/flight/rudder",get_js_percent(yaw_idx));
      set_float(f,"/controls/engines/engine[0]/throttle", get_js_percent(throttle_idx) + 0.5);
   }
}

int main(const int argc, const char *argv[]){
try {
	const char *hostname = argc > 1 ? argv[1] : "localhost";
	int port = argc > 2 ? atoi(argv[2]) : 5501;

	fgfs_telnet f(hostname, port);
	f.flush();

   quan::joystick js{"/dev/input/js0"};

   quan::timer<> timer;
   quan::time::us t = timer();
   for (;;){
      set_controls(f,js);
      auto const dt = timer() - t;
      t = timer();
      usleep(20000_us - dt );
   }
   
	return EXIT_SUCCESS;

} catch (const char s[]) {
	std::cerr << "Error: " << s << ": " << strerror(errno) << std::endl;
	return EXIT_FAILURE;

} catch (...) {
	std::cerr << "Error: unknown exception" << std::endl;
	return EXIT_FAILURE;
}
}