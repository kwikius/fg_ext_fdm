

#include <cstring>
#include <cstdlib>

#include <iostream>
#include "fgfsclient.hpp"

#include <quan/joystick.hpp>
#include <quan/utility/timer.hpp>

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

    int set_float(FGFSSocket<256U> & f, const char* prop, double const & val)
    {
        return f.write("set %s %f",prop,val);
    }

    bool get_float(FGFSSocket<256U> & f, const char* prop, double & val)
    {
         f.write("get %s",prop);
         const char* p = f.read();
         if (p){
           val = atof(p);
           return true;
         }else{
           return false;
         }
    }

    int set_int32(FGFSSocket<256U> & f, const char* prop,int const & val)
    {
       return f.write("set %s %d",prop,val);
    }

    bool get_int(FGFSSocket<256U> & f, const char* prop, int & val)
    {
         f.write("get %s",prop);
         const char* p = f.read();
         if (p){
           val = atoi(p);
           return true;
         }else{
           return false;
         }
    }

   void set_controls(FGFSSocket<256U> & f)
   {
      set_float(f,"/controls/flight/aileron",get_js_percent(roll_idx));
      set_float(f,"/controls/flight/elevator",get_js_percent(pitch_idx));
      set_float(f,"/controls/flight/rudder",get_js_percent(yaw_idx));
      set_float(f,"/controls/engines/engine[0]/throttle", get_js_percent(throttle_idx) + 0.5);
   }

   /*
      get
      Compass vector     attitude 
      GPS vector
      accelerometer vector
      gyro vector
      barometer
      airspeed
      windspeed vector
   */
}


int main(const int argc, const char *argv[])
try {
	const char *hostname = argc > 1 ? argv[1] : "localhost";
	int port = argc > 2 ? atoi(argv[2]) : 5501;

	FGFSSocket<256U> f(hostname, port);
	f.flush();

   quan::joystick js{"/dev/input/js0"};

   auto get_js_percent = [&js](int32_t i)->double {
      return static_cast<double>(js.get_channel(i) * js_sign[i]) / joystick_half_range ;
   };

   quan::timer<> timer;
   quan::time::us t = timer();
   for (;;){
      set_controls(f);
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