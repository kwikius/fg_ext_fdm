



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
#include "fgfsclient.hpp"

#include <quan/joystick.hpp>
#include <quan/utility/timer.hpp>

/*
 Control FlightGear using telnet to write properties from joystick
 derived from https://sourceforge.net/p/flightgear/flightgear/ci/next/tree/scripts/example/fgfsclient.cxx

Read fdm from Flightgear also

  
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

   void set_controls(FGFSSocket<256U> & f, quan::joystick & js )
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


namespace {
   
   sockaddr_in address = {0};
   /*
     return socket id
   */
   
   int setup_socket()
   {
      int const socket_fd = socket(AF_INET, SOCK_DGRAM, 0); 

      if (socket_fd == -1){
         perror("open socket failed");
         exit(errno);
      }else{
         fprintf(stdout,"socket created\n");
      }

      // ip address as a string
      const char ip_address_str[] = "127.0.0.1";
      static constexpr int32_t port = 5600;
      {
         address.sin_family = AF_INET;
         address.sin_port = htons(port);
         
         if ( inet_pton(AF_INET, ip_address_str, &address.sin_addr) != 1){
            fprintf(stderr,"convert ip address failed\n");
            exit(-1);
         }

         if (bind(socket_fd, (struct sockaddr *) &address,sizeof(address)) == -1){
            perror("bind");
            exit(errno);
         }
      }
      return socket_fd;
   }

   int read_fdm(int socket_fd, autoconv_FGNetFDM & fdm)
   {
      socklen_t address_size = sizeof(address);

      ssize_t nbytes_read = recvfrom(socket_fd,&fdm, sizeof(fdm),0,(struct sockaddr*)&address, &address_size );
      if( nbytes_read == sizeof(fdm) ){
          return 1;
      }else{
         if ( nbytes_read < 0){
            perror("error in recvfrom\n");
            exit(errno);
         }else {
            if ( nbytes_read == 0){
               fprintf(stderr,"recvfrom socket no bytes read\n");
            }else{
               fprintf(stderr,"unknown error\n");
            }
         }
      }
      exit(errno);
   }

   void output_fdm(autoconv_FGNetFDM const & fdm)
   {
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

      const char *hostname = argc > 1 ? argv[1] : "localhost";
      int port = argc > 2 ? atoi(argv[2]) : 5501;
      fprintf(stdout, "setting up telnet\n");
      FGFSSocket<256U> f(hostname, port);
      fprintf(stdout, "telnet setup\n");
      f.flush();
      socket_fd = setup_socket();
      quan::joystick js{"/dev/input/js0"};
      autoconv_FGNetFDM fdm;

      quan::timer<> timer;
      quan::time::us t = timer();
      for (;;){
         read_fdm(socket_fd,fdm);
         output_fdm(fdm);
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

   } catch (...) {
      close(socket_fd);
      std::cerr << "Error: unknown exception" << std::endl;
      return EXIT_FAILURE;
   }
}