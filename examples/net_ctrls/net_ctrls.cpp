
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>

#include <net_ctrls.hxx>
#include <byte_order.hpp>

#include <quan/joystick.hpp>
#include <quan/time.hpp>

namespace{

   QUAN_QUANTITY_LITERAL(time,us);

   void usleep(quan::time::us const & t);
   bool setup_socket();


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


    

}

/*
see https://wiki.flightgear.org/Property_Tree/Sockets
*/

int main(int argc, char ** argv)
{
   if ( setup_socket()){
      for(;;){
      // read joystick
      // write socket
        usleep(1'000'000_us);
      }
   }
}


namespace{

   int sendSocket = -1;
   struct sockaddr_in sendAddr;
   constexpr char  fg_ip[] = "127.0.0.1";
   int constexpr fg_port = 5600;

   bool setup_socket()
   {
      memset(&sendAddr,0,sizeof(sendAddr));
      sendAddr.sin_family = AF_INET;
      sendAddr.sin_port = htons(fg_port);

      if ( inet_pton(AF_INET, fg_ip, &sendAddr.sin_addr) != 1){
         fprintf(stderr,"convert ip address failed\n");
         exit(-1);
      }

      sendSocket = socket(AF_INET,SOCK_DGRAM,0);
      if (sendSocket  >= 0){
         return true;
      }else {
         perror("socket() failed");
         exit(errno);
         return false;
      }
   }

   /**
    * wrap usleep with quan::time type
   **/
   void usleep(quan::time::us const & t)
   {
      ::usleep(static_cast<unsigned long>(t.numeric_value()));
   }


}