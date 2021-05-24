
#include <time.h>
#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>

#include <autoconv_net_ctrls.hpp>

#include <quan/constrain.hpp>
#include <quan/joystick.hpp>
#include <quan/time.hpp>

/**
*  @brief Try sending in control structure to FlightGear. Unfortunately doesnt work.
*  It appears you have to also send in net_fdm structure.
*  or read modify write the control structure
*  Anyway causes exceptions in FG with nans for gps_pos and alt on cmd line
*@todo try read modify write
**/

namespace{

   static_assert( sizeof(autoconv_FGNetCtrls) == sizeof(FGNetCtrls) ,"");

   QUAN_QUANTITY_LITERAL(time,us);

   void usleep(quan::time::us const & t);
   bool setup_send_socket();
   bool setup_recv_socket();
   bool setup_sockets()
   {
      return setup_send_socket() && setup_recv_socket();
   }

   void run();
   void get_joystick(quan::joystick const & js,autoconv_FGNetCtrls & cs);
   int read_socket(autoconv_FGNetCtrls & cs);
   void write_socket(autoconv_FGNetCtrls const & cs);
   bool key_was_pressed();

}

/*
see https://wiki.flightgear.org/Property_Tree/Sockets
*/

int main(int argc, char ** argv)
{
   if ( setup_sockets()){
     run();
   }
}

namespace{

   int sendSocket = -1;
   struct sockaddr_in sendAddr;
   constexpr char fg_send_ip[] = "127.0.0.1";
   int constexpr fg_send_port = 5600;

   bool setup_send_socket()
   {
      memset(&sendAddr,0,sizeof(sendAddr));
      sendAddr.sin_family = AF_INET;
      sendAddr.sin_port = htons(fg_send_port);

      if ( inet_pton(AF_INET, fg_send_ip, &sendAddr.sin_addr) != 1){
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

   void write_socket(autoconv_FGNetCtrls const & fc)
   {
      sendto(sendSocket,(void *)&fc,sizeof(fc),0,(struct sockaddr *)&sendAddr,sizeof(sendAddr));
   }
}

namespace {

   int recvSocket = -1;
   struct sockaddr_in recvAddr;
   constexpr char fg_recv_ip[] = "127.0.0.1";
   int constexpr fg_recv_port = 5700;

   bool setup_recv_socket()
   {
      memset(&recvAddr,0,sizeof(recvAddr));

      recvAddr.sin_family = AF_INET;
      recvAddr.sin_port = htons(fg_recv_port);
      
      if ( inet_pton(AF_INET,fg_recv_ip, &recvAddr.sin_addr) != 1){
         fprintf(stderr,"convert recv ip address failed\n");
         exit(-1);
      }

      int const recvSocket = socket(AF_INET, SOCK_DGRAM, 0); 

      if (recvSocket == -1){
         perror("open socket failed");
         exit(errno);
      }else{
         fprintf(stdout,"rx socket created\n");
      }

      if (bind(recvSocket, (struct sockaddr *) &recvAddr,sizeof(recvAddr)) == -1){
         perror("bind");
         exit(errno);
      }else{
         fprintf(stdout,"rx socket bound\n");
      }
   }

   /**
   * @return 1 for correct no of bytes read
   * 0 for no bytes read
   * -1 for error
   */
   int read_socket(autoconv_FGNetCtrls & fc)
   {
      socklen_t address_size = sizeof(recvAddr);

      ssize_t const nbytes_read = recvfrom(recvSocket,&fc, sizeof(fc),0,(struct sockaddr*)&recvAddr, &address_size );

      switch (nbytes_read ){
         case sizeof(fc) :
            return 1;
         case 0:
            return 0;
         default:
            fprintf(stderr,"invalid no of bytes(%d) from socket\n",static_cast<int>(nbytes_read));
            return -1;
      }
   }

}

namespace {

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
         1,   // pitch  //n.b diff to net_fdm!
         1,   // throttle
        -1   // yaw
    };

   void get_joystick(quan::joystick const & js,autoconv_FGNetCtrls & fc)
   {
      auto get_js_percent = [&js](int32_t i)->double {
         return quan::constrain(
            static_cast<double>(js.get_channel(i) * js_sign[i]) / joystick_half_range, 
            -1.0, 1.0
         );
      };

      fc.aileron = get_js_percent(roll_idx);
      fc.elevator = get_js_percent(pitch_idx);
      fc.rudder = get_js_percent(yaw_idx);
      fc.throttle[0] = get_js_percent(throttle_idx);
   }


   /**
    * @brief simulation frame period
    * @todo we could provide the option to update in realtime 
    * and record that here
   */
   auto constexpr update_period = 100000.0_us;

   void setup(autoconv_FGNetCtrls & fc)
   {
      fc.num_engines = 1;
      fc.speedup = 1;
      
      fc.engine_ok[0] = 1;
   }

   void run()
   {
      try {

         quan::joystick js{"/dev/input/js0"};
         autoconv_FGNetCtrls  fc;
         setup(fc);

         for(;;){
            if (key_was_pressed()){
               close(recvSocket);
               close(sendSocket);
               //@todo close(js);
               fprintf(stderr, "exiting on keypress\n");
               fflush(stderr);
               exit(0);
            }

            quan::time::us update_time_left = update_period;
            while ( read_socket(fc) <= 0){
               usleep(1000.0_us);
               update_time_left -= 1000.0_us;
               if (update_time_left <= 0.0_us){
                  fprintf(stderr, "timeout waiting for fgnetctrl input socket\n");
                  fflush(stderr);
               //exit(1);
               }
            }
            get_joystick(js,fc);
            write_socket(fc);
            usleep(update_period);
         }

      }catch(std::runtime_error & e){
         close(recvSocket);
         close(sendSocket);
         fprintf(stderr,"Exception : %s\n",e.what());
         exit(1);
      }
   }

   /**
    * wrap usleep with quan::time type
   **/
   void usleep(quan::time::us const & t)
   {
      ::usleep(static_cast<unsigned long>(t.numeric_value()));
   }

   bool key_was_pressed()
   {
     termios oldt;
     tcgetattr(STDIN_FILENO, &oldt);

     termios newt = oldt;
     newt.c_lflag &= ~(ICANON | ECHO);
     tcsetattr(STDIN_FILENO, TCSANOW, &newt);

     int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);

     fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

     int ch = getchar();
     bool input = ch != EOF;

     tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
     fcntl(STDIN_FILENO, F_SETFL, oldf);

     if(input){
        ungetc(ch, stdin);
     }
     return input;
   }


}