


#include <quan/config.hpp>

// Linux impl
#include <termios.h>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <flight_mode.hpp>

// from
// http://stackoverflow.com/questions/22166074/is-there-a-way-to-detect-if-a-key-has-been-pressed
namespace {

   /**
    * @brief get any input from keyboard
    * @return keyboard char or EOF if no key pressed
   **/
   int get_keyboard()
   {
     termios oldt;
     tcgetattr(STDIN_FILENO, &oldt);

     termios newt = oldt;
     newt.c_lflag &= ~(ICANON | ECHO);
     tcsetattr(STDIN_FILENO, TCSANOW, &newt);

     int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);

     fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

     int ch = getchar();

     tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
     fcntl(STDIN_FILENO, F_SETFL, oldf);

     return ch;
   }
}


namespace {
  flight_mode m_flight_mode = flight_mode::Manual;
}


flight_mode get_flight_mode()
{
   int key = get_keyboard();
   auto old_mode = m_flight_mode;
   switch( key){
     case EOF:
         break;
     case 'a':
       if ( old_mode != flight_mode:: StraightnLevel){
         m_flight_mode = flight_mode:: StraightnLevel;
         fprintf(stderr,"Flightmode changed to StraightnLevel\n");
       }
       break;
     case ' ':
        if ( old_mode != flight_mode:: Manual){
           m_flight_mode = flight_mode:: Manual;
           fprintf(stderr,"Flightmode changed to Manual\n");
        }
     default:
        fprintf(stderr,"unknown key press %d\n",key);
       break;
   }
   return m_flight_mode;
}


