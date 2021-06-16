


#include <cstring>
#include <cstdlib>
#include <cassert>

#include <chrono>
#include <thread>
#include <iostream>

#include <quan/fs/get_file_dir.hpp>
#include "fgfs_telnet.hpp"
#include "joystick.hpp"

/**
*  
**/

namespace {

  /**
    * @brief Try starting telnet. Keep trying for 20 s
    * @return true if telnet started
   **/
   bool start_telnet();

  /**
    * @brief Get telnet interface. Throws logic error if not available
   **/
   fgfs_telnet & get_telnet();

  /**
    * @brief send joystick control values via telnet
   **/
   void set_controls(fgfs_telnet & telnet, joystick_t const & js );

   /**
     * @brief clean up telnet
    **/
   void stop_telnet();
}

/// @brief required for chrono literals to be found
using namespace std::chrono_literals;

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
            fprintf(stdout, "Flightgear telnet demo\n");

            joystick_t js("/dev/input/js0");
            
            if ( ! start_telnet() ){
               std::cout << "no sign of FlightGear starting.. giving up\n";
               return 1;
            }

            for (;;){
               auto const now = std::chrono::steady_clock::now();
               set_controls(get_telnet(),js);
               std::this_thread::sleep_until(now + 20ms);
            }

            return EXIT_SUCCESS;

         } catch (const char s[]) {
            std::cerr << "Error: " << s << ": " << strerror(errno) << std::endl;
            stop_telnet();
            return EXIT_FAILURE;
         } catch (std::exception & e){

            std::cerr << "Error: " << e.what() << std::endl;
            stop_telnet();
            return EXIT_FAILURE;
         } catch (...) {
            std::cerr << "Error: unknown exception" << std::endl;
            stop_telnet();
            return EXIT_FAILURE;
         }
      }else{
         std::cout << "fork failed\n";
         return -1;
      }
   }
}

namespace {

   void set_controls(fgfs_telnet & telnet, joystick_t const & js )
   {
      telnet.set("/controls/flight/aileron",js.roll.get());
      telnet.set("/controls/flight/elevator",js.pitch.get());
      telnet.set("/controls/flight/rudder",js.yaw.get());
      telnet.set("/controls/engines/engine[0]/throttle", js.throttle.get());
   }
}

namespace{

    fgfs_telnet* ptelnet = nullptr;

    fgfs_telnet& get_telnet()
    {
      if ( ptelnet != nullptr){
         return *ptelnet;
      }else{
         throw std::logic_error("telnet isn't available");
      }
    }

   void stop_telnet()
   {
      delete ptelnet;
      ptelnet = nullptr;
   }

   bool start_telnet()
   {
      if ( ptelnet){
         return true;
      }
      auto const start = std::chrono::steady_clock::now();
      while ( (std::chrono::steady_clock::now() - start) < 20s){
         try {
            ptelnet = new fgfs_telnet("localhost",5501);
            break;
         }catch(const char s[]){
            assert(ptelnet == nullptr);
            std::cout << "Waiting for flightgear to start...\n";
            std::this_thread::sleep_for(2s);
         }
      }
      return ptelnet != nullptr;
   }
}