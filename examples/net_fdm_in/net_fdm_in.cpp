
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <iostream>

#include <quan/out/angle.hpp>
#include <quan/fs/get_file_dir.hpp>

#include <autoconv_net_fdm.hpp>

/*
  use socket interface to read fdm state of FlightGear
*/

namespace {
  
   /**
     * @brief setup udp socket
     * @return success if socket is greater than equa to  0
   **/
   int setup_socket();
   int read_fdm(int socket_fd, autoconv_FGNetFDM & fdm);
   void output_fdm(autoconv_FGNetFDM const & fdm);
}

int main(int argc, char *argv[])
{

   int pid = fork();
   if (pid == 0){
      ///@brief run flightgear in child process
      auto const path = quan::fs::get_file_dir(argv[0]) + "/exec_flightgear.sh";
      return system(path.c_str());
   }else{
      if ( pid > 0){
         int socket_fd = -1;
         try {
            fprintf(stdout, "Flightgear net_fdm in\n");

            socket_fd = setup_socket();

            autoconv_FGNetFDM fdm;

            for (;;){
               read_fdm(socket_fd,fdm);
               output_fdm(fdm);
            }
            close(socket_fd);
         } catch (const char s[]) {
            std::cerr << "Error: " << s << ": " << strerror(errno) << std::endl;
            close(socket_fd);
            return EXIT_FAILURE;
         } catch (std::exception & e){
            std::cerr << "Error: " << e.what() << std::endl;
            close(socket_fd);
            return EXIT_FAILURE;
         } catch (...) {
            std::cerr << "Error: unknown exception" << std::endl;
            close(socket_fd);
            return EXIT_FAILURE;
         }
      }else{
         std::cout << "fork failed\n";
         return -1;
      }
   }
}

namespace {

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

namespace{
   
   sockaddr_in address = {0};

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
      static constexpr int32_t port = 5500;
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
}

