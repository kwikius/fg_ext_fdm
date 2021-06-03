
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <netdb.h>

#include <fgfs_fdm_in.hpp>

fgfs_fdm_in::fgfs_fdm_in(const char* hostname, int32_t port)
:fdm{},
 m_socket_fd{::socket(AF_INET, SOCK_DGRAM, 0)}, // 
 m_address{}
{
   if (m_socket_fd == -1){
      ::perror("open socket failed");
      ::exit(errno);
   }

   struct hostent* hostinfo = gethostbyname(hostname);
   if (!hostinfo) {
      close();
      throw("fgfs_fdm_in/gethostbyname: unknown host");
   }

   m_address.sin_family = AF_INET;
   m_address.sin_port = htons(port);
   m_address.sin_addr = *(struct in_addr *)hostinfo->h_addr;

   if (bind(m_socket_fd, (struct sockaddr *) &m_address,sizeof(m_address)) == -1){
      close();
      ::perror("bind");
      ::exit(errno);
   }else{
      ::fprintf(stdout,"fdm in socket created\n");
   }
}

/**
int select(int nfds, fd_set *readfds, fd_set *writefds,
           fd_set *exceptfds, struct timeval *timeout);
**/

bool fgfs_fdm_in::poll(quan::time::s const & time_to_wait)const
{
   fd_set fds;
   struct timeval tv;

   FD_ZERO(&fds);
   FD_SET(m_socket_fd, &fds);
   
   tv.tv_sec = static_cast<unsigned>(time_to_wait.numeric_value()); // integer part
   quan::time::us const tus = time_to_wait - quan::time::s{ static_cast<int>(tv.tv_sec)}; // microsec part
   tv.tv_usec = static_cast<unsigned>(tus.numeric_value());

   switch( ::select(FD_SETSIZE, &fds, 0, 0, &tv) ){
      case 1:
         return true;
      case 0:
         return false;
      case -1:
         throw("fgfs_fdm_in/poll bad select");
      default:
         throw("fgfs_fdm_in/poll unknown select");
   }
}

bool fgfs_fdm_in::update()
{
   socklen_t address_size = sizeof(m_address);

   // blocking read
   ssize_t const nbytes_read = ::recvfrom(m_socket_fd,&fdm, sizeof(fdm),0,(struct sockaddr*)&m_address, &address_size );
   if( nbytes_read == sizeof(fdm) ){
       return true;
   }else{
      if ( nbytes_read < 0){
         ::perror("error in recvfrom\n");
         exit(errno);
      }else {
         if ( nbytes_read == 0){
            ::fprintf(stderr,"recvfrom socket no bytes read\n");
            
         }else{
            ::fprintf(stderr,"unknown error\n");
         }
      }
   }
   exit(errno);
}

void fgfs_fdm_in::close()
{
   ::close(m_socket_fd);
}

fgfs_fdm_in::~fgfs_fdm_in()
{
   close();
}


