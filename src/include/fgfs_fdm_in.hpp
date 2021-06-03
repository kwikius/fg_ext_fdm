#ifndef FG_EXTERNAL_TEST_FGFS_FDM_IN_HPP_INCLUDED
#define FG_EXTERNAL_TEST_FGFS_FDM_IN_HPP_INCLUDED

#include <sys/socket.h>
#include <autoconv_net_fdm.hpp>
#include <quan/time.hpp>

struct fgfs_fdm_in{
   fgfs_fdm_in(const char* hostname, int32_t port);
   ~fgfs_fdm_in();
   /**
    * @brief explicitly close the socket. (done automatically in destructor)
    **/
   void close();
   /**
    * @briefget the latest version of the fdm from flighgear
    * blocks indefinitely
    *
    **/
   bool update();

   /**
    * @brief check if new fdm data is available from flightgear
    * @return true if data is available (update() would not block) else false
    **/ 
   bool poll(quan::time::s const & t)const;
   autoconv_FGNetFDM const & get_fdm()const { return fdm;}
private:
   autoconv_FGNetFDM fdm;
   int m_socket_fd;
   sockaddr_in m_address;
};

#endif // FG_EXTERNAL_TEST_FGFS_FDM_IN_HPP_INCLUDED
