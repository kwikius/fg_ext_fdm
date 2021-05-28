#ifndef FG_EXTERNAL_TEST_FGFS_FDM_IN_HPP_INCLUDED
#define FG_EXTERNAL_TEST_FGFS_FDM_IN_HPP_INCLUDED

#include <sys/socket.h>
#include <autoconv_net_fdm.hpp>

struct fgfs_fdm_in{
   fgfs_fdm_in(const char* hostname, int32_t port);
   ~fgfs_fdm_in();
   void close();
   bool update();
   autoconv_FGNetFDM const & get_fdm()const { return fdm;}
private:
   autoconv_FGNetFDM fdm;
   int m_socket_fd;
   sockaddr_in m_address;
};

#endif // FG_EXTERNAL_TEST_FGFS_FDM_IN_HPP_INCLUDED
