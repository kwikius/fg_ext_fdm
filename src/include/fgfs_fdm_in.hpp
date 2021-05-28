#ifndef FG_EXTERNAL_TEST_FGFS_FDM_IN_HPP_INCLUDED
#define FG_EXTERNAL_TEST_FGFS_FDM_IN_HPP_INCLUDED

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

struct fgfs_fdm_in{

    fgfs_fdm_in(const char* hostname, int32_t port)
    :fdm{},m_socket_fd{::socket(AF_INET, SOCK_DGRAM, 0)}, m_address{}
    {
      if (m_socket_fd == -1){
         perror("open socket failed");
         ::exit(errno);
      }else{
         ::fprintf(stdout,"socket created\n");
      }


   struct hostent* hostinfo = gethostbyname(hostname);
	if (!hostinfo) {
		close();
    //  delete[] m_buffer;
		throw("fgfs_fdm_in/gethostbyname: unknown host");
	}
/*
	struct sockaddr_in serv_addr;
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(port);

*/

      m_address.sin_family = AF_INET;
      m_address.sin_port = htons(port);
//
//      if ( ::inet_pton(AF_INET, ip_address, &m_address.sin_addr) != 1){
//         ::fprintf(stderr,"convert ip address failed\n");
//         exit(-1);
//      }

	   m_address.sin_addr = *(struct in_addr *)hostinfo->h_addr;

      if (bind(m_socket_fd, (struct sockaddr *) &m_address,sizeof(m_address)) == -1){
         close();
         ::perror("bind");
         ::exit(errno);
      }
    }

    ~fgfs_fdm_in()
    {
      close();
    }

    void close()
    {
      ::close(m_socket_fd);
    }

    bool update()
    {
      socklen_t address_size = sizeof(m_address);

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

   autoconv_FGNetFDM const & get_fdm()const { return fdm;}
private:
   autoconv_FGNetFDM fdm;
   int m_socket_fd;
   sockaddr_in m_address;
};

#endif // FG_EXTERNAL_TEST_FGFS_FDM_IN_HPP_INCLUDED
