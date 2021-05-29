
#include <cstdarg>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <unistd.h>

#include <netdb.h>
#include <netinet/in.h>

#include <fgfs_telnet.hpp>

/*
 derived from https://sourceforge.net/p/flightgear/flightgear/ci/next/tree/scripts/example/fgfsclient.cxx
*/

namespace {
   QUAN_QUANTITY_LITERAL(time,s)

   template <typename T, typename Where = void>
   struct ll_telnet;

   template <typename T >
   struct ll_telnet<T, typename quan::where_<std::is_floating_point<T> >::type>{

    static float set(fgfs_telnet const & f, const char* prop, T const & val)
    {
        return f.write("set %s %f",prop,static_cast<double>(val));
    }

    static bool get(fgfs_telnet & f, const char* prop, T & val)
    {
         f.write("get %s",prop);
         const char* p = f.read();
         if (p){
           double tmp = atof(p);
           val = static_cast<T>(tmp);
           return true;
         }else{
           return false;
         }
    }
   };

   template <typename T>
   struct ll_telnet<T, typename quan::where_<std::is_integral<T> >::type >{

    static int set(fgfs_telnet const & f, const char* prop,T const & val)
    {
       int32_t v = static_cast<int32_t>(val);
       return f.write("set %s %d",prop,v);
    }

    bool get_int(fgfs_telnet & f, const char* prop, int & val)
    {
         f.write("get %s",prop);
         const char* p = f.read();
         if (p){
           int32_t v = atoi(p);
           val = static_cast<T>(v);
           return true;
         }else{
           return false;
         }
    }

   };

}

fgfs_telnet::fgfs_telnet(const char *hostname, unsigned port,size_t buflen) :
	m_sock{::socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)},
   m_buffer{new char [buflen]{'0'}},
   m_buflen{buflen},
	m_timeout{1_s},
	m_connected{false}
{
	if (m_sock < 0){
      delete[] m_buffer;
	   throw("fgfs_telnet/socket");
   }

	struct hostent* hostinfo = gethostbyname(hostname);
	if (!hostinfo) {
		close();
      delete[] m_buffer;
		throw("fgfs_telnet/gethostbyname: unknown host");
	}

	struct sockaddr_in serv_addr;
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(port);
	serv_addr.sin_addr = *(struct in_addr *)hostinfo->h_addr;

/**
  * @todo change this to loop trying to connect for a certain time or forever
**/
	if (::connect(m_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
		close();
      delete[] m_buffer;
      m_buffer = nullptr;
		throw("fgfs_telnet/connect");
	}
	m_connected = true;
	try {
		write("data");
      flush();
	} catch (...) {
		close();
      delete[] m_buffer;
      m_buffer = nullptr;
		throw;
	}
}

fgfs_telnet::~fgfs_telnet()
{
   close();
   delete[] m_buffer;
}

int fgfs_telnet::close(void)
{
	if (m_connected){
		write("quit");
   }
	if (m_sock < 0){
		return 0;
   }
	int const ret = ::close(m_sock);
	m_sock = -1;
	return ret;
}

bool fgfs_telnet::write(const char *msg, ...)const
{
   {
      fd_set fd;
      struct timeval tv;

      FD_ZERO(&fd);
      FD_SET(m_sock, &fd);
      tv.tv_sec = static_cast<unsigned>(m_timeout.numeric_value());
      tv.tv_usec = 0;
      if (!::select(FD_SETSIZE, 0, &fd, 0, &tv)){
         throw("fgfs_telnet::write/select: timeout exceeded");
      }
   }

	char buf[m_buflen];
   {
      va_list va;
      ::va_start(va, msg);
      ::vsnprintf(buf, m_buflen - 2, msg, va);
      ::va_end(va);
   }
	::strcat(buf, "\015\012");
   ::ssize_t const len_to_write = ::strlen(buf);
	::ssize_t const len_written = ::write(m_sock, buf,len_to_write );
	if (len_written < 0){
		throw("fgfs_telnet::write");
   }
	return len_written == len_to_write;
}

const char * fgfs_telnet::read()
{
   {
      fd_set fd;
      struct timeval tv;
      
      FD_ZERO(&fd);
      FD_SET(m_sock, &fd);
      tv.tv_sec = static_cast<unsigned>(m_timeout.numeric_value());
      tv.tv_usec = 0;
      if (!select(FD_SETSIZE, &fd, 0, 0, &tv)) {
         if (m_timeout == 0_s)
            return nullptr;
         else
            throw("fgfs_telnet::read/select: timeout exceeded");
      }
   }

	ssize_t len = ::read(m_sock, m_buffer, m_buflen - 1);
	if (len < 0)
		throw("fgfs_telnet::read/read");
	if (len == 0)
		return 0;

   char *p;
	for (p = &m_buffer[len - 1]; p >= m_buffer; p--)
		if (*p != '\015' && *p != '\012')
			break;
	*++p = '\0';
	return strlen(m_buffer) ? m_buffer : nullptr;
}

inline void fgfs_telnet::flush(void)
{
	auto const tmp = m_timeout;
	m_timeout = 0_s;
	while (read())
		;
	m_timeout = tmp;
}


template <typename T>
bool fgfs_telnet::get(const char* prop, T& val) const
{
   return ll_telnet<T>::get(*this,prop,val);
}

template <typename T>
bool fgfs_telnet::set(const char* prop, T const & val) const
{
  return ll_telnet<T>::set(*this,prop,val);
}
/**
 * @todo add explicit specialisations as reqd
**/

template bool fgfs_telnet:: set<double>(char const *,double const &) const;
