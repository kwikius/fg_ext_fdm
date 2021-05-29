#ifndef FG_EXTERNAL_TEST_FGFS_CLIENT_HPP_INCLUDED
#define FG_EXTERNAL_TEST_FGFS_CLIENT_HPP_INCLUDED

#include <quan/time.hpp>

/*
 derived from https://sourceforge.net/p/flightgear/flightgear/ci/next/tree/scripts/example/fgfsclient.cxx
fgfs_telnet

*/

class fgfs_telnet {
public:

   static constexpr char default_host[] = "localhost";
   static constexpr unsigned default_port = 5501;
   static constexpr size_t default_buffer_size = 256;

	fgfs_telnet(const char *name = default_host, unsigned port = default_port, size_t buffer_size = default_buffer_size);
	~fgfs_telnet();

   bool is_readable(quan::time::s const & t) const;
   bool is_writeable(quan::time::s const & t) const;

   template <typename T>
   bool get(const char* prop, T& val) const;

   template <typename T>
   bool set(const char* prop, T const & val) const;

	bool write(const char *msg, ...)const;
/**
  @return pointer to internal buffer with result or null
*/
	const char* read();
	void flush();
	void settimeout(quan::time_<int32_t>::s t) { m_timeout = t; }
	int  close();
private:
	int		m_sock;
	char *	m_buffer;
   size_t   m_buflen;
	quan::time_<int32_t>::s	m_timeout;
	bool		m_connected;
};

#endif // FG_EXTERNAL_TEST_FGFS_CLIENT_HPP_INCLUDED
