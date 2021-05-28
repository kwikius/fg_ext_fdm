#ifndef FG_EXTERNAL_TEST_FGFS_CLIENT_HPP_INCLUDED
#define FG_EXTERNAL_TEST_FGFS_CLIENT_HPP_INCLUDED

#include <quan/time.hpp>

/*
 derived from https://sourceforge.net/p/flightgear/flightgear/ci/next/tree/scripts/example/fgfsclient.cxx
*/

class FGFSSocket {
public:

   static constexpr char default_host[] = "localhost";
   static constexpr unsigned default_port = 5501;

	FGFSSocket(const char *name = default_host, unsigned port = default_port, size_t buffer_size = 256);
	~FGFSSocket();

	int write(const char *msg, ...);
	const char *read(void);
	void flush(void);
	void settimeout(quan::time_<int32_t>::s t) { m_timeout = t; }

private:
	int		close(void);
	int		m_sock;
	char *	m_buffer;
   size_t   m_buflen;
	quan::time_<int32_t>::s	m_timeout;
	bool		m_connected;
};

#endif // FG_EXTERNAL_TEST_FGFS_CLIENT_HPP_INCLUDED
